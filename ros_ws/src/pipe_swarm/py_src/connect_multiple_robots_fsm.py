#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu, Range
import math
import re
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from builtin_interfaces.msg import Duration
from enum import Enum, auto
import time
from functools import partial
from collections import defaultdict 
import traceback

# ==========================================================================
# Enums for States
# ==========================================================================

class RobotState(Enum):
    """Overall state of the multi-robot connection process."""
    DISCOVERING_ROBOTS = auto()
    IDLE = auto()
    INITIALIZING_ROBOTS = auto()
    MOVING_ALL_FORWARD = auto()
    GLOBAL_DETECTION_HALT = auto()
    SEQUENTIAL_CONFIRMATION = auto()
    INITIATE_CONNECTION = auto()
    APPROACHING_TARGET = auto()
    CHECKING_ALIGNMENT = auto()
    ALIGNING = auto()
    DOCKING = auto()
    LOCKING = auto()                    
    VERIFYING_CONNECTION = auto()      
    CONNECTED = auto()                  
    HANDLING_FAILURE = auto()           
    RESETTING = auto()
    ALL_CONNECTED_OR_HALTED = auto()
    FAILED = auto()

class AgentStatus(Enum):
    """Individual status of a single robot."""
    UNKNOWN = auto()
    INACTIVE = auto()               # Discovered but deliberately not participating
    ACTIVE = auto()                 # Moving or ready to move
    DETECTED_SOMETHING = auto()     # Triggered the halt
    CONFIRMATION_TARGET = auto()    # Being moved for confirmation check
    CONNECTION_SEEKER = auto()      # Actively trying to connect (approaching, aligning, docking, locking, verifying)
    CONNECTION_TARGET = auto()      # Being targeted for connection (waiting, aligning, docking)
    CONNECTED = auto()              # Part of a stable connection
    OBSTACLE_NEAR = auto()          # Detected an obstacle, will not move
    RESETTING = auto()              # Involved in the reset procedure
    FAILED = auto()                 # Individual failure (less used)

# ==========================================================================
# Main Node Class
# ==========================================================================

class Connect_Robots(Node):
    """
    Manages the discovery, coordination, and connection of multiple robots.
    Robots move forward until one detects an object. They halt, confirm if
    it's another robot, and then initiate a pairwise connection sequence.
    Handles connected chains by moving/stopping the entire target group.
    """
    def __init__(self):
        super().__init__("connect_robots_fsm_node")

        # --- Discovery ---
        self.robots = {}                  
        self.discovered_agent_ids = set()
        self.num_discovered_robots = 0
        self.discovery_complete = False
        self.discovery_timer_period = 2.0
        self.discovery_attempts = 0
        self.max_discovery_attempts = 2

        # --- Dynamic Pub/Sub/Data Storage ---
        self.agent_pubs = {'cmd_vel': {}, 'male_joint': {}, 'female_joint': {}}
        self.agent_subs = {'imu': {}, 'range': {}, 'alignment': {}, 'male_joint_state': {}}
        # Store latest sensor data keyed by agent_id, defaulting nested keys
        self.agent_sensor_data = defaultdict(lambda: defaultdict(lambda: None))
        self.agent_last_cmd_vel = {} # {agent_id: (linear, angular)}

        # --- Multi-Robot State Tracking ---
        self.robot_status = {}           # {agent_id: AgentStatus}
        self.initial_scan_ranges = {}    # {agent_id: float | None} Stores range when MOVING started
        self.connected_pairs = set()     # Stores tuples {(id1, id2), ...} of connected robots

        # --- Temporary Variables for State Transitions ---
        # For Detection/Confirmation
        self.detecting_agent_id = None      # ID of agent that triggered GLOBAL_DETECTION_HALT
        self.detection_range = None         # Range value at the moment of detection
        self.confirmation_target_ids = []   # List of agent IDs to check sequentially
        self.confirmation_check_index = 0   # Index for the list above
        self.confirmation_start_time = None # Time when current confirmation check started
        # For Connection Sequence
        self.current_seeker_id = None       # Agent actively trying to connect
        self.current_target_id = None       # Agent being targeted for connection (confirmed ID)
        self.current_male_id = None         # Cache male role for current pair (seeker)
        self.current_female_id = None       # Cache female role for current pair (target)
        self.locked_connection_range = None # Range stored just before verification pull
        self.current_target_group = set()     # <<< Stores IDs in the target's connected group

        # --- State Machine ---
        self.current_state = RobotState.DISCOVERING_ROBOTS
        self.state_enter_time = self.get_clock().now()
        self.get_logger().info(f"Node starting in state: {self.current_state.name}")

        # --- Constants and Configuration ---
        # Timing
        self.initial_delay_sec = 2.0
        self.alignment_timeout_sec = 40.0
        self.docking_duration_sec = 16.0
        self.locking_timeout_sec = 12.0
        self.verification_duration_sec = 5.0
        self.confirmation_timeout_per_robot_sec = 2.0
        self.num_active_robots = 4
        # Speeds
        self.forward_speed = 0.1
        self.approach_speed = 0.05
        self.docking_speed_male = 0.005
        self.docking_speed_female = -0.0075
        self.docking_speed_group = -0.02
        self.verification_pull_speed = -0.05
        self.reset_speed_seeker = -0.02
        self.reset_speed_target = 0.01
        self.confirmation_move_speed = 0.05
        self.aligning_turn_speed = 0.15
        # Thresholds & Distances
        self.robot_detection_threshold = 0.15
        self.confirmation_detection_threshold = 0.2
        self.connection_verification_threshold = 0.1
        self.close_distance_m = 0.15
        self.reset_distance_m = 0.2
        self.range_valid_min_m = 0.05
        self.roll_threshold_deg = 1.5
        # Joints
        self.male_lock_angle = math.pi / 2.0
        self.male_unlock_angle = 0.0
        self.female_lift_angle = 0.0
        self.joint_pos_tolerance = 0.1
        # Control
        self.kp_centering_forward = -0.05

        # --- Timers ---
        self.get_logger().info("Creating discovery timer...")
        self.discovery_timer = self.create_timer(self.discovery_timer_period, self.discover_robots)
        if self.discovery_timer:
            self.get_logger().info("Discovery timer created successfully.")
        else:
            self.get_logger().error("Failed to create discovery timer!")

        self.fsm_loop_timer = self.create_timer(0.1, self.state_machine_tick) # 10 Hz

        self.get_logger().info("Connect_Robots FSM node initialized, starting discovery.")

    # ==========================================================================
    # Robot Discovery
    # ==========================================================================
    def discover_robots(self):
        """Periodically scans topics to find agent_N robots."""
        # Check if discovery is already marked complete
        if self.discovery_complete:
            return

        self.get_logger().debug(f"Discovery attempt {self.discovery_attempts + 1}/{self.max_discovery_attempts}...")
        found_new = False
        current_ids_found_this_scan = set()

        try:
            topic_list = self.get_topic_names_and_types()
            # Use a more specific topic likely present for each agent
            agent_topic_pattern = re.compile(r'/agent_(\d+)/cmd_vel')

            for topic_name, _ in topic_list:
                match = agent_topic_pattern.match(topic_name)
                if match:
                    agent_id = int(match.group(1))
                    current_ids_found_this_scan.add(agent_id)
                    if agent_id not in self.discovered_agent_ids:
                        self.get_logger().info(f"Discovered new agent_{agent_id}")
                        # Check if already registered (e.g., from previous partial discovery)
                        if agent_id not in self.robots:
                            self._register_robot(agent_id)
                        self.discovered_agent_ids.add(agent_id)
                        found_new = True

            self.num_discovered_robots = len(self.discovered_agent_ids)

        except Exception as e:
            self.get_logger().error(f"Error during robot discovery scan: {e}", exc_info=True)

        # Check for stability (N attempts with no new robots)
        if not found_new and self.num_discovered_robots > 0:
            self.discovery_attempts += 1
            self.get_logger().info(f"Discovery stable ({self.discovery_attempts}/{self.max_discovery_attempts}). Found {self.num_discovered_robots} robots: {sorted(list(self.discovered_agent_ids))}")
            if self.discovery_attempts >= self.max_discovery_attempts:
                self.get_logger().info("Discovery complete.")
                self.discovery_complete = True
                # Cancel timer here once complete
                if self.discovery_timer:
                     self.discovery_timer.cancel()
        elif found_new:
            self.discovery_attempts = 0 # Reset stability counter
            self.get_logger().info(f"Found new robot(s). Current count: {self.num_discovered_robots}. Resetting stability counter.")
        else: # No robots found yet
            self.discovery_attempts += 1
            self.get_logger().info(f"No robots found yet (attempt {self.discovery_attempts}/{self.max_discovery_attempts}).")
            if self.discovery_attempts >= self.max_discovery_attempts:
                 self.get_logger().error("Discovery timed out. No robots found.")
                 self.discovery_complete = True # Mark complete even if failed
                 # Cancel timer here on timeout
                 if self.discovery_timer:
                     self.discovery_timer.cancel()


    def _register_robot(self, agent_id):
        """Creates publishers, subscribers, and initializes data for a newly discovered robot."""
        if agent_id in self.robots:
            self.get_logger().warn(f"Attempted to re-register agent {agent_id}.")
            return

        ns = f"agent_{agent_id}"
        self.robots[agent_id] = {'ns': ns}
        self.get_logger().info(f"Registering resources for {ns}")

        # Initialize sensor data storage using agent_id keys
        self.agent_sensor_data['roll'][agent_id] = 0.0
        self.agent_sensor_data['pitch'][agent_id] = 0.0
        self.agent_sensor_data['range'][agent_id] = None
        self.agent_sensor_data['aligned'][agent_id] = False
        self.agent_sensor_data['male_locked'][agent_id] = False
        self.agent_sensor_data['male_unlocked'][agent_id] = True # Assume unlocked initially
        self.agent_last_cmd_vel[agent_id] = (0.0, 0.0)
        self.robot_status[agent_id] = AgentStatus.UNKNOWN
        self.initial_scan_ranges[agent_id] = None

        # Create Publishers
        try:
            self.agent_pubs['cmd_vel'][agent_id] = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
            self.agent_pubs['male_joint'][agent_id] = self.create_publisher(JointTrajectory, f"/{ns}/male_joint_trajectory_controller/joint_trajectory", 10)
            self.agent_pubs['female_joint'][agent_id] = self.create_publisher(JointTrajectory, f"/{ns}/female_joint_trajectory_controller/joint_trajectory", 10)
            # Create Subscribers - Use partial to pass agent_id to the callback
            self.agent_subs['imu'][agent_id] = self.create_subscription(Imu, f"/{ns}/imu/out", partial(self.imu_callback, agent_id=agent_id), 10)
            self.agent_subs['range'][agent_id] = self.create_subscription(Range, f"/{ns}/infrared_range", partial(self.range_callback, agent_id=agent_id), 10)
            self.agent_subs['alignment'][agent_id] = self.create_subscription(Bool, f"/{ns}/alignment", partial(self.alignment_callback, agent_id=agent_id), 10)
            self.agent_subs['male_joint_state'][agent_id] = self.create_subscription(JointTrajectoryControllerState, f"/{ns}/male_joint_trajectory_controller/controller_state", partial(self.male_joint_state_callback, agent_id=agent_id), 10)
            self.get_logger().info(f"Successfully registered resources for agent {agent_id}")
        except Exception as e:
            self.get_logger().error(f"Failed to create resources for agent {agent_id}: {e}", exc_info=True)
            # Mark as FAILED if resources couldn't be created
            self.robot_status[agent_id] = AgentStatus.FAILED


    # ==========================================================================
    # State Machine Core
    # ==========================================================================
    def state_machine_tick(self):
        """Main FSM loop, called periodically by a timer."""
        # Check discovery status first
        if not self.discovery_complete:
            # Check if discovery timer is active, if not, discovery might have finished/failed without transition
            if self.discovery_timer is None or self.discovery_timer.is_canceled():
                 self.get_logger().warn("Discovery timer inactive but discovery not marked complete. Attempting final check.")
                 self.discover_robots() # Try one last time
                 # If still not complete after this call, wait for next tick
                 if not self.discovery_complete:
                     self.get_logger().warn("Final discovery check did not complete process. Waiting...")
                     return
            else:
                 self.get_logger().debug("Waiting for discovery to complete...", throttle_duration_sec=5.0)
                 return

        # --- Handle initial state transition after discovery ---
        if self.current_state == RobotState.DISCOVERING_ROBOTS:
            if self.num_discovered_robots >= 2:
                self.change_state(RobotState.IDLE)
                return # Start next tick in IDLE
            elif self.num_discovered_robots > 0:
                self.get_logger().warn("Discovery complete, but less than 2 robots found. Halting.")
                self.change_state(RobotState.ALL_CONNECTED_OR_HALTED)
                return
            else: # No robots found
                self.get_logger().error("Discovery failed or timed out with 0 robots.")
                self.change_state(RobotState.FAILED)
                return


        # --- Check for terminal condition ---
        # (Terminal condition logic remains the same as your provided code)
        process_in_progress_states = {
            RobotState.GLOBAL_DETECTION_HALT, RobotState.SEQUENTIAL_CONFIRMATION,
            RobotState.INITIATE_CONNECTION, RobotState.APPROACHING_TARGET,
            RobotState.CHECKING_ALIGNMENT, RobotState.ALIGNING,
            RobotState.DOCKING, RobotState.LOCKING, RobotState.VERIFYING_CONNECTION,
            RobotState.HANDLING_FAILURE, RobotState.RESETTING
        }
        connection_or_confirmation_in_progress = self.current_state in process_in_progress_states
        active_robots_exist = any(status == AgentStatus.ACTIVE
                                  for aid, status in self.robot_status.items()
                                  if aid in self.discovered_agent_ids)
        current_state_valid_for_check = self.current_state not in [
            RobotState.DISCOVERING_ROBOTS, RobotState.IDLE, RobotState.INITIALIZING_ROBOTS,
            RobotState.FAILED, RobotState.ALL_CONNECTED_OR_HALTED, RobotState.CONNECTED
        ]
        should_enter_final_state = not active_robots_exist and \
                                   not connection_or_confirmation_in_progress and \
                                   current_state_valid_for_check

        self.get_logger().debug(f"Tick Check: State={self.current_state.name}, active={active_robots_exist}, in_prog={connection_or_confirmation_in_progress}, valid={current_state_valid_for_check} => Final={should_enter_final_state}")

        if should_enter_final_state:
             log_reason = f"active_exist={active_robots_exist}, proc_in_prog={connection_or_confirmation_in_progress}"
             self.get_logger().info(f"Condition met for final state ({log_reason}). Entering ALL_CONNECTED_OR_HALTED.")
             self.change_state(RobotState.ALL_CONNECTED_OR_HALTED)
             return

        # --- State Execution ---
        now = self.get_clock().now()
        time_in_state = (now - self.state_enter_time).nanoseconds / 1e9

        state_handler_map = {
            RobotState.IDLE: self._handle_idle_state,
            RobotState.INITIALIZING_ROBOTS: self._handle_initializing_robots_state,
            RobotState.MOVING_ALL_FORWARD: self._handle_moving_all_forward_state,
            RobotState.GLOBAL_DETECTION_HALT: self._handle_global_detection_halt_state,
            RobotState.SEQUENTIAL_CONFIRMATION: self._handle_sequential_confirmation_state,
            RobotState.INITIATE_CONNECTION: self._handle_initiate_connection_state,
            RobotState.APPROACHING_TARGET: self._handle_approaching_target_state,
            RobotState.CHECKING_ALIGNMENT: self._handle_checking_alignment_state,
            RobotState.ALIGNING: self._handle_aligning_state,
            RobotState.DOCKING: self._handle_docking_state,
            RobotState.LOCKING: self._handle_locking_state,
            RobotState.VERIFYING_CONNECTION: self._handle_verifying_connection_state,
            RobotState.CONNECTED: self._handle_connected_state,
            RobotState.HANDLING_FAILURE: self._handle_failure_state,
            RobotState.RESETTING: self._handle_resetting_state,
            RobotState.ALL_CONNECTED_OR_HALTED: self._handle_all_connected_or_halted_state,
            RobotState.FAILED: self._handle_failed_state,
        }

        handler = state_handler_map.get(self.current_state)

        if handler:
            try:
                takes_time_arg = self.current_state in [RobotState.IDLE, RobotState.ALIGNING, RobotState.DOCKING, RobotState.LOCKING, RobotState.VERIFYING_CONNECTION]
                if takes_time_arg:
                     handler(time_in_state)
                else:
                     handler()
            except Exception as e:
                 tb_str = traceback.format_exc()
                 self.get_logger().error(f"Exception in state handler for {self.current_state.name}: {e}\nTraceback:\n{tb_str}")
                 self.change_state(RobotState.FAILED)
        else:
            self.get_logger().error(f"No handler implemented for state: {self.current_state.name}")
            self.change_state(RobotState.FAILED)

    # ==========================================================================
    # State Change Logic
    # ==========================================================================
    def change_state(self, new_state: RobotState):
        """Transitions to a new state, logging the change and resetting temporary variables."""
        if new_state == self.current_state:
            return

        previous_state = self.current_state

        self.get_logger().debug(f"--- change_state START ---")
        self.get_logger().info(f"Changing state from {previous_state.name} to {new_state.name}")
        # Log vars before potential reset
        self.get_logger().debug(f"Vars BEFORE Reset Check: seeker={self.current_seeker_id}, target={self.current_target_id}, group={self.current_target_group}")

        self.current_state = new_state
        self.state_enter_time = self.get_clock().now()

        # --- Reset temporary variables based on state transitions ---

        # Reset detection/confirmation variables ONLY IF exiting the confirmation process
        confirmation_process_states = [RobotState.GLOBAL_DETECTION_HALT, RobotState.SEQUENTIAL_CONFIRMATION]
        prev_in_conf = previous_state in confirmation_process_states
        new_in_conf = new_state in confirmation_process_states
        if prev_in_conf and not new_in_conf:
             self.get_logger().debug(f"Resetting confirmation vars (leaving {previous_state.name})")
             self.detecting_agent_id = None
             self.detection_range = None
             self.confirmation_target_ids = []
             self.confirmation_check_index = 0
             self.confirmation_start_time = None
        # ---

        # Reset connection sequence variables ONLY IF exiting the connection sequence
        connection_sequence_states = [
            RobotState.INITIATE_CONNECTION, RobotState.APPROACHING_TARGET, RobotState.CHECKING_ALIGNMENT,
            RobotState.ALIGNING, RobotState.DOCKING, RobotState.LOCKING, RobotState.VERIFYING_CONNECTION,
            RobotState.HANDLING_FAILURE, RobotState.RESETTING, RobotState.CONNECTED
        ]
        prev_in_conn = previous_state in connection_sequence_states
        new_in_conn = new_state in connection_sequence_states
        if prev_in_conn and not new_in_conn:
             self.get_logger().debug(f"Resetting connection sequence vars (leaving {previous_state.name})")
             self.current_seeker_id = None
             self.current_target_id = None
             self.current_male_id = None
             self.current_female_id = None
             self.locked_connection_range = None
             self.current_target_group = set() # <<< RESET GROUP
        # ---

        self.get_logger().debug(f"Vars AFTER change_state: seeker={self.current_seeker_id}, target={self.current_target_id}, group={self.current_target_group}")
        self.get_logger().debug(f"--- change_state END ---")

    # ==========================================================================
    # State Handling Functions
    # ==========================================================================
    def _handle_idle_state(self, time_in_state):
        """Waits for initial delay and ensures resources are ready."""
        # Ensure all robots discovered have publishers ready
        all_ready = all(agent_id in self.agent_pubs.get('cmd_vel', {})
                        for agent_id in self.discovered_agent_ids)
        if not all_ready:
             self.get_logger().info("Waiting for all cmd_vel publishers to be registered...", throttle_duration_sec=2.0)
             return

        if time_in_state >= self.initial_delay_sec:
            self.get_logger().info("Initial delay complete.")
            self.change_state(RobotState.INITIALIZING_ROBOTS)

    def _handle_initializing_robots_state(self):
        """Sets initial joint positions, status, and gets initial range scans for all robots."""
        self.get_logger().info("Initializing all robots...")

        all_scans_obtained = True

        # --- Determine Active/Inactive based on Target Count ---
        discovered_ids_sorted = sorted(list(self.discovered_agent_ids))
        num_discovered = len(discovered_ids_sorted)
        num_to_activate = self.num_active_robots

        if num_to_activate > num_discovered:
             self.get_logger().error(f"Configuration error: num_active_robots_target ({num_to_activate}) is greater than discovered robots ({num_discovered}). Failing.")
             self.change_state(RobotState.FAILED)
             return

        # Identify which IDs will be active (highest IDs) and inactive (lowest IDs)
        num_to_deactivate = num_discovered - num_to_activate
        inactive_ids = set(discovered_ids_sorted[:num_to_deactivate])
        active_ids = set(discovered_ids_sorted[num_to_deactivate:])

        self.get_logger().info(f"Target Active Robots: {num_to_activate}. Discovered: {num_discovered}.")
        self.get_logger().info(f"Activating Agent IDs: {sorted(list(active_ids))}")
        if inactive_ids:
            self.get_logger().info(f"Deactivating Agent IDs: {sorted(list(inactive_ids))}")
        # --- End Active/Inactive Determination ---

        all_active_scans_obtained = True
        initialization_complete = True

        for agent_id in self.discovered_agent_ids: # Iterate through all discovered
            current_status = self.robot_status.get(agent_id, AgentStatus.UNKNOWN)

            if current_status in [AgentStatus.UNKNOWN, AgentStatus.FAILED]:
                self.get_logger().info(f"  Initializing Agent {agent_id}...")
                self._set_joint(agent_id, 'male', self.male_unlock_angle)
                self._set_joint(agent_id, 'female', self.female_lift_angle)

                if agent_id in active_ids:
                    self.robot_status[agent_id] = AgentStatus.ACTIVE
                    self.initial_scan_ranges[agent_id] = None # Clear previous scan
                    self.get_logger().info(f"    Agent {agent_id} set to ACTIVE")
                else:
                    self.robot_status[agent_id] = AgentStatus.INACTIVE
                    self.initial_scan_ranges[agent_id] = None # Clear previous scan
                    self.get_logger().info(f"    Agent {agent_id} set to INACTIVE")
            elif agent_id in inactive_ids and current_status != AgentStatus.INACTIVE:
                 # Ensure previously active robots that are now inactive get marked correctly
                 self.get_logger().warn(f"  Marking previously non-inactive Agent {agent_id} as INACTIVE.")
                 self.robot_status[agent_id] = AgentStatus.INACTIVE

            # Attempt to get initial scan
            if self.robot_status.get(agent_id) == AgentStatus.ACTIVE:
                if self.initial_scan_ranges.get(agent_id) is None:
                    current_range = self.agent_sensor_data['range'].get(agent_id, None)
                    if current_range is not None and current_range >= self.range_valid_min_m:
                        self.initial_scan_ranges[agent_id] = current_range
                        self.get_logger().info(f"    Stored initial scan range for ACTIVE agent {agent_id}: {current_range:.3f} m")
                    else:
                        self.get_logger().info(f"    Waiting for valid initial range for ACTIVE agent {agent_id}...")
                        all_active_scans_obtained = False # Wait until all ACTIVE robots have scans

        if all_active_scans_obtained:
            # Check if at least one robot is ACTIVE
            if any(self.robot_status.get(aid) == AgentStatus.ACTIVE for aid in self.discovered_agent_ids):
                self.get_logger().info("All active robots initialized and have initial range scans.")
                time.sleep(0.5) # Short delay for joints to potentially start moving
                self.change_state(RobotState.MOVING_ALL_FORWARD)
            else:
                self.get_logger().warn("Initialization complete, but no robots are ACTIVE. Moving to final state.")
                self.change_state(RobotState.ALL_CONNECTED_OR_HALTED)
        # Else: Stay in this state, will retry getting scans on next tick


    def _handle_moving_all_forward_state(self):
        """Moves all ACTIVE robots forward and checks for range changes."""
        detection_triggered = False
        active_agent_found = False
        for agent_id in self.discovered_agent_ids:
            agent_status = self.robot_status.get(agent_id)

            if agent_status == AgentStatus.ACTIVE:
                active_agent_found = True
                self.get_logger().debug(f"MOVING_FWD: Commanding Agent {agent_id} forward.", throttle_duration_sec=5.0)
                self._send_velocity(agent_id, self.forward_speed, 0.0)
                self._centre_robot(agent_id)

                current_range = self.agent_sensor_data['range'].get(agent_id, None)
                initial_range = self.initial_scan_ranges.get(agent_id, None)

                # Get initial range if missing (should normally be set in INIT)
                if initial_range is None:
                    if current_range is not None and current_range >= self.range_valid_min_m:
                        self.initial_scan_ranges[agent_id] = current_range
                        self.get_logger().warn(f"Re-acquired initial scan for agent {agent_id} in MOVING state: {current_range:.3f} m")
                    continue # Skip detection check this tick

                # Check for significant range change
                if current_range is not None:
                    if initial_range > 0.01: # Avoid division by zero
                        difference = abs(current_range - initial_range) / initial_range
                        if difference > self.robot_detection_threshold:
                            self.get_logger().info(f"Agent {agent_id} detected range change! (Range: {current_range:.3f}, Initial: {initial_range:.3f}, Diff: {difference:.2%}). Halting all.")
                            self.detecting_agent_id = agent_id
                            self.detection_range = current_range
                            self.robot_status[agent_id] = AgentStatus.DETECTED_SOMETHING
                            detection_triggered = True
                            break # Process only the first detection per tick
                    else:
                        self.get_logger().warn(f"Agent {agent_id} initial range {initial_range} too small to calculate difference.", throttle_duration_sec=5.0)

        if detection_triggered:
            self.stop_all_robots()
            self.change_state(RobotState.GLOBAL_DETECTION_HALT)
        # Note: The case where no active agents are found is handled by the terminal check in state_machine_tick


    def _handle_global_detection_halt_state(self):
        """Stops all robots and prepares list for sequential confirmation, including connected robots."""

        self.get_logger().info(f"Global halt triggered by Agent {self.detecting_agent_id}. Preparing confirmation checks.")

        # Build list of potential targets: Include ACTIVE, OBSTACLE_NEAR, and CONNECTED robots
        valid_target_statuses = {AgentStatus.ACTIVE, AgentStatus.OBSTACLE_NEAR, AgentStatus.CONNECTED}
        excluded_statuses = {
            AgentStatus.CONNECTION_SEEKER, AgentStatus.CONNECTION_TARGET,
            AgentStatus.RESETTING, AgentStatus.CONFIRMATION_TARGET,
            AgentStatus.FAILED, AgentStatus.INACTIVE
         }

        self.confirmation_target_ids = sorted([
            aid for aid in self.discovered_agent_ids
            if aid != self.detecting_agent_id and \
               self.robot_status.get(aid) in valid_target_statuses and \
               self.robot_status.get(aid) not in excluded_statuses
        ])

        self.get_logger().debug(f"Potential confirmation targets: {self.confirmation_target_ids}")

        if not self.confirmation_target_ids:
             self.get_logger().warn(f"Agent {self.detecting_agent_id} detected something, but no other suitable robots found to check. Assuming obstacle.")
             self._handle_obstacle_outcome()
             return

        # If list is NOT empty, proceed
        self.confirmation_check_index = 0
        self.confirmation_start_time = self.get_clock().now()
        self.get_logger().info(f"Starting sequential confirmation check with {self.confirmation_target_ids[0]}.")
        self.change_state(RobotState.SEQUENTIAL_CONFIRMATION)


    def _handle_sequential_confirmation_state(self):
        """
        Sequentially moves potential targets and checks detector's range.
        On success, identifies target group and proceeds to connection.
        On failure (invalid range, timeout, etc.), transitions directly to FAILED state.
        """
        # --- Initial Check for Valid State Variables ---
        if self.detecting_agent_id is None or not self.confirmation_target_ids or self.confirmation_start_time is None:
            log_msg = "SEQ_CONFIRM Error: Invalid state variables on entry! "
            log_msg += f"Detector={self.detecting_agent_id}, Targets={self.confirmation_target_ids}, StartTime={self.confirmation_start_time}"
            self.get_logger().error(log_msg)
            self.change_state(RobotState.FAILED)
            return

        # --- Check Index Validity ---
        if self.confirmation_check_index >= len(self.confirmation_target_ids):
             self.get_logger().error(f"SEQ_CONFIRM Error: Check index {self.confirmation_check_index} out of bounds for targets {self.confirmation_target_ids}.")
             self.change_state(RobotState.FAILED)
             return

        # --- Get Current Target ---
        target_check_id = self.confirmation_target_ids[self.confirmation_check_index]
        self.get_logger().debug(f"Checking target Agent {target_check_id} (Index {self.confirmation_check_index}).", throttle_duration_sec=1.0)

        # --- Check for Timeout FIRST ---
        time_since_check_started = (self.get_clock().now() - self.confirmation_start_time).nanoseconds / 1e9
        if time_since_check_started > self.confirmation_timeout_per_robot_sec:
             # Get current detector range *at the time of timeout* for logging
             detector_range_at_timeout = self.agent_sensor_data['range'].get(self.detecting_agent_id, "N/A")
             range_str = f"{detector_range_at_timeout:.3f}" if isinstance(detector_range_at_timeout, float) else detector_range_at_timeout
             detection_range_str = f"{self.detection_range:.3f}" if self.detection_range is not None else "None"

             self.get_logger().warn(f"SEQ_CONFIRM Timeout: Check for Agent {target_check_id} timed out. Detector range {range_str} (initial {detection_range_str}).")
             self._send_velocity(target_check_id, 0.0, 0.0) # Stop the timed-out target

             # --- Handle Timeout: Move to Next or Fail ---
             self.robot_status[target_check_id] = AgentStatus.ACTIVE # Revert status simply
             self.confirmation_check_index += 1 # Move to next index

             if self.confirmation_check_index >= len(self.confirmation_target_ids):
                 # All targets checked, none confirmed
                 self.get_logger().warn(f"All potential targets {self.confirmation_target_ids} checked for detector {self.detecting_agent_id}. Assuming obstacle.")
                 self._handle_obstacle_outcome()
             else:
                 # Prepare for the next check
                 next_target_id = self.confirmation_target_ids[self.confirmation_check_index]
                 self.confirmation_start_time = self.get_clock().now() # Reset timer for next target
                 self.current_target_group = set() # Ensure group is clear for next check
                 self.get_logger().info(f"Moving to check next target: {next_target_id} (Index {self.confirmation_check_index}).")
                 # Stay in SEQUENTIAL_CONFIRMATION, let next tick handle next target
             return # <<< Important: Exit function after handling timeout this tick

        # --- If no timeout, proceed with checks and movement ---

        # --- Mark Target (Only if not timed out this tick) ---
        self.robot_status[target_check_id] = AgentStatus.CONFIRMATION_TARGET

        # --- Validate Initial Detector Range ---
        if self.detection_range is None:
             self.get_logger().error(f"SEQ_CONFIRM Error: Initial detection range is None. Cannot proceed.")
             self._send_velocity(target_check_id, 0.0, 0.0)
             self.robot_status[target_check_id] = AgentStatus.ACTIVE # Revert status
             self.change_state(RobotState.FAILED)
             return

        # --- Move Target ---
        self._send_velocity(target_check_id, self.confirmation_move_speed, 0.0)
        self._centre_robot(target_check_id)

        # --- Check for Range Change (Confirmation) ---
        # Re-get detector range *after* moving the target
        detector_current_range_after_move = self.agent_sensor_data['range'].get(self.detecting_agent_id)

        # Validate the range reading after move
        if detector_current_range_after_move is None:
            self.get_logger().warn(f"SEQ_CONFIRM Warning: Detector range became None after moving target {target_check_id}. Assuming no confirmation this tick.")
            # Stay in this state and try again next tick, maybe range will recover
            return

        # Calculate difference only if ranges are valid
        difference = 0.0
        if self.detection_range > 0.01: # Avoid division by zero
             difference = abs(detector_current_range_after_move - self.detection_range) / self.detection_range

        if difference > self.confirmation_detection_threshold:
            # --- Confirmation Success ---
            self.get_logger().info(f"Range changed for detector {self.detecting_agent_id} (Now: {detector_current_range_after_move:.3f}, Initial: {self.detection_range:.3f}, Diff: {difference:.2%}). Confirmed Agent {target_check_id}.")
            self._send_velocity(target_check_id, 0.0, 0.0) # Stop the confirmed target

            confirmed_seeker_id = self.detecting_agent_id
            confirmed_target_id = target_check_id

            target_group = self._find_connected_group(confirmed_target_id)
            if not target_group:
                 self.get_logger().error(f"SEQ_CONFIRM Error: Could not determine group for confirmed target {confirmed_target_id}.")
                 self.robot_status[target_check_id] = AgentStatus.ACTIVE # Revert status
                 self.change_state(RobotState.FAILED)
                 return

            self.get_logger().info(f"Confirmed Agent {confirmed_target_id} belongs to group: {target_group}")

            # Final Check (should always pass here)
            if confirmed_seeker_id is None or confirmed_target_id is None:
                 self.get_logger().error("CRITICAL: Seeker/Target ID None after confirmation success!")
                 self.robot_status[target_check_id] = AgentStatus.ACTIVE # Revert status
                 self.change_state(RobotState.FAILED)
                 return

            self.get_logger().debug(f"SEQ_CONFIRM: Prep Seeker={confirmed_seeker_id}, Target={confirmed_target_id}, Group={target_group}")
            self.current_seeker_id = confirmed_seeker_id
            self.current_target_id = confirmed_target_id
            self.current_target_group = target_group
            self.current_target_partner_id = None # Ensure cleared if removing partner logic

            # Revert status of confirmed robot before initiating connection
            self.robot_status[target_check_id] = AgentStatus.ACTIVE # Set simply to Active
            self.change_state(RobotState.INITIATE_CONNECTION)
            return # Success - Exit

        # If no confirmation and no timeout yet, stay in this state for next tick
        self.get_logger().debug(f"SeqConfirm: No confirmation yet for {target_check_id}. Waiting.", throttle_duration_sec=1.0)


    def _handle_obstacle_outcome(self):
        """Helper to handle when sequential confirmation determines an obstacle."""
        checked_target_ids = list(self.confirmation_target_ids) # Make a copy
        detecting_agent_id_local = self.detecting_agent_id # Store locally

        if detecting_agent_id_local is not None:
             if self.robot_status.get(detecting_agent_id_local) != AgentStatus.OBSTACLE_NEAR:
                self.robot_status[detecting_agent_id_local] = AgentStatus.OBSTACLE_NEAR
                self.get_logger().info(f"Agent {detecting_agent_id_local} marked as OBSTACLE_NEAR.")
                self.initial_scan_ranges[detecting_agent_id_local] = None # Reset range
        else:
             self.get_logger().error("Obstacle outcome handler called without detecting_agent_id!")

        # Ensure robots that were checked but not confirmed are reset to ACTIVE
        self.get_logger().info(f"Resetting status for checked targets after obstacle determination: {checked_target_ids}")
        for checked_id in checked_target_ids:
            if checked_id == detecting_agent_id_local: continue

            current_status = self.robot_status.get(checked_id)
            # Reset if they are not busy/connected/failed
            resettable_statuses = {AgentStatus.CONFIRMATION_TARGET, AgentStatus.ACTIVE, AgentStatus.UNKNOWN}
            if current_status in resettable_statuses:
                self.get_logger().info(f"  Resetting Agent {checked_id} status from {current_status.name if current_status else 'None'} to ACTIVE.")
                self.robot_status[checked_id] = AgentStatus.ACTIVE
            else:
                self.get_logger().info(f"  Skipping status reset for Agent {checked_id} (Status: {current_status.name if current_status else 'None'})")

        # Reset detection variables implicitly via change_state below
        self.change_state(RobotState.MOVING_ALL_FORWARD)


    def _handle_initiate_connection_state(self):
        """Determines roles, commands joints, updates status. Uses target group for stopping."""
        if self.current_seeker_id is None or self.current_target_id is None:
            self.get_logger().error("Entered INITIATE_CONNECTION without seeker/target IDs!")
            self.change_state(RobotState.FAILED)
            return

        seeker_id = self.current_seeker_id
        target_id = self.current_target_id # The one being docked TO
        target_group = self.current_target_group

        # --- Assign roles for the NEW connection ---
        self.current_male_id = seeker_id
        self.current_female_id = target_id

        # --- Log based on group size and stop group members ---
        if len(target_group) > 1:
            other_members = target_group - {target_id} # Get other members excluding the direct target
            self.get_logger().info(f"Initiating connection: Seeker={seeker_id}, Target={target_id}, Other Group Members={other_members} | Male={self.current_male_id}, Female={self.current_female_id}")
            # Ensure other members are stopped initially
            self.get_logger().debug(f"InitConnect: Stopping other group members {other_members}")
            for member_id in other_members:
                 self._send_velocity(member_id, 0.0, 0.0)
                 # Ensure status is correct
                 if self.robot_status.get(member_id) != AgentStatus.CONNECTED:
                     self.get_logger().warn(f"Group Member {member_id} status was not CONNECTED ({self.robot_status.get(member_id)}). Setting.")
                     self.robot_status[member_id] = AgentStatus.CONNECTED
        else:
            self.get_logger().info(f"Initiating connection: Seeker={seeker_id}, Target={target_id} | Male={self.current_male_id}, Female={self.current_female_id}")

        # Stop the specific target robot too
        self._send_velocity(target_id, 0.0, 0.0)

        # Command joints for the connecting pair (Male=Seeker, Female=Target)
        self._set_joint(self.current_male_id, 'male', self.male_unlock_angle)
        self._set_joint(self.current_female_id, 'female', self.female_lift_angle)

        # Update status for the active participants
        self.robot_status[seeker_id] = AgentStatus.CONNECTION_SEEKER
        self.robot_status[target_id] = AgentStatus.CONNECTION_TARGET
        # Other group members remain CONNECTED

        time.sleep(0.5) # Allow time for joints to start moving
        self.change_state(RobotState.APPROACHING_TARGET)

    def _handle_approaching_target_state(self):
        """Moves seeker towards target. Keeps entire target group stopped."""
        if self.current_seeker_id is None or self.current_target_id is None: return self._fail_state_missing_ids("APPROACHING_TARGET")

        seeker_id = self.current_seeker_id
        target_group = self.current_target_group 

        # --- Ensure Target Group is stopped ---
        if not target_group:
             self.get_logger().error("Approaching target but target group is empty!")
             self._send_velocity(seeker_id, 0.0, 0.0)
             self.change_state(RobotState.HANDLING_FAILURE)
             return

        self.get_logger().debug(f"Approach: Stopping target group {target_group}", throttle_duration_sec=5.0)
        for member_id in target_group:
            self._send_velocity(member_id, 0.0, 0.0)

        # Move seeker
        self._centre_robot(seeker_id) # Apply centering to seeker

        current_range_seeker = self.agent_sensor_data['range'].get(seeker_id)
        if current_range_seeker is None or current_range_seeker < self.range_valid_min_m:
             self.get_logger().warn(f"Seeker {seeker_id}'s range lost/invalid ({current_range_seeker}) during approach.", throttle_duration_sec=5.0)
             self._send_velocity(seeker_id, 0.0, 0.0) # Stop seeker
             return

        if current_range_seeker < self.close_distance_m:
            self.get_logger().info(f"Seeker {seeker_id} reached close distance ({current_range_seeker:.3f} m) to target {self.current_target_id}.")
            self._send_velocity(seeker_id, 0.0, 0.0)
            self.change_state(RobotState.CHECKING_ALIGNMENT)
        else:
            _, last_angular = self.agent_last_cmd_vel.get(seeker_id, (0.0, 0.0))
            self._send_velocity(seeker_id, self.approach_speed, last_angular)


    def _handle_checking_alignment_state(self):
        """Checks seeker alignment. Keeps seeker and target group stopped."""
        if self.current_seeker_id is None or self.current_target_id is None: return self._fail_state_missing_ids("CHECKING_ALIGNMENT")

        seeker_id = self.current_seeker_id
        target_group = self.current_target_group

        # --- Ensure Seeker and Target Group are stopped ---
        self._send_velocity(seeker_id, 0.0, 0.0)
        if not target_group:
            self.get_logger().error("Checking alignment but target group is empty!")
            self.change_state(RobotState.HANDLING_FAILURE)
            return
        self.get_logger().debug(f"Check Align: Stopping target group {target_group}", throttle_duration_sec=5.0)
        for member_id in target_group:
            self._send_velocity(member_id, 0.0, 0.0)
        # ---

        self.get_logger().info(f"Checking alignment status (Seeker {seeker_id} sensor)...")
        seeker_aligned = self.agent_sensor_data['aligned'].get(seeker_id, False)
        self.get_logger().debug(f"CHECKING_ALIGNMENT: Seeker {seeker_id} alignment sensor value = {seeker_aligned}")

        if seeker_aligned:
            self.get_logger().info(f"Seeker {seeker_id} aligned with target {self.current_target_id}.")
            self.change_state(RobotState.DOCKING)
        else:
            self.get_logger().info(f"Seeker {seeker_id} not aligned. Aligning target {self.current_target_id}.")
            self.change_state(RobotState.ALIGNING)


    def _handle_aligning_state(self, time_in_state):
        """Rotates the specific target robot. Keeps seeker and rest of target group stopped."""
        if self.current_seeker_id is None or self.current_target_id is None: return self._fail_state_missing_ids("ALIGNING")

        seeker_id = self.current_seeker_id
        target_id = self.current_target_id # Only this robot rotates
        target_group = self.current_target_group
        other_group_members = target_group - {target_id} if target_group else set()

        # --- Ensure Seeker and Other Group Members stay stopped ---
        self._send_velocity(seeker_id, 0.0, 0.0)

        seeker_aligned = self.agent_sensor_data['aligned'].get(seeker_id, False)
        self.get_logger().debug(f"ALIGNING: Seeker {seeker_id} alignment sensor value = {seeker_aligned} (Time in state: {time_in_state:.2f}s)", throttle_duration_sec=0.5)

        if seeker_aligned:
            self.get_logger().info("Alignment successful during maneuver.")
            self._send_velocity(target_id, 0.0, 0.0) # Stop target rotation
            # Ensure others remain stopped before docking
            for member_id in other_group_members:
                self._send_velocity(member_id, 0.0, 0.0)
            self.change_state(RobotState.DOCKING)
            return

        if time_in_state > self.alignment_timeout_sec:
            self.get_logger().error(f"Alignment timed out for pair ({seeker_id}, {target_id})!")
            self._send_velocity(target_id, 0.0, 0.0) # Stop target
            for member_id in other_group_members: # Stop others
                 self._send_velocity(member_id, 0.0, 0.0)
            self.change_state(RobotState.HANDLING_FAILURE)
            return

        # Rotate target robot
        time_in_cycle = time_in_state % 4.0
        angular_z = 0.0
        if time_in_cycle <= 1.0: angular_z = self.aligning_turn_speed
        elif time_in_cycle <= 3.0: angular_z = -self.aligning_turn_speed
        else: angular_z = self.aligning_turn_speed
        for member_id in target_group:
            self._send_velocity(member_id, 0.0, angular_z) # Command rotation to all targets


    def _handle_docking_state(self, time_in_state):
        """Moves the male (seeker) forward and the entire target group backward."""
        # female_id is the target_id being docked TO
        if self.current_male_id is None or self.current_female_id is None: return self._fail_state_missing_ids("DOCKING")

        male_id = self.current_male_id     # This is the seeker
        target_group = self.current_target_group

        # --- Command Velocities ---
        self.get_logger().info(f"Docking Male={male_id}, TargetGroup={target_group}... ({time_in_state:.1f}/{self.docking_duration_sec:.1f})", throttle_duration_sec=1.0)

        # Command Male Forward
        self._send_velocity(male_id, self.docking_speed_male, 0.0)

        # Command Entire Target Group Backward
        if not target_group:
             self.get_logger().error("Docking state reached with empty target group!")
             self.change_state(RobotState.HANDLING_FAILURE)
             return

        # self.get_logger().info(f"Docking: Moving target group {target_group} backward at {self.docking_speed_group:.4f}", throttle_duration_sec=5.0)
        
        if len(target_group) > 1:
            for member_id in target_group:
                self._send_velocity(member_id, self.docking_speed_group, 0.0)
        else:
            for member_id in target_group:
                self._send_velocity(member_id, self.docking_speed_female, 0.0) 

        if time_in_state >= self.docking_duration_sec:
            self.get_logger().info("Docking duration complete. Attempting lock.")
            # Stop motion before locking
            self._send_velocity(male_id, 0.0, 0.0)
            for member_id in target_group:
                self._send_velocity(member_id, 0.0, 0.0)
            self.change_state(RobotState.LOCKING)

    def _handle_locking_state(self, time_in_state):
        """Commands lock while pushing male forward and target group backward."""
         # female_id is the target_id being docked to
        if self.current_male_id is None or self.current_female_id is None: return self._fail_state_missing_ids("LOCKING")

        male_id = self.current_male_id
        target_group = self.current_target_group 

        # --- Apply Gentle Push/Pull ---

        self.get_logger().debug(f"Pushing during LOCKING: Male={male_id}, TargetGroup={target_group}", throttle_duration_sec=1.0)
        # Push Male
        self._send_velocity(male_id, self.docking_speed_male, 0.0)

        # Pull Target Group
        if not target_group:
             self.get_logger().error("Locking state reached with empty target group!")
             self.change_state(RobotState.HANDLING_FAILURE)
             return
        for member_id in target_group:
             self._send_velocity(member_id, self.docking_speed_female, 0.0)

        # Command lock once near the beginning of the state
        if time_in_state < 0.2: # Only command once
            if not self.agent_sensor_data['male_locked'].get(male_id, False):
                 self.get_logger().info(f"Commanding male ({male_id}) joint to lock.")
                 self._set_joint(male_id, 'male', self.male_lock_angle)

        # Check lock status
        male_locked = self.agent_sensor_data['male_locked'].get(male_id, False)
        if male_locked:
            self.get_logger().info(f"Male ({male_id}) lock confirmed by feedback.")
            # Stop pushing before verifying
            self._send_velocity(male_id, 0.0, 0.0)
            for member_id in target_group:
                self._send_velocity(member_id, 0.0, 0.0)
            self.change_state(RobotState.VERIFYING_CONNECTION)
            return

        # Timeout check
        if time_in_state > self.locking_timeout_sec:
            self.get_logger().error(f"Locking timed out for male agent {male_id}!")
            # Stop pushing on failure
            self._send_velocity(male_id, 0.0, 0.0)
            for member_id in target_group:
                self._send_velocity(member_id, 0.0, 0.0)
            self.change_state(RobotState.HANDLING_FAILURE)


    def _handle_verifying_connection_state(self, time_in_state):
        """Pulls the male back slightly. Keeps entire target group stationary."""
         # female_id is the target_id being docked TO
        if self.current_male_id is None or self.current_female_id is None: return self._fail_state_missing_ids("VERIFYING_CONNECTION")

        male_id = self.current_male_id     # Seeker
        target_group = self.current_target_group 

        # --- Ensure Target Group stays stopped ---
        if not target_group:
            self.get_logger().error("Verify state reached with empty target group!")
            self.change_state(RobotState.HANDLING_FAILURE)
            return
        self.get_logger().debug(f"Verify: Stopping target group {target_group}", throttle_duration_sec=2.0)
        for member_id in target_group:
            self._send_velocity(member_id, 0.0, 0.0)
        # ---

        # Store initial range at the start of verification
        current_range_male = self.agent_sensor_data['range'].get(male_id)
        if self.locked_connection_range is None:
            if current_range_male is not None and current_range_male >= self.range_valid_min_m:
                self.locked_connection_range = current_range_male
                self.get_logger().info(f"Verifying connection (Pair: {male_id},{self.current_female_id}). Locked range: {self.locked_connection_range:.3f}m.")
            else:
                # Increased wait time for range as physics might settle
                if time_in_state > 3.0:
                    self.get_logger().error(f"Verification failed - No valid range from male {male_id} at start after waiting.")
                    self.change_state(RobotState.HANDLING_FAILURE)
                return # Wait for valid range

        # --- Pull male back to test lock ---
        self._send_velocity(male_id, self.verification_pull_speed, 0.0)
        self._centre_robot(male_id)
        # ---

        # Check after duration
        if time_in_state >= self.verification_duration_sec:
            self._send_velocity(male_id, 0.0, 0.0) # Stop male pull

            # Re-check range after pull stopped
            current_range_male_after_pull = self.agent_sensor_data['range'].get(male_id)

            # Check verification threshold
            if self.locked_connection_range is None or current_range_male_after_pull is None:
                self.get_logger().error(f"Verify check fail - Invalid range male={current_range_male_after_pull}, stored={self.locked_connection_range}.")
                self.change_state(RobotState.HANDLING_FAILURE)
                return

            if self.locked_connection_range > 0.01: # Avoid division by zero
                difference = abs(current_range_male_after_pull - self.locked_connection_range) / self.locked_connection_range
                self.get_logger().info(f"Verify check: Male range {current_range_male_after_pull:.3f}, Locked range {self.locked_connection_range:.3f}. Diff {difference:.2%}")
                if difference < self.connection_verification_threshold:
                    self.get_logger().info(f"Connection verified for pair ({male_id}, {self.current_female_id})!") # Log uses the direct pair
                    self.change_state(RobotState.CONNECTED)
                else:
                    self.get_logger().error(f"Verification failed for pair ({male_id}, {self.current_female_id})! Range difference {difference:.2%} too large.")
                    self.change_state(RobotState.HANDLING_FAILURE)
            else:
                 self.get_logger().error("Verify check fail - Locked range was too small to verify.")
                 self.change_state(RobotState.HANDLING_FAILURE)


    def _handle_connected_state(self):
        """Marks seeker/target pair as connected, updates statuses, stops involved group."""
        if self.current_seeker_id is None or self.current_target_id is None:
             self.get_logger().error("Entered CONNECTED state without valid seeker/target pair IDs! Returning to MOVING.")
             self.current_target_group = set() # Ensure group is clear on error
             self.change_state(RobotState.MOVING_ALL_FORWARD)
             return

        seeker_id = self.current_seeker_id
        target_id = self.current_target_id
        target_group = self.current_target_group 

        self.get_logger().info(f"Pair ({seeker_id}, {target_id}) successfully connected.")
        if len(target_group) > 1:
             self.get_logger().info(f"  (Target {target_id} was part of group {target_group})")

        # Ensure all involved robots are stopped
        self.get_logger().debug(f"Connected: Stopping Seeker={seeker_id}, TargetGroup={target_group}")
        self._send_velocity(seeker_id, 0.0, 0.0)
        for member_id in target_group: 
             self._send_velocity(member_id, 0.0, 0.0)

        # --- Update status and store pair ---
        self.get_logger().info(f"Updating status for {seeker_id} and {target_id} to CONNECTED.")
        self.robot_status[seeker_id] = AgentStatus.CONNECTED
        self.robot_status[target_id] = AgentStatus.CONNECTED
        # Update status for other group members (should already be CONNECTED, but ensure)
        for member_id in target_group: 
             if member_id != target_id and self.robot_status.get(member_id) != AgentStatus.CONNECTED:
                 self.get_logger().warn(f"Connected State: Group member {member_id} status was not CONNECTED. Setting.")
                 self.robot_status[member_id] = AgentStatus.CONNECTED

        # Store the newly formed connection link
        self.connected_pairs.add(tuple(sorted((seeker_id, target_id))))
        self.get_logger().info(f"Updated Robot Status: {self.robot_status}")
        self.get_logger().info(f"Updated Connected Pairs: {self.connected_pairs}")

        # Transition back to MOVING (group will be cleared by change_state)
        self.change_state(RobotState.MOVING_ALL_FORWARD)


    def _handle_failure_state(self):
        """Unified state for handling failures. Stops seeker and target group. Unlocks male."""
        self.get_logger().error("Handling connection sequence failure.")
        seeker_id = self.current_seeker_id
        target_id = self.current_target_id # Store direct target ID before group potentially clears
        target_group = self.current_target_group 
        male_id = self.current_male_id # Seeker is assumed male

        # Stop involved robots
        self.get_logger().debug(f"Failure: Stopping Seeker={seeker_id}, TargetGroup={target_group}")
        if seeker_id is not None: self._send_velocity(seeker_id, 0.0, 0.0)
        for member_id in target_group:
            self._send_velocity(member_id, 0.0, 0.0)

        # Attempt to unlock male (seeker's) joint
        if male_id is not None:
             self.get_logger().info(f"Attempting to unlock male joint ({male_id}).")
             self._set_joint(male_id, 'male', self.male_unlock_angle)
             time.sleep(0.5) # Give command time to send

        # Set status for involved robots for reset
        if seeker_id is not None: self.robot_status[seeker_id] = AgentStatus.RESETTING
        if target_id is not None: self.robot_status[target_id] = AgentStatus.RESETTING

        self.change_state(RobotState.RESETTING)


    def _handle_resetting_state(self):
            """Moves seeker and direct target apart. Keeps rest of prior target group stopped. Retries connection."""
            local_seeker_id = self.current_seeker_id
            local_target_id = self.current_target_id

            if local_seeker_id is None or local_target_id is None:
                # Recovery logic might need adjustment as group info is lost
                self.get_logger().warn("Entered RESETTING without seeker/target IDs. FAILURE")
                self.change_state(RobotState.FAILED)


            # --- Find group connected to target *now* and stop others ---
            target_group_at_reset = self._find_connected_group(local_target_id)
            other_group_members = target_group_at_reset - {local_target_id}
            partner_chain_stopped = set() # Keep track for final stop

            if other_group_members:
                self.get_logger().debug(f"Reset: Stopping other connected members {other_group_members}", throttle_duration_sec=2.0)
                for member_id in other_group_members:
                    self._send_velocity(member_id, 0.0, 0.0)
                    partner_chain_stopped.add(member_id) # Add to set to stop again later
            # ---
            
            # Use seeker's range to gauge distance between seeker and target
            current_range_seeker = self.agent_sensor_data['range'].get(local_seeker_id)
            valid_range = current_range_seeker if (current_range_seeker is not None and current_range_seeker >= self.range_valid_min_m) else self.reset_distance_m

            self.get_logger().info(f"Resetting pair ({local_seeker_id}, {local_target_id}): Range {valid_range:.3f} / Target {self.reset_distance_m:.3f}", throttle_duration_sec=1.0)

            if valid_range < self.reset_distance_m:
                # Only move seeker and direct target
                self._send_velocity(local_seeker_id, self.reset_speed_seeker, 0.0)
                self._send_velocity(local_target_id, self.reset_speed_target, 0.0)
                self._centre_robot(local_seeker_id)
                self._centre_robot(local_target_id)
            else:
                self.get_logger().info(f"Reset distance reached for pair ({local_seeker_id}, {local_target_id}). Retrying connection.")
                self._send_velocity(local_seeker_id, 0.0, 0.0)
                self._send_velocity(local_target_id, 0.0, 0.0)
                # Ensure partner chain remains stopped
                for member_id in partner_chain_stopped:
                     self._send_velocity(member_id, 0.0, 0.0)

                # Mark involved robots as active, ready for next state
                self.robot_status[local_seeker_id] = AgentStatus.ACTIVE
                self.robot_status[local_target_id] = AgentStatus.ACTIVE
                # Partner status remains CONNECTED

                # Reset temporary connection vars (male/female/lock range)
                # Keep seeker/target/partner IDs for the retry. Group will be recalculated.
                self.current_male_id = None
                self.current_female_id = None
                self.locked_connection_range = None
                # Group is already cleared by change_state

                # Transition back to INITIATE_CONNECTION for this pair
                self.change_state(RobotState.INITIATE_CONNECTION)

    def _handle_all_connected_or_halted_state(self):
        """Final state. Logs status, last velocities, and ensures robots are stopped."""
        self.get_logger().info("All robots are connected or have halted.", once=True)
        self.get_logger().info(f"Final Robot Status: {self.robot_status}", once=True)
        self.get_logger().info(f"Connected Pairs: {self.connected_pairs}", once=True)

        # # Log Last Commanded Velocities (from previous answer)
        # vel_log_msgs = []
        # for agent_id in sorted(list(self.discovered_agent_ids)):
        #     last_lin, last_ang = self.agent_last_cmd_vel.get(agent_id, (None, None))
        #     lin_str = f"{last_lin:.3f}" if last_lin is not None else "N/A"
        #     ang_str = f"{last_ang:.3f}" if last_ang is not None else "N/A"
        #     vel_log_msgs.append(f"  Agent {agent_id}: Last CmdVel (L={lin_str}, A={ang_str})")
        # if vel_log_msgs:
        #     log_string = "Last commanded velocities before final halt:\n" + "\n".join(vel_log_msgs)
        #     self.get_logger().info(log_string, once=True)

        self.stop_all_robots()
        self.get_logger().debug("Staying in ALL_CONNECTED_OR_HALTED state.", throttle_duration_sec=10.0)


    def _handle_failed_state(self):
        """Generic failure state. Logs error and stops all robots."""
        self.get_logger().error("Connection process entered FAILED state.", once=True)
        self.stop_all_robots()
        # Stay in this state

    def _fail_state_missing_ids(self, state_name):
        """Helper for states requiring seeker/target/male/female IDs"""
        self.get_logger().error(f"Entered {state_name} without required seeker/target/male/female IDs!")
        self.change_state(RobotState.FAILED)


    # ==========================================================================
    # Helper Functions
    # ==========================================================================

    def _find_connected_group(self, start_agent_id: int) -> set:
        """
        Finds all agent IDs connected to the start_agent_id, including itself,
        by traversing the self.connected_pairs graph.
        Returns an empty set if the start_agent_id is not discovered.
        """
        if start_agent_id not in self.discovered_agent_ids:
             self.get_logger().error(f"_find_connected_group: start_agent_id {start_agent_id} not discovered.")
             return set()

        group = set()
        queue = {start_agent_id} # Use a set as a queue/visited list

        while queue:
            current_id = queue.pop()
            if current_id not in group:
                group.add(current_id)
                # Find neighbors in connected_pairs
                for id1, id2 in self.connected_pairs:
                    neighbor = None
                    if id1 == current_id and id2 not in group:
                        neighbor = id2
                    elif id2 == current_id and id1 not in group:
                        neighbor = id1

                    if neighbor is not None:
                        queue.add(neighbor) # Add neighbor to explore

        # self.get_logger().debug(f"Found connected group for {start_agent_id}: {group}")
        return group

    def stop_all_robots(self):
        """Sends zero velocity to all discovered robots."""
        self.get_logger().debug("--- Stopping all discovered robots ---")
        for agent_id in self.discovered_agent_ids:
            # Ensure stop commands get through, even if last was zero
             pub = self.agent_pubs['cmd_vel'].get(agent_id)
             if pub:
                 cmd = Twist()
                 cmd.linear.x = 0.0
                 cmd.angular.z = 0.0
                 pub.publish(cmd)
                 self.agent_last_cmd_vel[agent_id] = (0.0, 0.0) # Update cache

    def _send_velocity(self, agent_id, linear_x, angular_z):
        """Sends velocity command to the specified agent, avoids duplicates except for zero, handles log errors."""
        pub = self.agent_pubs['cmd_vel'].get(agent_id)
        if not pub:
            self.get_logger().warn(f"_send_velocity: No cmd_vel publisher for agent {agent_id}", throttle_duration_sec=10.0)
            return

        # Check cache
        last_lin, last_ang = self.agent_last_cmd_vel.get(agent_id, (None, None))

        # Check if it's a zero velocity command
        is_zero_command = abs(linear_x) < 0.001 and abs(angular_z) < 0.001
        # Check if it's effectively the same as the last command
        is_same_as_last = (last_lin is not None and last_ang is not None and
                           abs(linear_x - last_lin) < 0.001 and
                           abs(angular_z - last_ang) < 0.001)

        # Send if it's NOT the same as the last command, OR if it IS a zero command
        if not is_same_as_last or is_zero_command:
            try:
                # Format numbers safely first
                last_lin_str = f"{last_lin:.3f}" if last_lin is not None else "N/A"
                last_ang_str = f"{last_ang:.3f}" if last_ang is not None else "N/A"
                log_msg_content = ""
                if not is_same_as_last:
                    log_msg_content = f"New L={linear_x:.3f} A={angular_z:.3f} (Last L={last_lin_str} A={last_ang_str})"
                elif is_zero_command and not (abs(last_lin or 0.0) < 0.001 and abs(last_ang or 0.0) < 0.001):
                    log_msg_content = f"Sending Zero L={linear_x:.3f} A={angular_z:.3f} (Last L={last_lin_str} A={last_ang_str})"
                # Log if there's content to log
                # if log_msg_content:
                #     self.get_logger().debug(f"SendVel {agent_id}: {log_msg_content}", throttle_duration_sec=2.0)
            except ValueError as e:
                self.get_logger().error(f"Error formatting debug log in _send_velocity for agent {agent_id}: {e}. Velocity command will still be sent.")

            # Publish command outside try block
            cmd = Twist()
            cmd.linear.x = float(linear_x)
            cmd.angular.z = float(angular_z)
            pub.publish(cmd)
            self.agent_last_cmd_vel[agent_id] = (linear_x, angular_z) # Update cache


    def _centre_robot(self, agent_id):
        """Applies centering correction based on roll for the specified agent."""
        if agent_id not in self.agent_sensor_data['roll'] or agent_id not in self.agent_pubs['cmd_vel']:
             # self.get_logger().warn(f"Cannot center agent {agent_id}: Missing roll data or cmd_vel publisher.", throttle_duration_sec=10.0)
             return

        roll_rad = self.agent_sensor_data['roll'].get(agent_id, 0.0)
        roll_deg = math.degrees(roll_rad)

        # Get the *current* linear speed from the cache to determine direction
        current_linear, last_angular = self.agent_last_cmd_vel.get(agent_id, (0.0, 0.0))

        is_reversing = current_linear < -0.001
        angular_z_cmd = 0.0 # Default to no correction

        # Calculate Correction
        if abs(roll_deg) > self.roll_threshold_deg:
            effective_k_p = -self.kp_centering_forward if is_reversing else self.kp_centering_forward
            angular_z_cmd = effective_k_p * roll_deg
            max_correction_vel = 0.3
            angular_z_cmd = max(-max_correction_vel, min(max_correction_vel, angular_z_cmd))

        # Send Velocity Command only if angular correction changes significantly
        if abs(angular_z_cmd - last_angular) > 0.01:
             # Send the *cached* linear speed along with the *new* angular command
             self._send_velocity(agent_id, current_linear, angular_z_cmd)
        # If the correction is now very small, but we were turning before, send zero angular velocity
        elif abs(angular_z_cmd) < 0.01 and abs(last_angular) > 0.01:
             self._send_velocity(agent_id, current_linear, 0.0)


    def _set_joint(self, agent_id, joint_type, position):
        """Commands 'male' or 'female' joint for the specified agent."""
        dict_key = f"{joint_type}_joint"
        pub = self.agent_pubs.get(dict_key, {}).get(agent_id)
        if not pub:
             self.get_logger().warn(f"_set_joint: No {dict_key} publisher registered for agent {agent_id}", throttle_duration_sec=10.0)
             return

        joint_name = f'base_{joint_type}_joint'
        self.get_logger().debug(f"Setting agent_{agent_id} {joint_name} to {position:.3f}")

        msg = JointTrajectory()
        msg.joint_names = [joint_name]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(sec=2, nanosec=0)
        msg.points.append(point)
        pub.publish(msg)

        # Reset internal state flags when commanding male joint
        if joint_type == 'male':
             self.agent_sensor_data['male_locked'][agent_id] = False
             self.agent_sensor_data['male_unlocked'][agent_id] = False

    # ==========================================================================
    # Subscriber Callbacks
    # ==========================================================================
    def _calculate_imu_rp(self, msg: Imu):
        """Helper to calculate roll/pitch from IMU accelerometer data."""
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        # Check for invalid input that could cause math domain error
        if ay**2 + az**2 < 1e-6 or ax**2 + az**2 < 1e-6: # Avoid sqrt(0) or near-zero
             # self.get_logger().warn("Near-zero denominator in IMU sqrt calculation.", throttle_duration_sec=5.0)
             return None, None
        accel_mag_sq = ax**2 + ay**2 + az**2
        if accel_mag_sq < 0.01: return None, None # Avoid division by zero if overall accel is tiny
        try:
            roll = math.atan2(-ax, math.sqrt(ay**2 + az**2))
            pitch = math.atan2(ay, math.sqrt(ax**2 + az**2))
            return roll, pitch
        except ValueError as e: # Catch potential math domain errors
            self.get_logger().warn(f"Math error in IMU calculation: {e}. Accel=[{ax:.2f},{ay:.2f},{az:.2f}]", once=True)
            return None, None

    def imu_callback(self, msg: Imu, agent_id: int):
        """Stores roll/pitch for the specific agent."""
        # self.get_logger().debug(f"IMU CB Agent {agent_id} Received", throttle_duration_sec=1.0)
        roll, pitch = self._calculate_imu_rp(msg)
        if roll is not None and math.isfinite(roll):
             self.agent_sensor_data['roll'][agent_id] = roll
        if pitch is not None and math.isfinite(pitch):
             self.agent_sensor_data['pitch'][agent_id] = pitch

    def range_callback(self, msg: Range, agent_id: int):
        """Stores valid range data for the specific agent."""
        # self.get_logger().debug(f"Range CB Agent {agent_id} Received: {msg.range:.3f}", throttle_duration_sec=1.0)
        valid_range = None
        if math.isfinite(msg.range) and msg.range >= msg.min_range and msg.range <= msg.max_range:
             valid_range = msg.range
        # Only update if value changes to avoid unnecessary processing
        if self.agent_sensor_data['range'].get(agent_id) != valid_range:
            self.agent_sensor_data['range'][agent_id] = valid_range

    def alignment_callback(self, msg: Bool, agent_id: int):
        """Stores alignment status for the specific agent."""
        # self.get_logger().debug(f"Align CB Agent {agent_id} Received: {msg.data}", throttle_duration_sec=1.0)
        # Only update if value changes
        if self.agent_sensor_data['aligned'].get(agent_id) != msg.data:
             self.agent_sensor_data['aligned'][agent_id] = msg.data

    def male_joint_state_callback(self, msg: JointTrajectoryControllerState, agent_id: int):
        """Stores male lock/unlock status based on reported joint position."""
        # self.get_logger().debug(f"JointState CB Agent {agent_id} Received", throttle_duration_sec=1.0)
        pos_list = None
        # Prioritize actual position
        if msg.actual and msg.actual.positions: pos_list = msg.actual.positions
        elif msg.feedback and msg.feedback.positions: pos_list = msg.feedback.positions # Fallback 1
        elif msg.desired and msg.desired.positions: pos_list = msg.desired.positions # Fallback 2

        if pos_list is None or len(pos_list) == 0:
            # self.get_logger().warn(f"No valid position data in JointState for agent {agent_id}", throttle_duration_sec=5.0)
            return

        current_pos = pos_list[0]
        if not math.isfinite(current_pos):
            self.get_logger().warn(f"Non-finite joint position received for agent {agent_id}: {current_pos}", once=True)
            return

        is_unlocked = abs(current_pos - self.male_unlock_angle) < self.joint_pos_tolerance
        is_locked = abs(current_pos - self.male_lock_angle) < self.joint_pos_tolerance

        # Update internal state only if it changes
        if self.agent_sensor_data['male_unlocked'].get(agent_id) != is_unlocked:
            self.agent_sensor_data['male_unlocked'][agent_id] = is_unlocked
        if self.agent_sensor_data['male_locked'].get(agent_id) != is_locked:
            self.agent_sensor_data['male_locked'][agent_id] = is_locked


# ==========================================================================
# Main Execution
# ==========================================================================
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Connect_Robots()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("Ctrl-C detected, shutting down.")
    except Exception as e:
        if node:
            tb_str = traceback.format_exc()
            node.get_logger().fatal(f"Unhandled exception in node execution: {e}\nTraceback:\n{tb_str}")
        else:
            print(f"Exception during node initialization:")
            traceback.print_exc()
    finally:
        if node and rclpy.ok():
             node.get_logger().info("Stopping robots before shutdown...")
             # Ensure stop commands are sent directly on shutdown
             for agent_id in node.discovered_agent_ids:
                 pub = node.agent_pubs['cmd_vel'].get(agent_id)
                 if pub:
                     cmd = Twist()
                     cmd.linear.x = 0.0
                     cmd.angular.z = 0.0
                     # Try publishing directly multiple times on shutdown
                     for _ in range(3):
                         pub.publish(cmd)
                         time.sleep(0.02)
             node.destroy_node()
        # Ensure shutdown happens even if node creation failed or was already destroyed
        if rclpy.ok():
            rclpy.shutdown()
        print("ROS 2 Shutdown complete.")

if __name__ == '__main__':
    main()