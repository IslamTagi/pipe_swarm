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
from collections import defaultdict # Useful for initializing dicts
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
    LOCKING = auto()                    # <<< Robots will push during this state
    VERIFYING_CONNECTION = auto()       # <<< Robots will pull back during this state
    CONNECTED = auto()                  # Transient state after verification success
    HANDLING_FAILURE = auto()           # Unified failure handling before reset
    RESETTING = auto()
    ALL_CONNECTED_OR_HALTED = auto()
    FAILED = auto()

class AgentStatus(Enum):
    """Individual status of a single robot."""
    UNKNOWN = auto()
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
    """
    def __init__(self):
        super().__init__("connect_robots_fsm_node")

        # --- Discovery ---
        self.robots = {}                    # Basic info {agent_id: {'ns': ns}}
        self.discovered_agent_ids = set()
        self.num_discovered_robots = 0
        self.discovery_complete = False
        self.discovery_timer_period = 2.0
        self.discovery_attempts = 0
        self.max_discovery_attempts = 5

        # --- Dynamic Pub/Sub/Data Storage ---
        self.agent_pubs = {'cmd_vel': {}, 'male_joint': {}, 'female_joint': {}}
        self.agent_subs = {'imu': {}, 'range': {}, 'alignment': {}, 'male_joint_state': {}}
        # Store latest sensor data keyed by agent_id, defaulting nested keys
        self.agent_sensor_data = defaultdict(lambda: defaultdict(lambda: None))
        self._initialize_sensor_data_keys() # Ensure top-level keys exist
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
        self.current_target_id = None       # Agent being targeted for connection
        self.current_male_id = None         # Cache male role for current pair
        self.current_female_id = None       # Cache female role for current pair
        self.locked_connection_range = None # Range stored just before verification pull
        self.current_target_partner_id = None # ID of target's partner if target is already connected

        # --- State Machine ---
        self.current_state = RobotState.DISCOVERING_ROBOTS
        self.state_enter_time = self.get_clock().now()
        self.get_logger().info(f"Node starting in state: {self.current_state.name}")

        # --- Constants and Configuration ---
        # Timing
        self.initial_delay_sec = 2.0
        self.alignment_timeout_sec = 40.0
        self.docking_duration_sec = 10.0
        self.locking_timeout_sec = 10.0 # Increased slightly as pushing might take longer
        self.verification_duration_sec = 5.0
        self.confirmation_timeout_per_robot_sec = 3.0 # Time to wait for range change per robot check
        # Speeds
        self.forward_speed = 0.1
        self.approach_speed_gain = 0.1
        self.approach_min_speed = 0.02
        self.docking_speed_male = 0.0075      # Speed for male pushing during DOCKING/LOCKING
        self.docking_speed_female = -0.01  # Speed for female pushing during DOCKING/LOCKING
        self.verification_pull_speed = -0.05 # Speed for male pulling during VERIFYING
        self.reset_speed_seeker = -0.02     # Adjusted for potentially better separation
        self.reset_speed_target = 0.01      # Adjusted for potentially better separation
        self.confirmation_move_speed = 0.05
        self.aligning_turn_speed = 0.15
        self.partner_hold_speed = 0.0
        # Thresholds & Distances
        self.robot_detection_threshold = 0.2       # % change to trigger halt
        self.confirmation_detection_threshold = 0.2 # % change during confirmation
        self.connection_verification_threshold = 0.2 # % change allowed during verify pull
        self.close_distance_m = 0.15
        self.reset_distance_m = 0.3
        self.range_valid_min_m = 0.05 # Ignore range readings below this
        self.roll_threshold_deg = 1.5
        # Joints
        self.male_lock_angle = math.pi / 2.0
        self.male_unlock_angle = 0.0
        self.female_lift_angle = 0.0
        self.joint_pos_tolerance = 0.1 # Radians
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

    def _initialize_sensor_data_keys(self):
        """Ensure top-level keys for sensor data dictionary exist."""
        keys = ['roll', 'pitch', 'range', 'aligned', 'male_locked', 'male_unlocked']
        for key in keys:
            if key not in self.agent_sensor_data:
                self.agent_sensor_data[key] = {}

    # ==========================================================================
    # Robot Discovery
    # ==========================================================================
    def discover_robots(self):
        """Periodically scans topics to find agent_N robots."""
        self.get_logger().debug("--- discover_robots() function called ---") # DEBUG log
        if self.discovery_complete:
            # self.get_logger().warn("Discovery already complete, timer should have been cancelled.")
            # self.discovery_timer.cancel() # Let completion logic handle cancel
            return

        self.get_logger().info(f"Discovery attempt {self.discovery_attempts + 1}/{self.max_discovery_attempts}...")
        found_new = False
        current_ids_found_this_scan = set()

        try:
            topic_list = self.get_topic_names_and_types()
            agent_topic_pattern = re.compile(r'/agent_(\d+)/robot_description') # Example topic

            for topic_name, _ in topic_list:
                match = agent_topic_pattern.match(topic_name)
                if match:
                    agent_id = int(match.group(1))
                    current_ids_found_this_scan.add(agent_id)
                    if agent_id not in self.discovered_agent_ids:
                        self.get_logger().info(f"Discovered new agent_{agent_id}")
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
        elif found_new:
            self.discovery_attempts = 0 # Reset stability counter
            self.get_logger().info(f"Found new robot(s). Current count: {self.num_discovered_robots}. Resetting stability counter.")
        else: # No robots found yet
            self.discovery_attempts += 1
            if self.discovery_attempts >= self.max_discovery_attempts:
                 self.get_logger().error("Discovery timed out. No robots found.")
                 self.discovery_complete = True # Mark complete even if failed

        # Final transition logic after discovery completion or failure
        if self.discovery_complete:
            self.discovery_timer.cancel()
            if self.current_state == RobotState.DISCOVERING_ROBOTS:
                 if self.num_discovered_robots >= 2:
                      self.change_state(RobotState.IDLE)
                 elif self.num_discovered_robots > 0:
                      self.get_logger().warn("Discovery complete, but less than 2 robots found. Halting.")
                      self.change_state(RobotState.ALL_CONNECTED_OR_HALTED)
                 else:
                      self.change_state(RobotState.FAILED)

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
            self.agent_subs['imu'][agent_id] = self.create_subscription(
                Imu, f"/{ns}/imu/out", partial(self.imu_callback, agent_id=agent_id), 10)
            self.agent_subs['range'][agent_id] = self.create_subscription(
                Range, f"/{ns}/infrared_range", partial(self.range_callback, agent_id=agent_id), 10)
            self.agent_subs['alignment'][agent_id] = self.create_subscription(
                Bool, f"/{ns}/alignment", partial(self.alignment_callback, agent_id=agent_id), 10)
            self.agent_subs['male_joint_state'][agent_id] = self.create_subscription(
                JointTrajectoryControllerState, f"/{ns}/male_joint_trajectory_controller/controller_state", partial(self.male_joint_state_callback, agent_id=agent_id), 10)
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
        if not self.discovery_complete:
            self.get_logger().debug("Waiting for discovery to complete...", throttle_duration_sec=5.0)
            return

        # --- Check for terminal condition ---
        # Define states that mean the process is actively working
        process_in_progress_states = {
            RobotState.GLOBAL_DETECTION_HALT,
            RobotState.SEQUENTIAL_CONFIRMATION, # Confirmation process
            RobotState.INITIATE_CONNECTION,     # Start of connection sequence
            RobotState.APPROACHING_TARGET,      # Connection sequence steps...
            RobotState.CHECKING_ALIGNMENT,
            RobotState.ALIGNING,
            RobotState.DOCKING,
            RobotState.LOCKING,
            RobotState.VERIFYING_CONNECTION,
            RobotState.HANDLING_FAILURE,        # Failure handling / Reset
            RobotState.RESETTING
        }
        connection_or_confirmation_in_progress = self.current_state in process_in_progress_states

        # Check if any discovered robots are still marked as ACTIVE
        active_robots_exist = any(status == AgentStatus.ACTIVE
                                  for aid, status in self.robot_status.items()
                                  if aid in self.discovered_agent_ids)

        # Check if the current state is one where the terminal check is relevant
        current_state_valid_for_check = self.current_state not in [
                                       RobotState.DISCOVERING_ROBOTS, RobotState.IDLE,
                                       RobotState.INITIALIZING_ROBOTS, RobotState.FAILED,
                                       RobotState.ALL_CONNECTED_OR_HALTED, RobotState.CONNECTED
                                   ]

        # Determine if the final state should be entered
        should_enter_final_state = not active_robots_exist and \
                                   not connection_or_confirmation_in_progress and \
                                   current_state_valid_for_check

        # <<< DETAILED LOGGING ADDED HERE >>>
        self.get_logger().debug(f"Terminal Check Tick: State={self.current_state.name}, "
                               f"active_exist={active_robots_exist}, "
                               f"proc_in_prog={connection_or_confirmation_in_progress}, "
                               f"state_valid={current_state_valid_for_check}, "
                               f"FINAL_COND_RESULT={should_enter_final_state}")
        # <<< END DETAILED LOGGING >>>

        if should_enter_final_state:
             # Log the reason more accurately based on the variables checked
             log_reason = f"active_exist={active_robots_exist}, proc_in_prog={connection_or_confirmation_in_progress}"
             self.get_logger().info(f"Condition met for final state ({log_reason}). Entering final state.")
             self.change_state(RobotState.ALL_CONNECTED_OR_HALTED)
             return # Skip rest of tick once in final state


        # --- State Execution ---
        now = self.get_clock().now()
        # Convert nanoseconds to seconds for time_in_state
        time_in_state = (now - self.state_enter_time).nanoseconds / 1e9

        # Map states to their handler functions
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
            # Note: DISCOVERING_ROBOTS is handled by the check at the very top
        }

        # Get the handler function for the current state
        handler = state_handler_map.get(self.current_state)

        if handler:
            try:
                # Pass time_in_state to handlers that need it for timeouts
                takes_time_arg = self.current_state in [
                    RobotState.IDLE, RobotState.SEQUENTIAL_CONFIRMATION,
                    RobotState.ALIGNING, RobotState.DOCKING, RobotState.LOCKING,
                    RobotState.VERIFYING_CONNECTION, RobotState.RESETTING
                ]

                if takes_time_arg:
                     handler(time_in_state)
                else:
                     handler()
            except Exception as e:
                 # Log exception with traceback
                 tb_str = traceback.format_exc() # Get traceback string
                 self.get_logger().error(f"Exception in state handler for {self.current_state.name}: {e}\nTraceback:\n{tb_str}")
                 self.change_state(RobotState.FAILED)
        else:
            # This case should ideally not be reached if all states are mapped
            self.get_logger().error(f"No handler implemented for state: {self.current_state.name}")
            self.change_state(RobotState.FAILED)

    # ==========================================================================
    # State Change Logic
    # ==========================================================================
    def change_state(self, new_state: RobotState):
        """Transitions to a new state, logging the change and resetting temporary variables."""
        if new_state == self.current_state:
            return

        previous_state = self.current_state # Store state being exited

        self.get_logger().debug(f"--- change_state START ---")
        self.get_logger().debug(f"Current state BEFORE change: {previous_state.name}")
        self.get_logger().debug(f"Target new state: {new_state.name}")
        self.get_logger().debug(f"Vars BEFORE: detector={self.detecting_agent_id}, targets={self.confirmation_target_ids}, seeker={self.current_seeker_id}, target={self.current_target_id}, partner={self.current_target_partner_id}")

        self.get_logger().info(f"Changing state from {previous_state.name} to {new_state.name}")

        self.current_state = new_state      # Update to new state
        self.state_enter_time = self.get_clock().now()

        self.get_logger().debug(f"Current state AFTER change: {self.current_state.name}")
        self.get_logger().debug(f"Previous state variable: {previous_state.name}")

        # --- Reset temporary variables based on state transitions ---

        # Reset detection/confirmation variables ONLY IF exiting the confirmation process
        confirmation_process_states = [RobotState.GLOBAL_DETECTION_HALT, RobotState.SEQUENTIAL_CONFIRMATION]
        prev_in_conf = previous_state in confirmation_process_states
        new_in_conf = new_state in confirmation_process_states
        self.get_logger().debug(f"CONF check: prev_in_conf={prev_in_conf}, new_in_conf={new_in_conf}. Condition = {prev_in_conf and not new_in_conf}")

        if prev_in_conf and not new_in_conf:
             self.get_logger().debug(f"CONF RESET TRIGGERED: Resetting confirmation vars (leaving confirmation process: {previous_state.name} -> {new_state.name})")
             self.detecting_agent_id = None
             self.detection_range = None
             self.confirmation_target_ids = []
             self.confirmation_check_index = 0
             self.confirmation_start_time = None
        else:
            self.get_logger().debug(f"CONF RESET SKIPPED.")


        # Reset connection sequence variables ONLY IF exiting the connection sequence
        connection_sequence_states = [
            RobotState.INITIATE_CONNECTION, RobotState.APPROACHING_TARGET, RobotState.CHECKING_ALIGNMENT,
            RobotState.ALIGNING, RobotState.DOCKING, RobotState.LOCKING, RobotState.VERIFYING_CONNECTION,
            RobotState.HANDLING_FAILURE, RobotState.RESETTING, RobotState.CONNECTED # Added CONNECTED
        ]
        prev_in_conn = previous_state in connection_sequence_states
        new_in_conn = new_state in connection_sequence_states
        self.get_logger().debug(f"CONN check: prev_in_conn={prev_in_conn}, new_in_conn={new_in_conn}. Condition = {prev_in_conn and not new_in_conn}")

        if prev_in_conn and not new_in_conn:
             self.get_logger().debug(f"CONN RESET TRIGGERED: Resetting connection sequence vars (leaving sequence: {previous_state.name} -> {new_state.name})")
             self.current_seeker_id = None
             self.current_target_id = None
             self.current_male_id = None
             self.current_female_id = None
             self.locked_connection_range = None
             # <<< ADDED RESET >>>
             self.current_target_partner_id = None # Reset partner ID too
             # <<< --- >>>
        else:
            self.get_logger().debug(f"CONN RESET SKIPPED.")

        # --- FINAL VAR CHECK ---
        self.get_logger().debug(f"Vars AFTER change_state: detector={self.detecting_agent_id}, targets={self.confirmation_target_ids}, seeker={self.current_seeker_id}, target={self.current_target_id}, partner={self.current_target_partner_id}")
        self.get_logger().debug(f"--- change_state END ---")
        # ---

    # ==========================================================================
    # State Handling Functions
    # ==========================================================================

    def _handle_idle_state(self, time_in_state):
        """Waits for initial delay and ensures resources are ready."""
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
        self.get_logger().info("Initializing all robots: Setting joints, status, and getting initial scans.")
        all_scans_obtained = True
        for agent_id in self.discovered_agent_ids:
            # Only initialize if status is Unknown or Failed (allow re-init)
            if self.robot_status.get(agent_id, AgentStatus.UNKNOWN) in [AgentStatus.UNKNOWN, AgentStatus.FAILED]:
                self._set_joint(agent_id, 'male', self.male_unlock_angle)
                self._set_joint(agent_id, 'female', self.female_lift_angle)
                self.robot_status[agent_id] = AgentStatus.ACTIVE
                self.initial_scan_ranges[agent_id] = None # Clear previous scan
                self.get_logger().info(f"  Agent {agent_id} set to ACTIVE, joints commanded.")

            # Attempt to get initial scan if not already obtained
            if self.initial_scan_ranges.get(agent_id) is None:
                current_range = self.agent_sensor_data['range'].get(agent_id, None)
                if current_range is not None and current_range >= self.range_valid_min_m:
                    self.initial_scan_ranges[agent_id] = current_range
                    self.get_logger().info(f"  Stored initial scan range for agent {agent_id}: {current_range:.3f} m")
                else:
                    self.get_logger().info(f"  Waiting for valid initial range for agent {agent_id}...")
                    all_scans_obtained = False # Wait until all have initial range

        if all_scans_obtained:
            self.get_logger().info("All robots initialized and have initial range scans.")
            time.sleep(0.5) # Short delay for joints to potentially start moving
            self.change_state(RobotState.MOVING_ALL_FORWARD)
        # Else: Stay in this state, will retry getting scans on next tick

    def _handle_moving_all_forward_state(self):
        """Moves all ACTIVE robots forward and checks for range changes."""
        detection_triggered = False
        active_agent_found = False
        for agent_id in self.discovered_agent_ids:
            if self.robot_status.get(agent_id) == AgentStatus.ACTIVE:
                active_agent_found = True
                self._send_velocity(agent_id, self.forward_speed, 0.0)
                self._centre_robot(agent_id)

                current_range = self.agent_sensor_data['range'].get(agent_id, None)
                initial_range = self.initial_scan_ranges.get(agent_id, None)

                if initial_range is None: # Should have been set in INIT, but check again
                    if current_range is not None and current_range >= self.range_valid_min_m:
                        self.initial_scan_ranges[agent_id] = current_range
                        self.get_logger().warn(f"Re-acquired initial scan for agent {agent_id} in MOVING state: {current_range:.3f} m")
                    continue # Skip detection check this tick

                # Check for significant range change
                if current_range is not None and current_range >= self.range_valid_min_m and initial_range >= self.range_valid_min_m:
                    # Avoid division by zero just in case initial range is tiny
                    if initial_range > 0.01:
                        difference = abs(current_range - initial_range) / initial_range
                        if difference > self.robot_detection_threshold:
                            self.get_logger().info(f"Agent {agent_id} detected range change! (Range: {current_range:.3f}, Initial: {initial_range:.3f}, Diff: {difference:.2%}). Halting all.")
                            self.detecting_agent_id = agent_id
                            self.detection_range = current_range # Store the range at detection
                            self.robot_status[agent_id] = AgentStatus.DETECTED_SOMETHING
                            detection_triggered = True
                            break # Process only the first detection per tick
                    else:
                        self.get_logger().warn(f"Agent {agent_id} initial range {initial_range} too small to calculate difference.", throttle_duration_sec=5.0)


        if detection_triggered:
            self.stop_all_robots()
            self.change_state(RobotState.GLOBAL_DETECTION_HALT)
        elif not active_agent_found:
             # This case handled by the check at the start of state_machine_tick
             self.get_logger().debug("No active robots found in MOVING_ALL_FORWARD (should transition soon).")


    def _handle_global_detection_halt_state(self):
        """Stops all robots and prepares list for sequential confirmation, including connected robots."""
        if self.detecting_agent_id is None:
            self.get_logger().error("Entered GLOBAL_DETECTION_HALT without detecting_agent_id!")
            self.change_state(RobotState.FAILED)
            return

        self.get_logger().info(f"Global halt triggered by Agent {self.detecting_agent_id}. Preparing confirmation checks.")
        self.stop_all_robots() # Ensure everyone is stopped

        # Build list of potential targets:
        # Include ACTIVE, OBSTACLE_NEAR, and CONNECTED robots
        valid_target_statuses = {AgentStatus.ACTIVE, AgentStatus.OBSTACLE_NEAR, AgentStatus.CONNECTED} # Added CONNECTED
        # Define statuses that make a robot unavailable to be checked
        excluded_statuses = {
            AgentStatus.CONNECTION_SEEKER,      # Currently connecting (as seeker)
            AgentStatus.CONNECTION_TARGET,      # Currently connecting (as target)
            AgentStatus.RESETTING,              # Currently resetting
            AgentStatus.CONFIRMATION_TARGET,    # Currently being moved for check by someone else
            AgentStatus.FAILED                  # Ignore failed robots
         }

        self.confirmation_target_ids = sorted([
            aid for aid in self.discovered_agent_ids
            if aid != self.detecting_agent_id and \
               self.robot_status.get(aid) in valid_target_statuses and \
               self.robot_status.get(aid) not in excluded_statuses
        ])

        self.get_logger().debug(f"In GLOBAL_HALT: Built confirmation_target_ids: {self.confirmation_target_ids}")

        if not self.confirmation_target_ids:
             self.get_logger().warn(f"Agent {self.detecting_agent_id} detected something, but no other suitable robots found to check. Assuming obstacle.")
             self._handle_obstacle_outcome()
             return

        # If list is NOT empty, proceed
        self.confirmation_check_index = 0
        self.confirmation_start_time = self.get_clock().now() # Initialize timer for the *first* check

        self.get_logger().debug(f"Exiting GLOBAL_HALT: detector={self.detecting_agent_id}, "
                               f"targets={self.confirmation_target_ids}, "
                               f"start_time_is_none={self.confirmation_start_time is None}")

        self.get_logger().info(f"Potential targets for confirmation: {self.confirmation_target_ids}. Starting check with {self.confirmation_target_ids[0]}.")
        self.change_state(RobotState.SEQUENTIAL_CONFIRMATION)

      
      
    def _handle_sequential_confirmation_state(self, time_in_state):
        """Sequentially moves potential targets and checks detector's range. Handles confirmation of single robots and connected pairs."""
        self.get_logger().debug(f"Entering SEQ_CONFIRM: detector={self.detecting_agent_id}, "
                               f"targets={self.confirmation_target_ids}, "
                               f"start_time_is_none={self.confirmation_start_time is None}")

        if self.detecting_agent_id is None or not self.confirmation_target_ids or self.confirmation_start_time is None:
            self.get_logger().error("Entered SEQUENTIAL_CONFIRMATION with invalid state variables!")
            self.change_state(RobotState.FAILED)
            return

        # Get the agent currently being checked
        target_check_id = self.confirmation_target_ids[self.confirmation_check_index]
        self.get_logger().debug(f"Checking target Agent {target_check_id} (Index {self.confirmation_check_index}). Moving it.", throttle_duration_sec=1.0)

        # --- Store original status BEFORE setting to CONFIRMATION_TARGET ---
        original_status_before_check = self.robot_status.get(target_check_id, AgentStatus.UNKNOWN)
        if original_status_before_check == AgentStatus.UNKNOWN:
            self.get_logger().warn(f"Agent {target_check_id} had UNKNOWN status before confirmation check.")
        # ---

        # Temporarily mark target being checked
        self.robot_status[target_check_id] = AgentStatus.CONFIRMATION_TARGET

        # Ensure detector's range is valid before starting check
        detector_current_range = self.agent_sensor_data['range'].get(self.detecting_agent_id)
        if self.detection_range is None or self.detection_range < 0.01 :
             self.get_logger().warn(f"Initial detection range for detector {self.detecting_agent_id} was invalid ({self.detection_range}). Cannot confirm. Assuming obstacle.")
             self._send_velocity(target_check_id, 0.0, 0.0)
             # Revert status even on early exit - Use the new logic here too
             can_reset_to_active = original_status_before_check in {AgentStatus.ACTIVE, AgentStatus.OBSTACLE_NEAR, AgentStatus.UNKNOWN} # Include OBSTACLE_NEAR? Maybe not. Let's stick to ACTIVE/UNKNOWN for now.
             if original_status_before_check in {AgentStatus.ACTIVE, AgentStatus.UNKNOWN}:
                 self.get_logger().debug(f"Early exit: Resetting {target_check_id} to ACTIVE (Original: {original_status_before_check.name})")
                 self.robot_status[target_check_id] = AgentStatus.ACTIVE
             elif original_status_before_check != AgentStatus.CONFIRMATION_TARGET: # Avoid reverting if it was already this temp state
                 self.get_logger().debug(f"Early exit: Reverting {target_check_id} to original non-active state: {original_status_before_check.name}")
                 self.robot_status[target_check_id] = original_status_before_check

             self._handle_obstacle_outcome() # Call obstacle handler
             return
        if detector_current_range is None or detector_current_range < self.range_valid_min_m:
            self.get_logger().warn(f"Detector Agent {self.detecting_agent_id}'s range ({detector_current_range}) invalid during confirmation check. Assuming obstacle.")
            self._send_velocity(target_check_id, 0.0, 0.0)
            # Revert status even on early exit - Use the new logic here too
            if original_status_before_check in {AgentStatus.ACTIVE, AgentStatus.UNKNOWN}:
                 self.get_logger().debug(f"Invalid range: Resetting {target_check_id} to ACTIVE (Original: {original_status_before_check.name})")
                 self.robot_status[target_check_id] = AgentStatus.ACTIVE
            elif original_status_before_check != AgentStatus.CONFIRMATION_TARGET:
                 self.get_logger().debug(f"Invalid range: Reverting {target_check_id} to original non-active state: {original_status_before_check.name}")
                 self.robot_status[target_check_id] = original_status_before_check

            self._handle_obstacle_outcome() # Call obstacle handler
            return

        # Move the current target robot slightly forward
        self._send_velocity(target_check_id, self.confirmation_move_speed, 0.0)
        self._centre_robot(target_check_id)

        # Check if detector's range changes significantly
        difference = abs(detector_current_range - self.detection_range) / self.detection_range
        if difference > self.confirmation_detection_threshold:
            # --- Confirmation Success ---
            self.get_logger().info(f"Range changed for detector {self.detecting_agent_id} (Now: {detector_current_range:.3f}, Initial: {self.detection_range:.3f}, Diff: {difference:.2%}). Confirmed Agent {target_check_id}.")
            self._send_velocity(target_check_id, 0.0, 0.0) # Stop the confirmed target

            # Capture the IDs *before* calling change_state
            confirmed_seeker_id = self.detecting_agent_id
            confirmed_target_id = target_check_id # Default to the directly checked target
            confirmed_partner_id = None

            # Check if the confirmed target is part of an existing pair
            is_connected = False
            partner_id = None
            for pair in self.connected_pairs:
                if target_check_id in pair:
                    is_connected = True
                    partner_id = pair[0] if pair[1] == target_check_id else pair[1]
                    break

            if is_connected and partner_id is not None:
                # Target is part of a pair
                self.get_logger().info(f"Confirmed Agent {target_check_id} is CONNECTED with Agent {partner_id}.")
                pair_ids = sorted([target_check_id, partner_id])
                rear_target_id = pair_ids[0] # Assume lower ID is rear/male
                front_partner_id = pair_ids[1] # Assume higher ID is front/female
                self.get_logger().info(f"Designating rear robot {rear_target_id} as target, partner is {front_partner_id}.")
                confirmed_target_id = rear_target_id
                confirmed_partner_id = front_partner_id
            else:
                # Target is a single robot
                 self.get_logger().info(f"Confirmed Agent {target_check_id} is single. Initiating connection.")

            # Final Check and Assignment
            if confirmed_seeker_id is None or confirmed_target_id is None:
                self.get_logger().error(f"CRITICAL: Cannot initiate connection. Seeker ({confirmed_seeker_id}) or Target ({confirmed_target_id}) is None AFTER confirmation logic.")
                # Revert the checked target's status appropriately before obstacle outcome
                if original_status_before_check in {AgentStatus.ACTIVE, AgentStatus.UNKNOWN}:
                     self.robot_status[target_check_id] = AgentStatus.ACTIVE
                elif original_status_before_check != AgentStatus.CONFIRMATION_TARGET:
                     self.robot_status[target_check_id] = original_status_before_check
                self._handle_obstacle_outcome() # Treat as obstacle if IDs invalid
                return

            self.get_logger().debug(f"SEQ_CONFIRM: Preparing to set Seeker={confirmed_seeker_id}, Target={confirmed_target_id}, Partner={confirmed_partner_id}")
            self.current_seeker_id = confirmed_seeker_id
            self.current_target_id = confirmed_target_id
            self.current_target_partner_id = confirmed_partner_id

            self.change_state(RobotState.INITIATE_CONNECTION)
            return # Exit state handler after handling confirmation

        # --- Timeout check for *this specific target* ---
        time_since_check_started = (self.get_clock().now() - self.confirmation_start_time).nanoseconds / 1e9
        if time_since_check_started > self.confirmation_timeout_per_robot_sec:
             self.get_logger().warn(f"Confirmation check for Agent {target_check_id} timed out. Detector range {detector_current_range:.3f} (initial {self.detection_range:.3f}). Assuming not the target.")
             self._send_velocity(target_check_id, 0.0, 0.0) # Stop the target being checked

             # --- MODIFIED STATUS RESET LOGIC ---
             # Reset to ACTIVE if the original status allows it (was ACTIVE or UNKNOWN)
             # Otherwise, revert to the original non-CONFIRMATION_TARGET status.
             if original_status_before_check in {AgentStatus.ACTIVE, AgentStatus.UNKNOWN}:
                 self.get_logger().info(f"Timeout check: Resetting Agent {target_check_id} status to ACTIVE (Original was {original_status_before_check.name}).")
                 self.robot_status[target_check_id] = AgentStatus.ACTIVE
             # Check if original status was something else that should be restored (e.g. CONNECTED, OBSTACLE_NEAR)
             # Avoid reverting if original was already CONFIRMATION_TARGET (shouldn't happen but safeguard)
             elif original_status_before_check != AgentStatus.CONFIRMATION_TARGET:
                 self.get_logger().info(f"Timeout check: Reverting Agent {target_check_id} status to its original state: {original_status_before_check.name}.")
                 self.robot_status[target_check_id] = original_status_before_check
             else:
                 # This case implies original_status was CONFIRMATION_TARGET, which is unexpected.
                 # Fallback to ACTIVE might be safest.
                 self.get_logger().warn(f"Timeout check: Agent {target_check_id} original status was unexpectedly {original_status_before_check.name}. Setting to ACTIVE as fallback.")
                 self.robot_status[target_check_id] = AgentStatus.ACTIVE
             # --- END MODIFIED STATUS RESET LOGIC ---


             # Move to the next target
             self.confirmation_check_index += 1
             if self.confirmation_check_index >= len(self.confirmation_target_ids):
                 # All targets checked, none confirmed
                 self.get_logger().warn(f"All potential targets checked for detector {self.detecting_agent_id}. Assuming obstacle.")
                 self._handle_obstacle_outcome() # Call obstacle handler here
             else:
                 # Reset timer for the next check
                 next_target_id = self.confirmation_target_ids[self.confirmation_check_index]
                 self.confirmation_start_time = self.get_clock().now()
                 self.get_logger().info(f"Moving to check next target: {next_target_id} (Index {self.confirmation_check_index}).")
                 # Stay in SEQUENTIAL_CONFIRMATION


    
    def _handle_obstacle_outcome(self):
        """Helper to handle when sequential confirmation determines an obstacle."""
        # Store target IDs before they might be cleared by change_state
        checked_target_ids = list(self.confirmation_target_ids) # Make a copy

        detecting_agent_id_local = self.detecting_agent_id # Store locally before change_state clears it

        if detecting_agent_id_local is not None:
             # ... (marking detector as OBSTACLE_NEAR logic) ...
             if self.robot_status.get(detecting_agent_id_local) != AgentStatus.OBSTACLE_NEAR:
                self.robot_status[detecting_agent_id_local] = AgentStatus.OBSTACLE_NEAR
                self.get_logger().info(f"Agent {detecting_agent_id_local} marked as OBSTACLE_NEAR.")
                self.initial_scan_ranges[detecting_agent_id_local] = None # Reset range only if status changed
        else:
             self.get_logger().error("Obstacle outcome handler called without detecting_agent_id!")

        # --- Ensure robots that were checked but not confirmed are reset to ACTIVE ---
        self.get_logger().info(f"Resetting status for checked targets after obstacle determination: {checked_target_ids}")
        for checked_id in checked_target_ids:
            # Don't reset the detecting agent itself
            if checked_id == detecting_agent_id_local:
                continue

            current_status = self.robot_status.get(checked_id)
            # Only reset if they are not already CONNECTED, OBSTACLE_NEAR, FAILED, etc.
            # Status might be CONFIRMATION_TARGET (reverted by timeout/failure) or potentially ACTIVE already.
            # Resetting ACTIVE to ACTIVE is harmless.
            resettable_statuses = {AgentStatus.CONFIRMATION_TARGET, AgentStatus.ACTIVE, AgentStatus.UNKNOWN} # Statuses that imply it's not busy/connected/failed
            if current_status in resettable_statuses:
                self.get_logger().info(f"  Resetting Agent {checked_id} status from {current_status.name if current_status else 'None'} to ACTIVE.")
                self.robot_status[checked_id] = AgentStatus.ACTIVE
            else:
                self.get_logger().info(f"  Skipping status reset for Agent {checked_id} (Status: {current_status.name if current_status else 'None'})")
        # --- END ---

        # Reset detection variables (done by change_state below) and resume movement
        self.change_state(RobotState.MOVING_ALL_FORWARD)

    # <<< MODIFIED FUNCTION >>>
    def _handle_initiate_connection_state(self):
        """Determines male/female roles for NEW connection, commands joints, updates status. Handles single target vs pair target."""
        if self.current_seeker_id is None or self.current_target_id is None:
            self.get_logger().error("Entered INITIATE_CONNECTION without seeker/target IDs!")
            self.change_state(RobotState.FAILED)
            return

        seeker_id = self.current_seeker_id
        target_id = self.current_target_id # This is the *specific* robot being docked with (rear of pair if applicable)
        partner_id = self.current_target_partner_id # Will be None if target is single

        # --- Assign roles for the NEW connection (Seeker vs Target) ---
        # Assume seeker is always male, target is female for this new connection attempt
        # This simplifies docking/locking logic later.
        self.current_male_id = seeker_id
        self.current_female_id = target_id
        # ---

        if partner_id is not None:
            self.get_logger().info(f"Initiating connection to PAIR: Seeker={seeker_id}, Target={target_id}, Partner={partner_id} | New Male={self.current_male_id}, New Female={self.current_female_id}")
            # Ensure partner remains stationary and CONNECTED
            self._send_velocity(partner_id, self.partner_hold_speed, 0.0)
            if self.robot_status.get(partner_id) != AgentStatus.CONNECTED:
                 self.get_logger().warn(f"Target Partner {partner_id} status was not CONNECTED ({self.robot_status.get(partner_id)}). Setting to CONNECTED.")
                 self.robot_status[partner_id] = AgentStatus.CONNECTED
        else:
            self.get_logger().info(f"Initiating connection to SINGLE: Seeker={seeker_id}, Target={target_id} | New Male={self.current_male_id}, New Female={self.current_female_id}")

        # Command joints for the connecting pair (Seeker=Male, Target=Female)
        self._set_joint(self.current_male_id, 'male', self.male_unlock_angle)
        self._set_joint(self.current_female_id, 'female', self.female_lift_angle) # Assume target needs female joint positioned

        # Update status for the active participants
        self.robot_status[seeker_id] = AgentStatus.CONNECTION_SEEKER
        self.robot_status[target_id] = AgentStatus.CONNECTION_TARGET

        time.sleep(0.5) # Allow time for joints to start moving
        self.change_state(RobotState.APPROACHING_TARGET)

    def _handle_approaching_target_state(self):
        """Moves seeker towards target. Keeps partner still if target is part of a pair."""
        if self.current_seeker_id is None or self.current_target_id is None: return self._fail_state_missing_ids("APPROACHING_TARGET")

        seeker_id = self.current_seeker_id
        target_id = self.current_target_id
        partner_id = self.current_target_partner_id # Check if target has a partner

        # --- Ensure Target and Partner (if exists) stay stopped ---
        self._send_velocity(target_id, self.partner_hold_speed, 0.0)
        if partner_id is not None:
            self.get_logger().debug(f"Approach: Keeping partner {partner_id} stopped.", throttle_duration_sec=2.0)
            self._send_velocity(partner_id, self.partner_hold_speed, 0.0)
        # ---

        self._centre_robot(seeker_id)

        current_range_seeker = self.agent_sensor_data['range'].get(seeker_id)
        if current_range_seeker is None or current_range_seeker < self.range_valid_min_m:
             self.get_logger().warn(f"Seeker {seeker_id}'s range lost/invalid ({current_range_seeker}) during approach.", throttle_duration_sec=5.0)
             self._send_velocity(seeker_id, 0.0, 0.0) # Stop seeker
             return

        if current_range_seeker < self.close_distance_m:
            self.get_logger().info(f"Seeker {seeker_id} reached close distance ({current_range_seeker:.3f} m) to target {target_id}.")
            self._send_velocity(seeker_id, 0.0, 0.0)
            self.change_state(RobotState.CHECKING_ALIGNMENT)
        else:
            speed = max(self.approach_min_speed, self.approach_speed_gain * (current_range_seeker - self.close_distance_m))
            self._send_velocity(seeker_id, speed, 0.0)

    def _handle_checking_alignment_state(self):
        """Checks the seeker's alignment sensor status. Keeps partner still."""
        if self.current_seeker_id is None or self.current_target_id is None: return self._fail_state_missing_ids("CHECKING_ALIGNMENT")

        seeker_id = self.current_seeker_id
        target_id = self.current_target_id
        partner_id = self.current_target_partner_id

        # --- Ensure Target and Partner (if exists) stay stopped ---
        self._send_velocity(target_id, self.partner_hold_speed, 0.0)
        if partner_id is not None:
            self._send_velocity(partner_id, self.partner_hold_speed, 0.0)
        # ---

        self.get_logger().info(f"Checking alignment status (Seeker {seeker_id} sensor)...")
        seeker_aligned = self.agent_sensor_data['aligned'].get(seeker_id, False)

        self.get_logger().debug(f"CHECKING_ALIGNMENT: Seeker {seeker_id} alignment sensor value = {seeker_aligned}")

        if seeker_aligned:
            self.get_logger().info(f"Seeker {seeker_id} aligned with target {target_id}.")
            self.change_state(RobotState.DOCKING)
        else:
            self.get_logger().info(f"Seeker {seeker_id} not aligned. Aligning target {target_id}.")
            self.change_state(RobotState.ALIGNING)

    def _handle_aligning_state(self, time_in_state):
        """Rotates the target robot based on the seeker's alignment sensor. Keeps partner still."""
        if self.current_seeker_id is None or self.current_target_id is None: return self._fail_state_missing_ids("ALIGNING")

        seeker_id = self.current_seeker_id
        target_id = self.current_target_id # Only this robot rotates
        partner_id = self.current_target_partner_id # This robot stays still

        # --- Ensure Seeker and Partner (if exists) stay stopped ---
        self._send_velocity(seeker_id, 0.0, 0.0)
        if partner_id is not None:
            self.get_logger().debug(f"Align: Keeping partner {partner_id} stopped.", throttle_duration_sec=2.0)
            self._send_velocity(partner_id, 0.0, 0.0)
        # ---

        self._centre_robot(target_id) # Center target while rotating

        seeker_aligned = self.agent_sensor_data['aligned'].get(seeker_id, False)

        self.get_logger().debug(f"ALIGNING: Seeker {seeker_id} alignment sensor value = {seeker_aligned} (Time in state: {time_in_state:.2f}s)", throttle_duration_sec=0.5)

        if seeker_aligned:
            self.get_logger().info("Alignment successful during maneuver.")
            self._send_velocity(target_id, 0.0, 0.0) # Stop target rotation
            self.change_state(RobotState.DOCKING)
            return

        if time_in_state > self.alignment_timeout_sec:
            self.get_logger().error(f"Alignment timed out for pair ({seeker_id}, {target_id})!")
            self._send_velocity(target_id, 0.0, 0.0)
            self.change_state(RobotState.HANDLING_FAILURE)
            return

        # Rotate target robot
        time_in_cycle = time_in_state % 4.0
        angular_z = 0.0
        if time_in_cycle <= 1.0: angular_z = self.aligning_turn_speed
        elif time_in_cycle <= 3.0: angular_z = -self.aligning_turn_speed
        else: angular_z = self.aligning_turn_speed
        self._send_velocity(target_id, 0.0, angular_z)

    def _handle_docking_state(self, time_in_state):
        """Moves the 'male' (seeker) forward and 'female' (target) backward. Keeps partner still."""
        # Uses self.current_male/female_id which were set in INITIATE_CONNECTION
        if self.current_male_id is None or self.current_female_id is None: return self._fail_state_missing_ids("DOCKING")

        male_id = self.current_male_id     # This is the seeker
        female_id = self.current_female_id # This is the target (rear of pair if applicable)
        partner_id = self.current_target_partner_id # Check if target has a partner

         # --- Get current roll angles ---
        # Access roll data, default to None if not found yet
        male_roll_rad = self.agent_sensor_data['roll'].get(male_id, None)
        female_roll_rad = self.agent_sensor_data['roll'].get(female_id, None)
        partner_roll_rad = None
        if partner_id is not None:
            partner_roll_rad = self.agent_sensor_data['roll'].get(partner_id, None)

        # Convert to degrees for logging, handle None cases gracefully
        male_roll_deg = f"{math.degrees(male_roll_rad):.1f}" if male_roll_rad is not None else "N/A"
        female_roll_deg = f"{math.degrees(female_roll_rad):.1f}" if female_roll_rad is not None else "N/A"
        partner_roll_deg_str = "N/A" # Default partner string
        if partner_id is not None:
            partner_roll_deg_str = f"{math.degrees(partner_roll_rad):.1f}" if partner_roll_rad is not None else "N/A"

        # --- Log docking status and roll angles (throttled) ---
        # Combine the status and roll info into one log message
        roll_log_msg = f"Rolls (deg): Male({male_id})={male_roll_deg}, Female({female_id})={female_roll_deg}"
        if partner_id is not None:
            roll_log_msg += f", Partner({partner_id})={partner_roll_deg_str}"

        # self.get_logger().info(
        #     f"Docking: StateTime={time_in_state:.1f}/{self.docking_duration_sec:.1f} | {roll_log_msg}",
        #     throttle_duration_sec=0.5 # Log roll info twice per second during docking
        # )


        self.get_logger().info(f"Docking Male={male_id}, Female={female_id}... ({time_in_state:.1f}/{self.docking_duration_sec:.1f})", throttle_duration_sec=1.0)
        self._send_velocity(male_id, self.docking_speed_male, 0.0)
        self._send_velocity(female_id, self.docking_speed_female, 0.0)
        # self._centre_robot(male_id)
        # self._centre_robot(female_id)

        # --- Ensure Partner (if exists) stays stopped ---
        if partner_id is not None:
            self.get_logger().debug(f"Docking CMD (Partner Present): Male={male_id} V={self.docking_speed_male:.3f}, Female={female_id} V={self.docking_speed_female:.3f}, Partner={partner_id} V=0.0", throttle_duration_sec=1.0)
            self._send_velocity(partner_id, self.docking_speed_female, 0.0)

            # self._centre_robot(partner_id)
        # ---

        if time_in_state >= self.docking_duration_sec:
            self.get_logger().info("Docking duration complete. Attempting lock.")
            # Keep pushing into LOCKING state
            self.change_state(RobotState.LOCKING)

    # <<< MODIFIED FUNCTION >>>
    def _handle_locking_state(self, time_in_state):
        """Commands the 'male' (seeker) joint to lock WHILE pushing 'male'/'female' (seeker/target). Keeps partner still."""
        if self.current_male_id is None or self.current_female_id is None: return self._fail_state_missing_ids("LOCKING")

        male_id = self.current_male_id     # This is the seeker
        female_id = self.current_female_id # This is the target
        partner_id = self.current_target_partner_id # Check if target has a partner

        # --- Apply gentle push using docking speeds ---
        self.get_logger().debug(f"Pushing during LOCKING: Male={male_id}, Female={female_id}", throttle_duration_sec=1.0)
        self._send_velocity(male_id, self.docking_speed_male, 0.0)
        self._send_velocity(female_id, self.docking_speed_female, 0.0)
        self._centre_robot(male_id)
        self._centre_robot(female_id)
        # ---

        # --- Ensure Partner (if exists) stays stopped ---
        if partner_id is not None:
            self.get_logger().debug(f"Locking: Keeping partner {partner_id} stopped.", throttle_duration_sec=2.0)
            self._send_velocity(partner_id, self.partner_hold_speed, 0.0)
        # ---

        # Command lock once near the beginning of the state
        if time_in_state < 0.2:
            if not self.agent_sensor_data['male_locked'].get(male_id, False):
                 self.get_logger().info(f"Commanding male ({male_id}) joint to lock.")
                 self._set_joint(male_id, 'male', self.male_lock_angle)

        # Check lock status (updated by callback)
        male_locked = self.agent_sensor_data['male_locked'].get(male_id, False)
        if male_locked:
            self.get_logger().info(f"Male ({male_id}) lock confirmed by feedback.")
            # Stop pushing before verifying
            self._send_velocity(male_id, 0.0, 0.0)
            self._send_velocity(female_id, 0.0, 0.0)
            # Partner is already stopped
            self.change_state(RobotState.VERIFYING_CONNECTION)
            return

        # Timeout check
        if time_in_state > self.locking_timeout_sec:
            self.get_logger().error(f"Locking timed out for male agent {male_id}!")
            # Stop pushing on failure
            self._send_velocity(male_id, 0.0, 0.0)
            self._send_velocity(female_id, 0.0, 0.0)
            # Partner is already stopped
            self.change_state(RobotState.HANDLING_FAILURE)

    # <<< MODIFIED FUNCTION >>>
    def _handle_verifying_connection_state(self, time_in_state):
        """Pulls the 'male' (seeker) back slightly. Keeps 'female' (target) and partner still."""
        if self.current_male_id is None or self.current_female_id is None: return self._fail_state_missing_ids("VERIFYING_CONNECTION")

        male_id = self.current_male_id     # This is the seeker
        female_id = self.current_female_id # This is the target
        partner_id = self.current_target_partner_id # Check if target has a partner

        # --- Ensure Female (target) and Partner (if exists) stay stopped ---
        self._send_velocity(female_id, 0.0, 0.0)
        if partner_id is not None:
            self.get_logger().debug(f"Verify: Keeping partner {partner_id} stopped.", throttle_duration_sec=2.0)
            self._send_velocity(partner_id, 0.0, 0.0)
        # ---

        current_range_male = self.agent_sensor_data['range'].get(male_id)

        # Store initial range at the start of verification
        if self.locked_connection_range is None:
            # ... (logic to store locked_connection_range or fail remains same) ...
            if current_range_male is not None and current_range_male >= self.range_valid_min_m:
                self.locked_connection_range = current_range_male
                self.get_logger().info(f"Verifying connection (Pair: {male_id},{female_id}). Locked range: {self.locked_connection_range:.3f}m.")
            else:
                if time_in_state > 1.0: # Wait 1s for valid range
                    self.get_logger().error(f"Verification failed - No valid range from male {male_id} at start.")
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

            # ... (verification logic based on range difference remains same) ...
            if self.locked_connection_range is None or current_range_male_after_pull is None or current_range_male_after_pull < self.range_valid_min_m:
                self.get_logger().error(f"Verify check fail - Invalid range male={current_range_male_after_pull}, stored={self.locked_connection_range}.")
                self.change_state(RobotState.HANDLING_FAILURE)
                return
            if self.locked_connection_range > 0.01: # Avoid division by zero
                difference = abs(current_range_male_after_pull - self.locked_connection_range) / self.locked_connection_range
                self.get_logger().info(f"Verify check: Male range {current_range_male_after_pull:.3f}, Locked range {self.locked_connection_range:.3f}. Diff {difference:.2%}")
                if difference < self.connection_verification_threshold:
                    self.get_logger().info(f"Connection verified for pair ({male_id}, {female_id})!")
                    self.change_state(RobotState.CONNECTED)
                else:
                    self.get_logger().error(f"Verification failed for pair ({male_id}, {female_id})! Range difference too large.")
                    self.change_state(RobotState.HANDLING_FAILURE)
            else:
                 self.get_logger().error("Verify check fail - Locked range was too small to verify.")
                 self.change_state(RobotState.HANDLING_FAILURE)


    # <<< MODIFIED FUNCTION >>>
    def _handle_connected_state(self):
        """Marks the current seeker/target pair as connected, updates statuses, and transitions back."""
        # IDs should still be set from VERIFYING_CONNECTION
        # Uses seeker/target as that's the pair that just connected
        if self.current_seeker_id is None or self.current_target_id is None:
             self.get_logger().error("Entered CONNECTED state without valid seeker/target pair IDs! Returning to MOVING.")
             # Ensure partner ID is cleared if error occurs
             self.current_target_partner_id = None
             # Reset potentially problematic statuses before moving on
             if self.current_seeker_id: self.robot_status[self.current_seeker_id] = AgentStatus.ACTIVE
             if self.current_target_id: self.robot_status[self.current_target_id] = AgentStatus.ACTIVE
             self.current_seeker_id = None # Clear invalid IDs
             self.current_target_id = None
             self.change_state(RobotState.MOVING_ALL_FORWARD)
             return

        seeker_id = self.current_seeker_id
        target_id = self.current_target_id
        partner_id = self.current_target_partner_id # May be None

        self.get_logger().info(f"Pair ({seeker_id}, {target_id}) successfully connected.")
        if partner_id is not None:
             # This case shouldn't happen if connecting a single seeker to a single target,
             # but is relevant if connecting TO a pair. The partner ID comes from the confirmation stage.
             self.get_logger().info(f"  (Target {target_id} was potentially part of existing pair with {partner_id})")

        # Ensure all involved robots are stopped immediately after verification success
        self._send_velocity(seeker_id, 0.0, 0.0)
        self._send_velocity(target_id, 0.0, 0.0)
        if partner_id is not None:
            # Ensure partner is also stopped, although it should have been stationary
            self._send_velocity(partner_id, self.partner_hold_speed, 0.0)

        # --- Update status and store pair ---
        self.get_logger().info(f"Updating status for {seeker_id} and {target_id} to CONNECTED.")
        self.robot_status[seeker_id] = AgentStatus.CONNECTED
        self.robot_status[target_id] = AgentStatus.CONNECTED
        # Partner status should already be CONNECTED if partner_id is valid
        # Store the newly formed connection link
        self.connected_pairs.add(tuple(sorted((seeker_id, target_id))))
        # Note: self.connected_pairs now represents pairwise links.
        # A chain 0-1-2 would be represented as {(0,1), (1,2)}
        self.get_logger().info(f"Updated Robot Status: {self.robot_status}")
        self.get_logger().info(f"Updated Connected Pairs: {self.connected_pairs}")
        # ---

        # Reset current pair info (done by change_state exiting CONNECTED)
        # Transition back to MOVING to allow other robots (like Agent 0) to proceed
        self.change_state(RobotState.MOVING_ALL_FORWARD)

    def _handle_failure_state(self):
        """Unified state for handling failures. Unlocks male (seeker). Keeps partner still."""
        self.get_logger().error("Handling connection sequence failure.")
        seeker_id = self.current_seeker_id
        target_id = self.current_target_id
        partner_id = self.current_target_partner_id
        male_id = self.current_male_id # Seeker is assumed male for this connection

        # Stop involved robots (seeker, target)
        if seeker_id is not None: self._send_velocity(seeker_id, 0.0, 0.0)
        if target_id is not None: self._send_velocity(target_id, 0.0, 0.0)
        # Partner should already be stopped, but ensure it
        if partner_id is not None: self._send_velocity(partner_id, 0.0, 0.0)


        # Attempt to unlock male (seeker's) joint
        if male_id is not None: # Should be same as seeker_id here
             self.get_logger().info(f"Attempting to unlock male joint ({male_id}).")
             self._set_joint(male_id, 'male', self.male_unlock_angle)
             time.sleep(0.5) # Give command time to send

        # Set status for involved robots (seeker, target) for reset
        if seeker_id is not None: self.robot_status[seeker_id] = AgentStatus.RESETTING
        if target_id is not None: self.robot_status[target_id] = AgentStatus.RESETTING
        # Partner status remains CONNECTED

        self.change_state(RobotState.RESETTING)

    # <<< MODIFIED FUNCTION >>>
    def _handle_resetting_state(self, time_in_state):
            """Moves seeker and target apart. Keeps partner still. Retries connection."""
            local_seeker_id = self.current_seeker_id
            local_target_id = self.current_target_id
            local_partner_id = self.current_target_partner_id # Get partner ID

            if local_seeker_id is None or local_target_id is None:
                # ... (recovery logic remains same - needs improvement if hit often) ...
                self.get_logger().warn("Entered RESETTING without seeker/target IDs! Attempting recovery by finding RESETTING agents.")
                resetting_agents = [aid for aid, status in self.robot_status.items() if status == AgentStatus.RESETTING]
                if len(resetting_agents) == 2:
                    self.get_logger().info(f"Recovered resetting pair (arbitrary roles): {resetting_agents[0]}, {resetting_agents[1]}")
                    local_seeker_id = resetting_agents[0]
                    local_target_id = resetting_agents[1]
                    self.current_seeker_id = local_seeker_id # Store back for next state
                    self.current_target_id = local_target_id
                    # Cannot recover partner ID reliably here
                    self.current_target_partner_id = None
                    local_partner_id = None
                else:
                    self.get_logger().error("Could not recover resetting pair. Returning to MOVING.")
                    for aid in resetting_agents: self.robot_status[aid] = AgentStatus.ACTIVE
                    self.current_seeker_id = None; self.current_target_id = None; self.current_male_id = None; self.current_female_id = None; self.locked_connection_range = None; self.current_target_partner_id = None
                    self.change_state(RobotState.MOVING_ALL_FORWARD)
                    return

            # --- Ensure Partner (if exists) stays stopped during reset ---
            if local_partner_id is not None:
                self.get_logger().debug(f"Reset: Keeping partner {local_partner_id} stopped.", throttle_duration_sec=2.0)
                self._send_velocity(local_partner_id, 0.0, 0.0)
            # ---

            # Use seeker's range to gauge distance
            current_range_seeker = self.agent_sensor_data['range'].get(local_seeker_id)
            valid_range = current_range_seeker if (current_range_seeker is not None and current_range_seeker >= self.range_valid_min_m) else self.reset_distance_m

            self.get_logger().info(f"Resetting pair ({local_seeker_id}, {local_target_id}): Range {valid_range:.3f} / Target {self.reset_distance_m:.3f}", throttle_duration_sec=1.0)

            if valid_range < self.reset_distance_m:
                self._send_velocity(local_seeker_id, self.reset_speed_seeker, 0.0) # Seeker reverse
                self._send_velocity(local_target_id, self.reset_speed_target, 0.0) # Target forward (away from seeker)
                self._centre_robot(local_seeker_id)
                self._centre_robot(local_target_id)
            else:
                self.get_logger().info(f"Reset complete for pair ({local_seeker_id}, {local_target_id}). Attempting connection again.")
                self._send_velocity(local_seeker_id, 0.0, 0.0)
                self._send_velocity(local_target_id, 0.0, 0.0)
                # Partner is already stopped

                # Mark involved robots as active, ready for next state
                self.robot_status[local_seeker_id] = AgentStatus.ACTIVE
                self.robot_status[local_target_id] = AgentStatus.ACTIVE
                # Partner status remains CONNECTED

                # Reset temporary connection vars (male/female/lock range)
                # Keep seeker/target/partner IDs for the retry
                self.current_male_id = None
                self.current_female_id = None
                self.locked_connection_range = None

                # Transition directly back to INITIATE_CONNECTION for this pair
                self.change_state(RobotState.INITIATE_CONNECTION)


    def _handle_all_connected_or_halted_state(self):
        """Final state. Logs status and ensures robots are stopped."""
        self.get_logger().info("All robots are connected or have halted.", once=True)
        self.get_logger().info(f"Final Robot Status: {self.robot_status}", once=True)
        self.get_logger().info(f"Connected Pairs: {self.connected_pairs}", once=True)

        # --- Add Velocity Logging Here (Before Stop Command) ---
        vel_log_msgs = [] # Collect messages to log together
        for agent_id in sorted(list(self.discovered_agent_ids)): # Log in sorted order
            last_lin, last_ang = self.agent_last_cmd_vel.get(agent_id, (None, None))
            # Format nicely, handle None case
            lin_str = f"{last_lin:.3f}" if last_lin is not None else "N/A"
            ang_str = f"{last_ang:.3f}" if last_ang is not None else "N/A"
            vel_log_msgs.append(f"  Agent {agent_id}: Last CmdVel (L={lin_str}, A={ang_str})")

        # Log all velocity messages together, only once
        if vel_log_msgs:
            log_string = "Last commanded velocities before final halt:\n" + "\n".join(vel_log_msgs)
            self.get_logger().info(log_string, once=True)
        # --- End Velocity Logging ---

        self.stop_all_robots()
        # Stay in this state - Node will continue spinning but do nothing further

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

    def stop_all_robots(self):
        """Sends zero velocity to all discovered robots."""
        self.get_logger().debug("--- Stopping all discovered robots ---")
        for agent_id in self.discovered_agent_ids:
            self._send_velocity(agent_id, 0.0, 0.0)

    def _send_velocity(self, agent_id, linear_x, angular_z):
        """Sends velocity command to the specified agent, avoids duplicates."""
        pub = self.agent_pubs['cmd_vel'].get(agent_id)
        if not pub:
            self.get_logger().warn(f"_send_velocity: No cmd_vel publisher for agent {agent_id}", throttle_duration_sec=10.0)
            return

        # Check cache to avoid redundant sends
        last_lin, last_ang = self.agent_last_cmd_vel.get(agent_id, (None, None))
        # if abs(linear_x - (last_lin or 0.0)) < 0.001 and abs(angular_z - (last_ang or 0.0)) < 0.001:
        #     return

        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        pub.publish(cmd)
        self.agent_last_cmd_vel[agent_id] = (linear_x, angular_z) # Update cache

      
    def _centre_robot(self, agent_id):
        """Applies centering correction based on roll for the specified agent. Logs roll and calculated angular_z."""
        if agent_id not in self.agent_sensor_data['roll'] or agent_id not in self.agent_pubs['cmd_vel']:
             self.get_logger().warn(f"Cannot center agent {agent_id}: Missing roll data or cmd_vel publisher.", throttle_duration_sec=10.0)
             return

        roll_rad = self.agent_sensor_data['roll'].get(agent_id, 0.0)
        # Convert roll to degrees for calculation and logging
        roll_deg = math.degrees(roll_rad)

        current_linear, last_angular = self.agent_last_cmd_vel.get(agent_id, (0.0, 0.0))

        is_reversing = current_linear < -0.001
        # Threshold in degrees now, makes more intuitive sense with the calculation
        angular_z_cmd = 0.0 # Default to no correction

        # --- Calculate Correction ---
        if abs(roll_deg) > self.roll_threshold_deg:
            # Determine gain based on direction (assuming self.kp_centering_forward is negative)
            effective_k_p = -self.kp_centering_forward if is_reversing else self.kp_centering_forward
            # Calculate angular command using roll in *degrees*
            angular_z_cmd = effective_k_p * roll_deg
            max_correction_vel = 0.3 # Allow slightly higher correction speed maybe?
            angular_z_cmd = max(-max_correction_vel, min(max_correction_vel, angular_z_cmd))
        # --- End Calculation ---

        # --- Logging Added Here ---
        # Log the current roll and the *calculated* angular command before checking thresholds to send
        # self.get_logger().info(
        #     f"Center Agent {agent_id}: Roll={roll_deg:.2f} deg => Calc AngZ={angular_z_cmd:.3f} rad/s",
        #     throttle_duration_sec=1.0 # Throttle logging per agent
        # )
        # --- End Logging ---

        # --- Send Velocity Command (Original Logic) ---
        # Check if the calculated command is significantly different from the last sent one
        if abs(angular_z_cmd - last_angular) > 0.01: # Tolerance for change
             # Send the *current* linear speed along with the *new* angular command
             self._send_velocity(agent_id, current_linear, angular_z_cmd)
        # If the correction is now very small, but we were turning before, send zero angular velocity
        elif abs(angular_z_cmd) < 0.01 and abs(last_angular) > 0.01:
             self._send_velocity(agent_id, current_linear, 0.0)
        # --- End Send Velocity ---

    

    def _set_joint(self, agent_id, joint_type, position):
        """Commands 'male' or 'female' joint for the specified agent."""
        dict_key = f"{joint_type}_joint" # 'male_joint' or 'female_joint'
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
        point.time_from_start = Duration(sec=2, nanosec=0) # Adjust time as needed
        msg.points.append(point)
        pub.publish(msg)

        if joint_type == 'male':
             self.agent_sensor_data['male_locked'][agent_id] = False
             self.agent_sensor_data['male_unlocked'][agent_id] = False

    # ==========================================================================
    # Subscriber Callbacks (No changes from previous version)
    # ==========================================================================
    def _calculate_imu_rp(self, msg: Imu):
        """Helper to calculate roll/pitch from IMU accelerometer data."""
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        accel_mag_sq = ax**2 + ay**2 + az**2
        if accel_mag_sq < 0.01: return None, None
        try:
            roll = math.atan2(-ax, math.sqrt(ay**2 + az**2))
            pitch = math.atan2(ay, math.sqrt(ax**2 + az**2))
            return roll, pitch
        except ValueError:
            self.get_logger().warn("Math error in IMU calculation.", once=True)
            return None, None

    def imu_callback(self, msg: Imu, agent_id: int):
        """Stores roll/pitch for the specific agent."""
        roll, pitch = self._calculate_imu_rp(msg)
        if roll is not None: self.agent_sensor_data['roll'][agent_id] = roll
        if pitch is not None: self.agent_sensor_data['pitch'][agent_id] = pitch

    def range_callback(self, msg: Range, agent_id: int):
        """Stores valid range data for the specific agent."""
        valid_range = None
        if math.isfinite(msg.range) and msg.range >= msg.min_range and msg.range <= msg.max_range:
             valid_range = msg.range
        if self.agent_sensor_data['range'].get(agent_id) != valid_range:
            self.agent_sensor_data['range'][agent_id] = valid_range

    def alignment_callback(self, msg: Bool, agent_id: int):
        """Stores alignment status for the specific agent."""
        if self.agent_sensor_data['aligned'].get(agent_id) != msg.data:
             self.agent_sensor_data['aligned'][agent_id] = msg.data

    def male_joint_state_callback(self, msg: JointTrajectoryControllerState, agent_id: int):
        """Stores male lock/unlock status based on reported joint position."""
        # Check multiple fields for position, prioritizing 'actual'
        pos_list = None
        if msg.actual and msg.actual.positions:
            pos_list = msg.actual.positions
        elif msg.feedback and msg.feedback.positions: # Fallback to feedback
            pos_list = msg.feedback.positions
        # Add other fallbacks if needed e.g. desired?

        if pos_list is None or len(pos_list) == 0: return

        current_pos = pos_list[0]
        is_unlocked = abs(current_pos - self.male_unlock_angle) < self.joint_pos_tolerance
        is_locked = abs(current_pos - self.male_lock_angle) < self.joint_pos_tolerance

        if self.agent_sensor_data['male_unlocked'].get(agent_id) != is_unlocked:
            self.agent_sensor_data['male_unlocked'][agent_id] = is_unlocked
        if self.agent_sensor_data['male_locked'].get(agent_id) != is_locked:
            self.agent_sensor_data['male_locked'][agent_id] = is_locked


# ==========================================================================
# Main Execution
# ==========================================================================
def main(args=None):
    rclpy.init(args=args)
    node = None # Initialize node to None
    try:
        node = Connect_Robots()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("Ctrl-C detected, shutting down.")
    except Exception as e:
        if node:
            tb_str = traceback.format_exc() # Get traceback string
            node.get_logger().fatal(f"Unhandled exception in node: {e}\nTraceback:\n{tb_str}")
        else:
            # Also print traceback if initialization fails
            print(f"Exception during node initialization: {e}")
            traceback.print_exc()
    finally:
        if node and rclpy.ok():
             node.get_logger().info("Stopping robots before shutdown...")
             node.stop_all_robots()
             node.destroy_node()
        # Ensure shutdown happens even if node creation failed
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()