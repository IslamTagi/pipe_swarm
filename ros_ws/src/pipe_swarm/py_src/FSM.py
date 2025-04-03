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
        self.current_target_id = None       # Agent being targeted for connection (specific rear robot)
        self.current_male_id = None         # Cache male role for current pair
        self.current_female_id = None       # Cache female role for current pair
        self.locked_connection_range = None # Range stored just before verification pull
        # self.current_target_partner_id = None # ID of target's partner if target is already connected (REPLACED by group)
        self.current_target_group = set()   # Stores IDs in the target's connected group

        # --- State Machine ---
        self.current_state = RobotState.DISCOVERING_ROBOTS
        self.state_enter_time = self.get_clock().now()
        self.get_logger().info(f"Node starting in state: {self.current_state.name}")

        # --- Constants and Configuration ---
        # Timing
        self.initial_delay_sec = 2.0
        self.alignment_timeout_sec = 40.0
        self.docking_duration_sec = 10.0 # Reduced from 20 for testing
        self.locking_timeout_sec = 10.0
        self.verification_duration_sec = 5.0
        self.confirmation_timeout_per_robot_sec = 3.0
        # Speeds
        self.forward_speed = 0.1
        self.approach_speed_gain = 0.1
        self.approach_min_speed = 0.02
        self.docking_speed_male = 0.01
        self.docking_speed_female = -0.015 # Negative indicates reverse
        self.verification_pull_speed = -0.05
        self.reset_speed_seeker = -0.02
        self.reset_speed_target = 0.01
        self.confirmation_move_speed = 0.05
        self.aligning_turn_speed = 0.15
        self.partner_hold_speed = 0.0 # Set to 0, rely on physics/stop commands first
        # Thresholds & Distances
        self.robot_detection_threshold = 0.2
        self.confirmation_detection_threshold = 0.2
        self.connection_verification_threshold = 0.2
        self.close_distance_m = 0.15
        self.reset_distance_m = 0.3
        self.range_valid_min_m = 0.05
        self.roll_threshold_deg = 1.5
        # Joints
        self.male_lock_angle = math.pi / 2.0
        self.male_unlock_angle = 0.0
        self.female_lift_angle = 0.0
        self.joint_pos_tolerance = 0.1
        # Control
        self.kp_centering_forward = -0.05 # Gain for centering controller

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
        self.get_logger().debug("--- discover_robots() function called ---", throttle_duration_sec=5.0)
        if self.discovery_complete:
            return

        self.get_logger().info(f"Discovery attempt {self.discovery_attempts + 1}/{self.max_discovery_attempts}...")
        found_new = False
        current_ids_found_this_scan = set()

        try:
            topic_list = self.get_topic_names_and_types()
            agent_topic_pattern = re.compile(r'/agent_(\d+)/robot_description')

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

        if not found_new and self.num_discovered_robots > 0:
            self.discovery_attempts += 1
            self.get_logger().info(f"Discovery stable ({self.discovery_attempts}/{self.max_discovery_attempts}). Found {self.num_discovered_robots} robots: {sorted(list(self.discovered_agent_ids))}")
            if self.discovery_attempts >= self.max_discovery_attempts:
                self.get_logger().info("Discovery complete.")
                self.discovery_complete = True
        elif found_new:
            self.discovery_attempts = 0
            self.get_logger().info(f"Found new robot(s). Current count: {self.num_discovered_robots}. Resetting stability counter.")
        else:
            self.discovery_attempts += 1
            if self.discovery_attempts >= self.max_discovery_attempts:
                 self.get_logger().error("Discovery timed out. No robots found.")
                 self.discovery_complete = True

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

        self.agent_sensor_data['roll'][agent_id] = 0.0
        self.agent_sensor_data['pitch'][agent_id] = 0.0
        self.agent_sensor_data['range'][agent_id] = None
        self.agent_sensor_data['aligned'][agent_id] = False
        self.agent_sensor_data['male_locked'][agent_id] = False
        self.agent_sensor_data['male_unlocked'][agent_id] = True
        self.agent_last_cmd_vel[agent_id] = (0.0, 0.0)
        self.robot_status[agent_id] = AgentStatus.UNKNOWN
        self.initial_scan_ranges[agent_id] = None

        try:
            self.agent_pubs['cmd_vel'][agent_id] = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
            self.agent_pubs['male_joint'][agent_id] = self.create_publisher(JointTrajectory, f"/{ns}/male_joint_trajectory_controller/joint_trajectory", 10)
            self.agent_pubs['female_joint'][agent_id] = self.create_publisher(JointTrajectory, f"/{ns}/female_joint_trajectory_controller/joint_trajectory", 10)

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
        process_in_progress_states = {
            RobotState.GLOBAL_DETECTION_HALT, RobotState.SEQUENTIAL_CONFIRMATION,
            RobotState.INITIATE_CONNECTION, RobotState.APPROACHING_TARGET,
            RobotState.CHECKING_ALIGNMENT, RobotState.ALIGNING, RobotState.DOCKING,
            RobotState.LOCKING, RobotState.VERIFYING_CONNECTION,
            RobotState.HANDLING_FAILURE, RobotState.RESETTING
        }
        connection_or_confirmation_in_progress = self.current_state in process_in_progress_states

        active_robots_exist = any(status == AgentStatus.ACTIVE
                                  for aid, status in self.robot_status.items()
                                  if aid in self.discovered_agent_ids)

        current_state_valid_for_check = self.current_state not in [
            RobotState.DISCOVERING_ROBOTS, RobotState.IDLE,
            RobotState.INITIALIZING_ROBOTS, RobotState.FAILED,
            RobotState.ALL_CONNECTED_OR_HALTED, RobotState.CONNECTED
        ]

        should_enter_final_state = not active_robots_exist and \
                                   not connection_or_confirmation_in_progress and \
                                   current_state_valid_for_check

        self.get_logger().debug(f"Terminal Check Tick: State={self.current_state.name}, "
                               f"active_exist={active_robots_exist}, "
                               f"proc_in_prog={connection_or_confirmation_in_progress}, "
                               f"state_valid={current_state_valid_for_check}, "
                               f"FINAL_COND_RESULT={should_enter_final_state}", throttle_duration_sec=5.0)

        if should_enter_final_state:
             log_reason = f"active_exist={active_robots_exist}, proc_in_prog={connection_or_confirmation_in_progress}"
             self.get_logger().info(f"Condition met for final state ({log_reason}). Entering final state.")
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
                takes_time_arg = self.current_state in [
                    RobotState.IDLE, RobotState.SEQUENTIAL_CONFIRMATION,
                    RobotState.ALIGNING, RobotState.DOCKING, RobotState.LOCKING,
                    RobotState.VERIFYING_CONNECTION, RobotState.RESETTING
                ]
                if takes_time_arg: handler(time_in_state)
                else: handler()
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
        self.get_logger().debug(f"Current state BEFORE change: {previous_state.name}")
        self.get_logger().debug(f"Target new state: {new_state.name}")
        self.get_logger().debug(f"Vars BEFORE: detector={self.detecting_agent_id}, seeker={self.current_seeker_id}, target={self.current_target_id}, group={self.current_target_group}")

        self.get_logger().info(f"Changing state from {previous_state.name} to {new_state.name}")

        self.current_state = new_state
        self.state_enter_time = self.get_clock().now()

        self.get_logger().debug(f"Current state AFTER change: {self.current_state.name}")

        # --- Reset temporary variables based on state transitions ---
        confirmation_process_states = [RobotState.GLOBAL_DETECTION_HALT, RobotState.SEQUENTIAL_CONFIRMATION]
        prev_in_conf = previous_state in confirmation_process_states
        new_in_conf = new_state in confirmation_process_states
        if prev_in_conf and not new_in_conf:
             self.get_logger().debug(f"Resetting confirmation vars (leaving confirmation process)")
             self.detecting_agent_id = None
             self.detection_range = None
             self.confirmation_target_ids = []
             self.confirmation_check_index = 0
             self.confirmation_start_time = None

        connection_sequence_states = [
            RobotState.INITIATE_CONNECTION, RobotState.APPROACHING_TARGET, RobotState.CHECKING_ALIGNMENT,
            RobotState.ALIGNING, RobotState.DOCKING, RobotState.LOCKING, RobotState.VERIFYING_CONNECTION,
            RobotState.HANDLING_FAILURE, RobotState.RESETTING, RobotState.CONNECTED
        ]
        prev_in_conn = previous_state in connection_sequence_states
        new_in_conn = new_state in connection_sequence_states
        if prev_in_conn and not new_in_conn:
             self.get_logger().debug(f"Resetting connection sequence vars (leaving sequence)")
             self.current_seeker_id = None
             self.current_target_id = None
             self.current_male_id = None
             self.current_female_id = None
             self.locked_connection_range = None
             # self.current_target_partner_id = None # No longer used
             self.current_target_group = set() # <<< RESET GROUP
        # ---

        self.get_logger().debug(f"Vars AFTER change_state: detector={self.detecting_agent_id}, seeker={self.current_seeker_id}, target={self.current_target_id}, group={self.current_target_group}")
        self.get_logger().debug(f"--- change_state END ---")

    # ==========================================================================
    # State Handling Functions
    # ==========================================================================

    def _handle_idle_state(self, time_in_state):
        """Waits for initial delay and ensures resources are ready."""
        all_ready = all(agent_id in self.agent_pubs.get('cmd_vel', {})
                        for agent_id in self.discovered_agent_ids)
        if not all_ready:
             self.get_logger().info("Waiting for all cmd_vel publishers...", throttle_duration_sec=2.0)
             return

        if time_in_state >= self.initial_delay_sec:
            self.get_logger().info("Initial delay complete.")
            self.change_state(RobotState.INITIALIZING_ROBOTS)

    def _handle_initializing_robots_state(self):
        """Sets initial joint positions, status, and gets initial range scans for all robots."""
        self.get_logger().info("Initializing all robots...")
        all_scans_obtained = True
        for agent_id in self.discovered_agent_ids:
            if self.robot_status.get(agent_id, AgentStatus.UNKNOWN) in [AgentStatus.UNKNOWN, AgentStatus.FAILED]:
                self._set_joint(agent_id, 'male', self.male_unlock_angle)
                self._set_joint(agent_id, 'female', self.female_lift_angle)
                self.robot_status[agent_id] = AgentStatus.ACTIVE
                self.initial_scan_ranges[agent_id] = None
                self.get_logger().info(f"  Agent {agent_id} set to ACTIVE.")

            if self.initial_scan_ranges.get(agent_id) is None:
                current_range = self.agent_sensor_data['range'].get(agent_id, None)
                if current_range is not None and current_range >= self.range_valid_min_m:
                    self.initial_scan_ranges[agent_id] = current_range
                    self.get_logger().info(f"  Stored initial range for {agent_id}: {current_range:.3f} m")
                else:
                    self.get_logger().info(f"  Waiting for initial range for {agent_id}...")
                    all_scans_obtained = False

        if all_scans_obtained:
            self.get_logger().info("All robots initialized and have initial scans.")
            time.sleep(0.5) # Short delay
            self.change_state(RobotState.MOVING_ALL_FORWARD)

    def _handle_moving_all_forward_state(self):
        """Moves all ACTIVE robots forward and checks for range changes."""
        detection_triggered = False
        active_agent_found = False
        for agent_id in self.discovered_agent_ids:
            agent_status = self.robot_status.get(agent_id)
            self.get_logger().debug(f"MOVING_FWD Check: Agent {agent_id} Status: {agent_status.name if agent_status else 'None'}", throttle_duration_sec=5.0)

            if agent_status == AgentStatus.ACTIVE:
                active_agent_found = True
                self.get_logger().debug(f"MOVING_FWD: Commanding Agent {agent_id} forward.", throttle_duration_sec=5.0)
                self._send_velocity(agent_id, self.forward_speed, 0.0)
                self._centre_robot(agent_id)

                current_range = self.agent_sensor_data['range'].get(agent_id, None)
                initial_range = self.initial_scan_ranges.get(agent_id, None)

                if initial_range is None:
                    if current_range is not None and current_range >= self.range_valid_min_m:
                        self.initial_scan_ranges[agent_id] = current_range
                        self.get_logger().warn(f"Re-acquired initial scan for {agent_id} in MOVING state: {current_range:.3f} m")
                    continue

                if current_range is not None and current_range >= self.range_valid_min_m and initial_range > 0.01:
                    difference = abs(current_range - initial_range) / initial_range
                    if difference > self.robot_detection_threshold:
                        self.get_logger().info(f"Agent {agent_id} detected range change! (R:{current_range:.3f}, I:{initial_range:.3f}, D:{difference:.2%}). Halting.")
                        self.detecting_agent_id = agent_id
                        self.detection_range = current_range
                        self.robot_status[agent_id] = AgentStatus.DETECTED_SOMETHING
                        detection_triggered = True
                        break # Process first detection only
                elif initial_range <= 0.01 and current_range is not None:
                     self.get_logger().warn(f"Agent {agent_id} initial range {initial_range} too small.", throttle_duration_sec=5.0)


        if detection_triggered:
            self.stop_all_robots()
            self.change_state(RobotState.GLOBAL_DETECTION_HALT)
        elif not active_agent_found and self.current_state == RobotState.MOVING_ALL_FORWARD:
             # Should be handled by the terminal check at the start of state_machine_tick
             self.get_logger().debug("No active robots found in MOVING_ALL_FORWARD (Terminal check should trigger).")


    def _handle_global_detection_halt_state(self):
        """Stops all robots and prepares list for sequential confirmation, including connected robots."""
        if self.detecting_agent_id is None:
            self.get_logger().error("Entered GLOBAL_DETECTION_HALT without detecting_agent_id!")
            self.change_state(RobotState.FAILED)
            return

        self.get_logger().info(f"Global halt triggered by Agent {self.detecting_agent_id}. Preparing confirmation checks.")
        self.stop_all_robots()

        valid_target_statuses = {AgentStatus.ACTIVE, AgentStatus.OBSTACLE_NEAR, AgentStatus.CONNECTED}
        excluded_statuses = {
            AgentStatus.CONNECTION_SEEKER, AgentStatus.CONNECTION_TARGET,
            AgentStatus.RESETTING, AgentStatus.CONFIRMATION_TARGET, AgentStatus.FAILED
        }

        self.confirmation_target_ids = sorted([
            aid for aid in self.discovered_agent_ids
            if aid != self.detecting_agent_id and \
               self.robot_status.get(aid) in valid_target_statuses and \
               self.robot_status.get(aid) not in excluded_statuses
        ])

        self.get_logger().debug(f"Built confirmation_target_ids: {self.confirmation_target_ids}")

        if not self.confirmation_target_ids:
             self.get_logger().warn(f"Agent {self.detecting_agent_id} detected but no suitable targets found. Assuming obstacle.")
             self._handle_obstacle_outcome()
             return

        self.confirmation_check_index = 0
        self.confirmation_start_time = self.get_clock().now()

        self.get_logger().info(f"Potential targets: {self.confirmation_target_ids}. Checking {self.confirmation_target_ids[0]}.")
        self.change_state(RobotState.SEQUENTIAL_CONFIRMATION)


    def _handle_sequential_confirmation_state(self, time_in_state):
        """Sequentially moves potential targets and checks detector's range. Handles groups."""
        if self.detecting_agent_id is None or not self.confirmation_target_ids or self.confirmation_start_time is None:
            self.get_logger().error("Entered SEQUENTIAL_CONFIRMATION with invalid state vars!")
            self.change_state(RobotState.FAILED)
            return

        target_check_id = self.confirmation_target_ids[self.confirmation_check_index]
        self.get_logger().debug(f"Checking target Agent {target_check_id} (Index {self.confirmation_check_index}). Moving it.", throttle_duration_sec=1.0)

        original_status_before_check = self.robot_status.get(target_check_id, AgentStatus.UNKNOWN)
        if original_status_before_check == AgentStatus.UNKNOWN:
            self.get_logger().warn(f"Agent {target_check_id} had UNKNOWN status before check.")

        # Mark target being checked (temporary status)
        self.robot_status[target_check_id] = AgentStatus.CONFIRMATION_TARGET

        # Check detector's initial range validity
        detector_current_range = self.agent_sensor_data['range'].get(self.detecting_agent_id)
        if self.detection_range is None or self.detection_range < self.range_valid_min_m:
             self.get_logger().warn(f"Initial detection range invalid ({self.detection_range}). Cannot confirm. Assuming obstacle.")
             self._send_velocity(target_check_id, 0.0, 0.0)
             # Revert status
             if original_status_before_check in {AgentStatus.ACTIVE, AgentStatus.UNKNOWN}:
                 self.robot_status[target_check_id] = AgentStatus.ACTIVE
             elif original_status_before_check != AgentStatus.CONFIRMATION_TARGET:
                 self.robot_status[target_check_id] = original_status_before_check
             self._handle_obstacle_outcome()
             return
        if detector_current_range is None or detector_current_range < self.range_valid_min_m:
            self.get_logger().warn(f"Detector {self.detecting_agent_id} range invalid ({detector_current_range}) during check. Assuming obstacle.")
            self._send_velocity(target_check_id, 0.0, 0.0)
             # Revert status
            if original_status_before_check in {AgentStatus.ACTIVE, AgentStatus.UNKNOWN}:
                 self.robot_status[target_check_id] = AgentStatus.ACTIVE
            elif original_status_before_check != AgentStatus.CONFIRMATION_TARGET:
                 self.robot_status[target_check_id] = original_status_before_check
            self._handle_obstacle_outcome()
            return

        # Move the target robot forward slightly
        self._send_velocity(target_check_id, self.confirmation_move_speed, 0.0)
        self._centre_robot(target_check_id) # Centre the one moving

        # Check detector range change after moving target
        detector_current_range = self.agent_sensor_data['range'].get(self.detecting_agent_id)
        difference = 0.0
        if self.detection_range is not None and self.detection_range > 0.01 and detector_current_range is not None:
             difference = abs(detector_current_range - self.detection_range) / self.detection_range

        if difference > self.confirmation_detection_threshold:
            # --- Confirmation Success ---
            self.get_logger().info(f"Range changed for detector {self.detecting_agent_id} (Now:{detector_current_range:.3f}, Initial:{self.detection_range:.3f}, Diff:{difference:.2%}). Confirmed {target_check_id}.")
            self._send_velocity(target_check_id, 0.0, 0.0) # Stop the checked target

            confirmed_seeker_id = self.detecting_agent_id
            # Find the entire group the confirmed target belongs to
            self.current_target_group = self._find_connected_group(target_check_id)
            self.get_logger().info(f"Confirmed Agent {target_check_id} belongs to group: {self.current_target_group}")

            if not self.current_target_group:
                 self.get_logger().error(f"CRITICAL: Found empty group for confirmed target {target_check_id}")
                 self._handle_obstacle_outcome()
                 return

            # Designate the lowest ID in the group as the actual target for docking
            confirmed_target_id = min(self.current_target_group)
            self.get_logger().info(f"Designating rear robot {confirmed_target_id} from group as connection target.")

            # Final Check and Assignment
            if confirmed_seeker_id is None or confirmed_target_id is None:
                self.get_logger().error(f"CRITICAL: Cannot initiate. Seeker({confirmed_seeker_id}) or Target({confirmed_target_id}) is None.")
                # Revert status before handling obstacle
                if original_status_before_check in {AgentStatus.ACTIVE, AgentStatus.UNKNOWN}:
                    self.robot_status[target_check_id] = AgentStatus.ACTIVE
                elif original_status_before_check != AgentStatus.CONFIRMATION_TARGET:
                    self.robot_status[target_check_id] = original_status_before_check
                self._handle_obstacle_outcome()
                return

            self.get_logger().debug(f"SEQ_CONFIRM: Setting Seeker={confirmed_seeker_id}, Target={confirmed_target_id}, Group={self.current_target_group}")
            self.current_seeker_id = confirmed_seeker_id
            self.current_target_id = confirmed_target_id

            self.change_state(RobotState.INITIATE_CONNECTION)
            return # Exit state handler

        # --- Timeout check ---
        time_since_check_started = (self.get_clock().now() - self.confirmation_start_time).nanoseconds / 1e9
        if time_since_check_started > self.confirmation_timeout_per_robot_sec:
             self.get_logger().warn(f"Confirmation check for {target_check_id} timed out (Range:{detector_current_range:.3f}, Initial:{self.detection_range:.3f}).")
             self._send_velocity(target_check_id, 0.0, 0.0) # Stop check target

             # Revert status
             if original_status_before_check in {AgentStatus.ACTIVE, AgentStatus.UNKNOWN}:
                 self.get_logger().info(f"Timeout: Resetting {target_check_id} to ACTIVE.")
                 self.robot_status[target_check_id] = AgentStatus.ACTIVE
             elif original_status_before_check != AgentStatus.CONFIRMATION_TARGET:
                 self.get_logger().info(f"Timeout: Reverting {target_check_id} to {original_status_before_check.name}.")
                 self.robot_status[target_check_id] = original_status_before_check
             else:
                 self.get_logger().warn(f"Timeout: Original status {original_status_before_check.name} unexpected. Setting {target_check_id} to ACTIVE.")
                 self.robot_status[target_check_id] = AgentStatus.ACTIVE

             # Clear group state before moving to next
             self.current_target_group = set()

             # Move to the next target or handle obstacle
             self.confirmation_check_index += 1
             if self.confirmation_check_index >= len(self.confirmation_target_ids):
                 self.get_logger().warn(f"All potential targets checked for {self.detecting_agent_id}. Assuming obstacle.")
                 self._handle_obstacle_outcome()
             else:
                 next_target_id = self.confirmation_target_ids[self.confirmation_check_index]
                 self.confirmation_start_time = self.get_clock().now()
                 self.get_logger().info(f"Moving to check next target: {next_target_id} (Index {self.confirmation_check_index}).")
                 # Stay in SEQUENTIAL_CONFIRMATION state


    def _handle_obstacle_outcome(self):
        """Helper to handle when sequential confirmation determines an obstacle."""
        checked_target_ids = list(self.confirmation_target_ids)
        detecting_agent_id_local = self.detecting_agent_id

        if detecting_agent_id_local is not None:
             if self.robot_status.get(detecting_agent_id_local) != AgentStatus.OBSTACLE_NEAR:
                self.robot_status[detecting_agent_id_local] = AgentStatus.OBSTACLE_NEAR
                self.get_logger().info(f"Agent {detecting_agent_id_local} marked as OBSTACLE_NEAR.")
                self.initial_scan_ranges[detecting_agent_id_local] = None
        else:
             self.get_logger().error("Obstacle outcome handler called without detecting_agent_id!")

        self.get_logger().info(f"Resetting status for checked targets after obstacle: {checked_target_ids}")
        for checked_id in checked_target_ids:
            if checked_id == detecting_agent_id_local: continue # Don't reset detector

            current_status = self.robot_status.get(checked_id)
            resettable_statuses = {AgentStatus.CONFIRMATION_TARGET, AgentStatus.ACTIVE, AgentStatus.UNKNOWN}
            if current_status in resettable_statuses:
                self.get_logger().info(f"  Resetting Agent {checked_id} from {current_status.name if current_status else 'None'} to ACTIVE.")
                self.robot_status[checked_id] = AgentStatus.ACTIVE
            else:
                self.get_logger().info(f"  Skipping status reset for Agent {checked_id} (Status: {current_status.name if current_status else 'None'})")

        self.change_state(RobotState.MOVING_ALL_FORWARD) # Temporary vars cleared by change_state


    def _handle_initiate_connection_state(self):
        """Determines roles, commands joints, updates status. Uses target group."""
        if self.current_seeker_id is None or self.current_target_id is None:
            self.get_logger().error("Entered INITIATE_CONNECTION without seeker/target IDs!")
            self.change_state(RobotState.FAILED)
            return

        seeker_id = self.current_seeker_id
        target_id = self.current_target_id # This is the specific robot to dock with (lowest ID)
        target_group = self.current_target_group

        # --- Assign roles ---
        self.current_male_id = seeker_id
        self.current_female_id = target_id
        # ---

        # --- Log based on group size ---
        if len(target_group) > 1:
            other_members = target_group - {target_id} # Get other members excluding the direct target
            self.get_logger().info(f"Initiating connection to GROUP: Seeker={seeker_id}, Target={target_id}, Other Group Members={other_members} | New Male={self.current_male_id}, New Female={self.current_female_id}")
            # Ensure other members are stopped (or held) initially
            for member_id in other_members:
                 # Command stop first, rely on physics/damping to hold
                 self._send_velocity(member_id, 0.0, 0.0)
                 # self._send_velocity(member_id, self.partner_hold_speed, 0.0) # Optional hold speed
                 if self.robot_status.get(member_id) != AgentStatus.CONNECTED:
                     self.get_logger().warn(f"Group Member {member_id} status was not CONNECTED ({self.robot_status.get(member_id)}). Setting.")
                     self.robot_status[member_id] = AgentStatus.CONNECTED
        else:
            self.get_logger().info(f"Initiating connection to SINGLE: Seeker={seeker_id}, Target={target_id} | New Male={self.current_male_id}, New Female={self.current_female_id}")
        # ---

        # Command joints for the connecting pair (Seeker=Male, Target=Female=Rear of Group)
        self._set_joint(self.current_male_id, 'male', self.male_unlock_angle)
        self._set_joint(self.current_female_id, 'female', self.female_lift_angle)

        # Update status for seeker and specific target
        self.robot_status[seeker_id] = AgentStatus.CONNECTION_SEEKER
        self.robot_status[target_id] = AgentStatus.CONNECTION_TARGET
        # Other group members remain CONNECTED

        time.sleep(0.5) # Allow time for joints to start moving
        self.change_state(RobotState.APPROACHING_TARGET)


    def _handle_approaching_target_state(self):
        """Moves seeker towards target. Keeps entire target group still."""
        if self.current_seeker_id is None or self.current_target_id is None: return self._fail_state_missing_ids("APPROACHING_TARGET")

        seeker_id = self.current_seeker_id
        target_id = self.current_target_id # Direct target ID
        target_group = self.current_target_group

        # --- Ensure Target Group is stopped/held ---
        if not target_group:
             self.get_logger().error("Approaching target but target group is empty!")
             self._send_velocity(seeker_id, 0.0, 0.0)
             self.change_state(RobotState.HANDLING_FAILURE)
             return

        self.get_logger().debug(f"Approach: Stopping target group {target_group}", throttle_duration_sec=5.0)
        for member_id in target_group:
            # Command stop first, rely on damping/friction
            self._send_velocity(member_id, 0.0, 0.0)
            # Optional: Use partner_hold_speed if stopping isn't enough due to drift
            # self._send_velocity(member_id, self.partner_hold_speed, 0.0)
        # ---

        # Move seeker
        self._centre_robot(seeker_id) # Only centre the seeker

        current_range_seeker = self.agent_sensor_data['range'].get(seeker_id)
        if current_range_seeker is None or current_range_seeker < self.range_valid_min_m:
             self.get_logger().warn(f"Seeker {seeker_id} range lost/invalid ({current_range_seeker}) during approach.", throttle_duration_sec=5.0)
             self._send_velocity(seeker_id, 0.0, 0.0) # Stop seeker
             return

        if current_range_seeker < self.close_distance_m:
            self.get_logger().info(f"Seeker {seeker_id} reached close distance ({current_range_seeker:.3f} m) to target {target_id}.")
            self._send_velocity(seeker_id, 0.0, 0.0)
            self.change_state(RobotState.CHECKING_ALIGNMENT)
        else:
            # Calculate proportional speed
            speed = max(self.approach_min_speed, self.approach_speed_gain * (current_range_seeker - self.close_distance_m))
            # Get centering command for seeker
            angular_z = self._calculate_centering_angular_z(seeker_id) if self.kp_centering_forward != 0 else 0.0 # Calculate if gain is non-zero
            self._send_velocity(seeker_id, speed, angular_z) # Send combined command


    def _handle_checking_alignment_state(self):
        """Checks seeker alignment. Keeps target group still."""
        if self.current_seeker_id is None or self.current_target_id is None: return self._fail_state_missing_ids("CHECKING_ALIGNMENT")

        seeker_id = self.current_seeker_id
        target_id = self.current_target_id
        target_group = self.current_target_group

        # --- Ensure Target Group is stopped/held ---
        if not target_group:
            self.get_logger().error("Check Align state reached with empty target group!")
            self._send_velocity(seeker_id, 0.0, 0.0) # Stop seeker too
            self.change_state(RobotState.HANDLING_FAILURE)
            return
        self.get_logger().debug(f"Check Align: Stopping target group {target_group}", throttle_duration_sec=5.0)
        for member_id in target_group:
            self._send_velocity(member_id, 0.0, 0.0)
            # self._send_velocity(member_id, self.partner_hold_speed, 0.0) # Optional hold speed
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
        """Rotates the specific target robot. Keeps seeker and rest of target group still."""
        if self.current_seeker_id is None or self.current_target_id is None: return self._fail_state_missing_ids("ALIGNING")

        seeker_id = self.current_seeker_id
        target_id = self.current_target_id # Only this robot rotates
        target_group = self.current_target_group
        other_group_members = target_group - {target_id}

        # --- Ensure Seeker and Other Group Members stay stopped ---
        self._send_velocity(seeker_id, 0.0, 0.0)
        if other_group_members:
             self.get_logger().debug(f"Align: Stopping other group members {other_group_members}", throttle_duration_sec=2.0)
             for member_id in other_group_members:
                  self._send_velocity(member_id, 0.0, 0.0)
        # ---

        # Only centre the target being rotated
        self._centre_robot(target_id)

        seeker_aligned = self.agent_sensor_data['aligned'].get(seeker_id, False)
        self.get_logger().debug(f"ALIGNING: Seeker {seeker_id} aligned={seeker_aligned} (Time:{time_in_state:.2f}s)", throttle_duration_sec=0.5)

        if seeker_aligned:
            self.get_logger().info("Alignment successful during maneuver.")
            self._send_velocity(target_id, 0.0, 0.0) # Stop target rotation
            self.change_state(RobotState.DOCKING)
            return

        if time_in_state > self.alignment_timeout_sec:
            self.get_logger().error(f"Alignment timed out for pair ({seeker_id}, {target_id})!")
            self._send_velocity(target_id, 0.0, 0.0) # Stop target
            # Ensure rest of group is also stopped
            for member_id in other_group_members: self._send_velocity(member_id, 0.0, 0.0)
            self.change_state(RobotState.HANDLING_FAILURE)
            return

        # Rotate target robot using oscillating pattern
        time_in_cycle = time_in_state % 4.0
        angular_z = 0.0
        if time_in_cycle <= 1.0: angular_z = self.aligning_turn_speed
        elif time_in_cycle <= 3.0: angular_z = -self.aligning_turn_speed
        else: angular_z = self.aligning_turn_speed

        # Combine rotation with centering (or just send rotation if centering is off)
        # angular_z_center = self._calculate_centering_angular_z(target_id) # Calculate centering
        # final_angular_z = angular_z + angular_z_center # Combine? Careful with magnitudes
        # Let's just send the alignment rotation for now
        self._send_velocity(target_id, 0.0, angular_z)


    def _handle_docking_state(self, time_in_state):
        """Moves the male (seeker) forward and the entire target group backward."""
        if self.current_male_id is None or self.current_female_id is None: return self._fail_state_missing_ids("DOCKING") # female_id is the target_id

        male_id = self.current_male_id     # This is the seeker
        target_group = self.current_target_group

        # --- Log Roll Angles (Optional) ---
        # ... (Add roll logging here if desired, iterating through male_id and target_group) ...

        # --- Command Velocities ---
        self.get_logger().info(f"Docking Male={male_id}, TargetGroup={target_group}... ({time_in_state:.1f}/{self.docking_duration_sec:.1f})", throttle_duration_sec=1.0)

        # Move Male Forward (with optional centering)
        male_angular_z = self._calculate_centering_angular_z(male_id) if self.kp_centering_forward != 0 else 0.0
        self._send_velocity(male_id, self.docking_speed_male, male_angular_z)

        # Move Entire Target Group Backward (with optional centering)
        if not target_group:
             self.get_logger().error("Docking state reached with empty target group!")
             self.change_state(RobotState.HANDLING_FAILURE)
             return
        for member_id in target_group:
            member_angular_z = self._calculate_centering_angular_z(member_id) if self.kp_centering_forward != 0 else 0.0
            self._send_velocity(member_id, self.docking_speed_female, member_angular_z) # All members move backward
        # ---

        if time_in_state >= self.docking_duration_sec:
            self.get_logger().info("Docking duration complete. Attempting lock.")
            self.change_state(RobotState.LOCKING)


    def _handle_locking_state(self, time_in_state):
        """Commands male joint lock WHILE pushing male forward and target group backward."""
        if self.current_male_id is None or self.current_female_id is None: return self._fail_state_missing_ids("LOCKING")

        male_id = self.current_male_id     # This is the seeker
        target_group = self.current_target_group

        # --- Apply Gentle Push with Centering ---
        self.get_logger().debug(f"Pushing during LOCKING: Male={male_id}, TargetGroup={target_group}", throttle_duration_sec=1.0)
        # Push Male
        male_angular_z = self._calculate_centering_angular_z(male_id) if self.kp_centering_forward != 0 else 0.0
        self._send_velocity(male_id, self.docking_speed_male, male_angular_z)

        # Push Target Group
        if not target_group:
             self.get_logger().error("Locking state reached with empty target group!")
             self.change_state(RobotState.HANDLING_FAILURE)
             return
        for member_id in target_group:
             member_angular_z = self._calculate_centering_angular_z(member_id) if self.kp_centering_forward != 0 else 0.0
             self._send_velocity(member_id, self.docking_speed_female, member_angular_z)
        # ---

        # Command lock once near the beginning
        if time_in_state < 0.2 and not self.agent_sensor_data['male_locked'].get(male_id, False):
             self.get_logger().info(f"Commanding male ({male_id}) joint to lock.")
             self._set_joint(male_id, 'male', self.male_lock_angle)

        # Check lock status
        if self.agent_sensor_data['male_locked'].get(male_id, False):
            self.get_logger().info(f"Male ({male_id}) lock confirmed. Stopping push.")
            # Stop pushing
            self._send_velocity(male_id, 0.0, 0.0)
            for member_id in target_group:
                self._send_velocity(member_id, 0.0, 0.0)
            self.change_state(RobotState.VERIFYING_CONNECTION)
            return

        # Timeout check
        if time_in_state > self.locking_timeout_sec:
            self.get_logger().error(f"Locking timed out for male agent {male_id}! Stopping push.")
            # Stop pushing on failure
            self._send_velocity(male_id, 0.0, 0.0)
            for member_id in target_group:
                self._send_velocity(member_id, 0.0, 0.0)
            self.change_state(RobotState.HANDLING_FAILURE)


    def _handle_verifying_connection_state(self, time_in_state):
        """Pulls the male back slightly. Keeps entire target group stationary."""
        if self.current_male_id is None or self.current_female_id is None: return self._fail_state_missing_ids("VERIFYING_CONNECTION")

        male_id = self.current_male_id     # This is the seeker
        female_id = self.current_female_id # The direct target ID
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
                self.get_logger().info(f"Verifying connection (Pair: {male_id},{female_id}). Locked range: {self.locked_connection_range:.3f}m.")
            elif time_in_state > 1.0: # Wait 1s for valid range
                self.get_logger().error(f"Verification failed - No valid range from male {male_id} at start.")
                self.change_state(RobotState.HANDLING_FAILURE)
                return # Wait longer if necessary
            else: return # Wait for valid range

        # --- Pull male back to test lock (with optional centering) ---
        male_angular_z = self._calculate_centering_angular_z(male_id) if self.kp_centering_forward != 0 else 0.0
        self._send_velocity(male_id, self.verification_pull_speed, male_angular_z)
        # ---

        # Check after duration
        if time_in_state >= self.verification_duration_sec:
            self._send_velocity(male_id, 0.0, 0.0) # Stop male pull

            current_range_male_after_pull = self.agent_sensor_data['range'].get(male_id)

            if self.locked_connection_range is None or current_range_male_after_pull is None or current_range_male_after_pull < self.range_valid_min_m:
                self.get_logger().error(f"Verify check fail - Invalid range male={current_range_male_after_pull}, stored={self.locked_connection_range}.")
                self.change_state(RobotState.HANDLING_FAILURE)
                return

            if self.locked_connection_range > 0.01: # Avoid division by zero
                difference = abs(current_range_male_after_pull - self.locked_connection_range) / self.locked_connection_range
                self.get_logger().info(f"Verify check: Male range {current_range_male_after_pull:.3f}, Locked range {self.locked_connection_range:.3f}. Diff {difference:.2%}")
                if difference < self.connection_verification_threshold:
                    self.get_logger().info(f"Connection verified for pair ({male_id}, {female_id})!") # Log refers to the pair being formed
                    self.change_state(RobotState.CONNECTED)
                else:
                    self.get_logger().error(f"Verification failed for pair ({male_id}, {female_id})! Range difference {difference:.2%} too large.")
                    self.change_state(RobotState.HANDLING_FAILURE)
            else:
                 self.get_logger().error("Verify check fail - Locked range was too small to verify.")
                 self.change_state(RobotState.HANDLING_FAILURE)


    def _handle_connected_state(self):
        """Marks seeker/target pair as connected, updates statuses, stops involved group."""
        if self.current_seeker_id is None or self.current_target_id is None:
             self.get_logger().error("Entered CONNECTED state without valid seeker/target pair IDs! Returning to MOVING.")
             self.current_target_group = set() # Ensure group is clear on error
             # Reset statuses? Maybe not needed if returning to MOVING
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
        # ---

        # --- Update status and store pair ---
        self.get_logger().info(f"Updating status for {seeker_id} and {target_id} to CONNECTED.")
        self.robot_status[seeker_id] = AgentStatus.CONNECTED
        self.robot_status[target_id] = AgentStatus.CONNECTED
        # Update status for other group members (should already be CONNECTED, but ensure)
        for member_id in target_group:
             if member_id != target_id and self.robot_status.get(member_id) != AgentStatus.CONNECTED:
                 self.get_logger().warn(f"Connected State: Group member {member_id} was not CONNECTED. Setting.")
                 self.robot_status[member_id] = AgentStatus.CONNECTED

        # Store the newly formed connection link
        self.connected_pairs.add(tuple(sorted((seeker_id, target_id))))
        self.get_logger().info(f"Updated Robot Status: {self.robot_status}")
        self.get_logger().info(f"Updated Connected Pairs: {self.connected_pairs}")
        # ---

        # Transition back to MOVING (temporary vars cleared by change_state)
        self.change_state(RobotState.MOVING_ALL_FORWARD)


    def _handle_failure_state(self):
        """Unified state for handling failures. Stops seeker and target group. Unlocks male."""
        self.get_logger().error("Handling connection sequence failure.")
        seeker_id = self.current_seeker_id
        target_id = self.current_target_id # Direct target involved in failure
        target_group = self.current_target_group
        male_id = self.current_male_id # Seeker is assumed male

        # Stop involved robots
        self.get_logger().debug(f"Failure: Stopping Seeker={seeker_id}, TargetGroup={target_group}")
        if seeker_id is not None: self._send_velocity(seeker_id, 0.0, 0.0)
        for member_id in target_group:
            self._send_velocity(member_id, 0.0, 0.0)
        # ---

        # Attempt to unlock male (seeker's) joint
        if male_id is not None:
             self.get_logger().info(f"Attempting to unlock male joint ({male_id}).")
             self._set_joint(male_id, 'male', self.male_unlock_angle)
             time.sleep(0.5) # Give command time

        # Set status for involved robots for reset
        # Only mark seeker and direct target as RESETTING
        # Other group members remain CONNECTED (unless connection physically broke)
        if seeker_id is not None: self.robot_status[seeker_id] = AgentStatus.RESETTING
        if target_id is not None: self.robot_status[target_id] = AgentStatus.RESETTING
        # ---

        self.change_state(RobotState.RESETTING)


    def _handle_resetting_state(self, time_in_state):
            """Moves seeker and direct target apart. Keeps rest of target group still. Retries connection."""
            local_seeker_id = self.current_seeker_id
            local_target_id = self.current_target_id
            target_group = self.current_target_group # Group involved in the failed attempt
            other_group_members = target_group - {local_target_id} if target_group else set()

            if local_seeker_id is None or local_target_id is None:
                # ... (recovery logic - might need update if group info is lost) ...
                self.get_logger().warn("Entered RESETTING without seeker/target IDs! Attempting recovery.")
                resetting_agents = [aid for aid, status in self.robot_status.items() if status == AgentStatus.RESETTING]
                if len(resetting_agents) >= 2: # Allow recovery even if group info lost
                    self.get_logger().info(f"Recovered resetting pair (arbitrary roles): {resetting_agents[0]}, {resetting_agents[1]}")
                    local_seeker_id = resetting_agents[0]
                    local_target_id = resetting_agents[1]
                    self.current_seeker_id = local_seeker_id
                    self.current_target_id = local_target_id
                    self.current_target_group = set() # Cannot recover group reliably here
                    other_group_members = set()
                else:
                    self.get_logger().error("Could not recover resetting pair. Returning to MOVING.")
                    for aid in resetting_agents: self.robot_status[aid] = AgentStatus.ACTIVE
                    # Clear all relevant temp vars before moving
                    self.current_seeker_id = None; self.current_target_id = None; self.current_male_id = None; self.current_female_id = None; self.locked_connection_range = None; self.current_target_group = set()
                    self.change_state(RobotState.MOVING_ALL_FORWARD)
                    return

            # --- Ensure Other Group Members stay stopped during reset ---
            if other_group_members:
                self.get_logger().debug(f"Reset: Stopping other group members {other_group_members}", throttle_duration_sec=2.0)
                for member_id in other_group_members:
                    self._send_velocity(member_id, 0.0, 0.0)
            # ---

            # Use seeker's range to gauge distance
            current_range_seeker = self.agent_sensor_data['range'].get(local_seeker_id)
            valid_range = current_range_seeker if (current_range_seeker is not None and current_range_seeker >= self.range_valid_min_m) else self.reset_distance_m

            self.get_logger().info(f"Resetting pair ({local_seeker_id}, {local_target_id}): Range {valid_range:.3f} / Target {self.reset_distance_m:.3f}", throttle_duration_sec=1.0)

            if valid_range < self.reset_distance_m:
                # Only move seeker and direct target
                seeker_angular_z = self._calculate_centering_angular_z(local_seeker_id) if self.kp_centering_forward != 0 else 0.0
                target_angular_z = self._calculate_centering_angular_z(local_target_id) if self.kp_centering_forward != 0 else 0.0
                self._send_velocity(local_seeker_id, self.reset_speed_seeker, seeker_angular_z) # Seeker reverse
                self._send_velocity(local_target_id, self.reset_speed_target, target_angular_z) # Target forward (away)
            else:
                self.get_logger().info(f"Reset distance reached for pair ({local_seeker_id}, {local_target_id}). Attempting connection again.")
                self._send_velocity(local_seeker_id, 0.0, 0.0)
                self._send_velocity(local_target_id, 0.0, 0.0)
                # Other group members already stopped

                # Mark involved robots as active, ready for next state
                self.robot_status[local_seeker_id] = AgentStatus.ACTIVE
                self.robot_status[local_target_id] = AgentStatus.ACTIVE
                # Other group members remain CONNECTED

                # Reset temporary connection vars (male/female/lock range)
                # Keep seeker/target IDs for the retry. Group will be recalculated in confirmation.
                self.current_male_id = None
                self.current_female_id = None
                self.locked_connection_range = None

                # Transition directly back to INITIATE_CONNECTION for this pair
                self.change_state(RobotState.INITIATE_CONNECTION) # Group will be recalculated


    def _handle_all_connected_or_halted_state(self):
        """Final state. Logs status, last velocities, and ensures robots are stopped."""
        self.get_logger().info("All robots are connected or have halted.", once=True)
        self.get_logger().info(f"Final Robot Status: {self.robot_status}", once=True)
        self.get_logger().info(f"Connected Pairs: {self.connected_pairs}", once=True)

        vel_log_msgs = []
        for agent_id in sorted(list(self.discovered_agent_ids)):
            last_lin, last_ang = self.agent_last_cmd_vel.get(agent_id, (None, None))
            lin_str = f"{last_lin:.3f}" if last_lin is not None else "N/A"
            ang_str = f"{last_ang:.3f}" if last_ang is not None else "N/A"
            vel_log_msgs.append(f"  Agent {agent_id}: Last CmdVel (L={lin_str}, A={ang_str})")

        if vel_log_msgs:
            log_string = "Last commanded velocities before final halt:\n" + "\n".join(vel_log_msgs)
            self.get_logger().info(log_string, once=True)

        self.stop_all_robots()
        self.get_logger().debug("Staying in ALL_CONNECTED_OR_HALTED state.", throttle_duration_sec=10.0)


    def _handle_failed_state(self):
        """Generic failure state. Logs error and stops all robots."""
        self.get_logger().error("Connection process entered FAILED state.", once=True)
        self.stop_all_robots()
        # Stay in this state

    def _fail_state_missing_ids(self, state_name):
        """Helper for states requiring seeker/target/male/female IDs"""
        self.get_logger().error(f"Entered {state_name} without required seeker/target IDs!")
        self.change_state(RobotState.FAILED)


    # ==========================================================================
    # Helper Functions
    # ==========================================================================

    def _find_connected_group(self, start_agent_id: int) -> set:
        """
        Finds all agent IDs connected to the start_agent_id, including itself,
        by traversing the self.connected_pairs graph.
        """
        if start_agent_id not in self.robot_status:
            self.get_logger().warn(f"_find_connected_group called with unknown agent {start_agent_id}")
            return {start_agent_id}

        group = set()
        queue = {start_agent_id}

        while queue:
            current_id = queue.pop()
            if current_id not in group:
                group.add(current_id)
                for id1, id2 in self.connected_pairs:
                    neighbor = None
                    if id1 == current_id:
                        neighbor = id2
                    elif id2 == current_id:
                        neighbor = id1

                    if neighbor is not None and neighbor not in group:
                        queue.add(neighbor)

        self.get_logger().debug(f"Found connected group for {start_agent_id}: {group}")
        return group

    def stop_all_robots(self):
        """Sends zero velocity to all discovered robots."""
        self.get_logger().debug("--- Stopping all discovered robots ---")
        for agent_id in self.discovered_agent_ids:
            self._send_velocity(agent_id, 0.0, 0.0)

    def _send_velocity(self, agent_id, linear_x, angular_z):
        """Sends velocity command to the specified agent, avoids duplicates except for zero."""
        pub = self.agent_pubs['cmd_vel'].get(agent_id)
        if not pub:
            self.get_logger().warn(f"_send_velocity: No cmd_vel pub for {agent_id}", throttle_duration_sec=10.0)
            return

        last_lin, last_ang = self.agent_last_cmd_vel.get(agent_id, (None, None))

        is_zero_command = abs(linear_x) < 0.001 and abs(angular_z) < 0.001
        is_same_as_last = (last_lin is not None and last_ang is not None and
                           abs(linear_x - last_lin) < 0.001 and
                           abs(angular_z - last_ang) < 0.001)

        if not is_same_as_last or is_zero_command:
            cmd = Twist()
            cmd.linear.x = float(linear_x)
            cmd.angular.z = float(angular_z)

            # Format the numbers first, handling None
            last_lin_str = f"{last_lin:.3f}" if last_lin is not None else "N/A"
            last_ang_str = f"{last_ang:.3f}" if last_ang is not None else "N/A"

            if not is_same_as_last:
                 # Use the pre-formatted strings in the f-string
                 self.get_logger().debug(f"SendVel {agent_id}: New L={linear_x:.3f} A={angular_z:.3f} (Last L={last_lin_str} A={last_ang_str})", throttle_duration_sec=2.0)
            elif is_zero_command and not (abs(last_lin or 0.0) < 0.001 and abs(last_ang or 0.0) < 0.001):
                 # Use the pre-formatted strings here too
                 self.get_logger().debug(f"SendVel {agent_id}: Sending Zero L={linear_x:.3f} A={angular_z:.3f} (Last L={last_lin_str} A={last_ang_str})", throttle_duration_sec=2.0) 

            pub.publish(cmd)
            self.agent_last_cmd_vel[agent_id] = (linear_x, angular_z) # Update cache


    def _calculate_centering_angular_z(self, agent_id):
        """Calculates the desired angular_z for centering based on roll."""
        if agent_id not in self.agent_sensor_data['roll']: return 0.0

        roll_rad = self.agent_sensor_data['roll'].get(agent_id, 0.0)
        roll_deg = math.degrees(roll_rad)

        current_linear, _ = self.agent_last_cmd_vel.get(agent_id, (0.0, 0.0))
        is_reversing = current_linear < -0.001

        angular_z_cmd = 0.0
        if abs(roll_deg) > self.roll_threshold_deg:
            effective_k_p = -self.kp_centering_forward if is_reversing else self.kp_centering_forward
            angular_z_cmd = effective_k_p * roll_deg # Using degrees
            max_correction_vel = 0.3
            angular_z_cmd = max(-max_correction_vel, min(max_correction_vel, angular_z_cmd))
        return angular_z_cmd

    # Use this function where centering was previously applied directly
    def _centre_robot(self, agent_id):
        """Applies centering correction by calculating and sending velocity."""
        # This function now calculates and sends the command directly, incorporating linear speed.
        # Consider removing direct calls to this if using the calculate+combine approach in state handlers.
        # For simplicity in this refactor, let's keep it calculating+sending.

        angular_z_cmd = self._calculate_centering_angular_z(agent_id)

        # Get current linear speed from cache to maintain it
        current_linear, last_angular = self.agent_last_cmd_vel.get(agent_id, (0.0, 0.0))

        # Log calculation (optional)
        # self.get_logger().info(
        #     f"Center Calc {agent_id}: Roll={math.degrees(self.agent_sensor_data['roll'].get(agent_id, 0.0)):.2f} => AngZ={angular_z_cmd:.3f}",
        #     throttle_duration_sec=1.0
        # )

        # Send velocity command only if angular component changes significantly
        if abs(angular_z_cmd - last_angular) > 0.01:
             self._send_velocity(agent_id, current_linear, angular_z_cmd)
        elif abs(angular_z_cmd) < 0.01 and abs(last_angular) > 0.01:
             self._send_velocity(agent_id, current_linear, 0.0)


    def _set_joint(self, agent_id, joint_type, position):
        """Commands 'male' or 'female' joint for the specified agent."""
        dict_key = f"{joint_type}_joint"
        pub = self.agent_pubs.get(dict_key, {}).get(agent_id)

        if not pub:
             self.get_logger().warn(f"_set_joint: No {dict_key} pub for {agent_id}", throttle_duration_sec=10.0)
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

        if joint_type == 'male':
             self.agent_sensor_data['male_locked'][agent_id] = False
             self.agent_sensor_data['male_unlocked'][agent_id] = False

    # ==========================================================================
    # Subscriber Callbacks
    # ==========================================================================
    def _calculate_imu_rp(self, msg: Imu):
        """Helper to calculate roll/pitch from IMU accelerometer data."""
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        accel_mag_sq = ax**2 + ay**2 + az**2
        if accel_mag_sq < 0.01: return None, None
        try:
            # Ensure arguments to sqrt are non-negative
            ay_sq_plus_az_sq = ay**2 + az**2
            ax_sq_plus_az_sq = ax**2 + az**2 # Use az for pitch? Should be ay, ax for pitch? Revisit formula if needed
            # Let's assume standard roll/pitch from accel:
            # Roll: atan2(ay, az)
            # Pitch: atan2(-ax, sqrt(ay^2 + az^2))

            # Using common formula:
            roll = math.atan2(ay, az)
            pitch = math.atan2(-ax, math.sqrt(ay_sq_plus_az_sq)) if ay_sq_plus_az_sq >= 0 else 0.0
            return roll, pitch
        except ValueError as e:
            self.get_logger().warn(f"Math error in IMU calculation: {e}", once=True)
            return None, None

    def imu_callback(self, msg: Imu, agent_id: int):
        """Stores roll/pitch for the specific agent."""
        # self.get_logger().debug(f"IMU CB Agent {agent_id} Received: {msg}", throttle_duration_sec=1.0) # Optional Debug
        roll, pitch = self._calculate_imu_rp(msg)
        if roll is not None: self.agent_sensor_data['roll'][agent_id] = roll
        if pitch is not None: self.agent_sensor_data['pitch'][agent_id] = pitch

    def range_callback(self, msg: Range, agent_id: int):
        """Stores valid range data for the specific agent."""
        # self.get_logger().debug(f"Range CB Agent {agent_id} Received: {msg}", throttle_duration_sec=1.0) # Optional Debug
        valid_range = None
        if math.isfinite(msg.range) and msg.range >= msg.min_range and msg.range <= msg.max_range:
             valid_range = msg.range
        # Only update if changed to avoid constant dict writes
        if self.agent_sensor_data['range'].get(agent_id) != valid_range:
            self.agent_sensor_data['range'][agent_id] = valid_range

    def alignment_callback(self, msg: Bool, agent_id: int):
        """Stores alignment status for the specific agent."""
        # self.get_logger().debug(f"Align CB Agent {agent_id} Received: {msg}", throttle_duration_sec=1.0) # Optional Debug
        if self.agent_sensor_data['aligned'].get(agent_id) != msg.data:
             self.agent_sensor_data['aligned'][agent_id] = msg.data

    def male_joint_state_callback(self, msg: JointTrajectoryControllerState, agent_id: int):
        """Stores male lock/unlock status based on reported joint position."""
        # self.get_logger().debug(f"JointState CB Agent {agent_id} Received: {msg}", throttle_duration_sec=1.0) # Optional Debug
        pos_list = None
        # Prioritize actual position if available
        if hasattr(msg, 'actual') and msg.actual and hasattr(msg.actual, 'positions') and msg.actual.positions:
            pos_list = msg.actual.positions
        # Fallback to feedback if actual not present or empty
        elif hasattr(msg, 'feedback') and msg.feedback and hasattr(msg.feedback, 'positions') and msg.feedback.positions:
            pos_list = msg.feedback.positions
        # Add other fallbacks if needed (e.g., desired?)

        if pos_list is None or len(pos_list) == 0: return

        current_pos = pos_list[0]
        is_unlocked = abs(current_pos - self.male_unlock_angle) < self.joint_pos_tolerance
        is_locked = abs(current_pos - self.male_lock_angle) < self.joint_pos_tolerance

        # Only update if changed
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