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
from sensor_msgs.msg import Range
from enum import Enum, auto
import time
from functools import partial

# Define the states using Enum for clarity 
class RobotState(Enum):
    DISCOVERING_ROBOTS = auto()
    IDLE = auto()
    INITIALIZING_ROBOTS = auto()
    MOVING_FORWARD = auto()
    DETECTING_OBSTACLE = auto()
    CONFIRMING_ROBOT = auto()
    OBSTACLE_DETECTED_HALTED = auto()
    APPROACHING_TARGET = auto()
    CHECKING_ALIGNMENT = auto()
    ALIGNING = auto()
    DOCKING = auto()
    LOCKING = auto()
    VERIFYING_CONNECTION = auto()
    CONNECTED = auto()
    HANDLING_ALIGNMENT_FAILURE = auto()
    RESETTING = auto()
    FAILED = auto()

class Connect_Robots(Node):

    def __init__(self):
        super().__init__("connect_robots_fsm_node")

        # --- Discovery and Robot Storage ---
        self.robots = {} # Stores info about discovered robots (not heavily used yet)
        self.discovered_agent_ids = set()
        self.num_discovered_robots = 0
        self.discovery_complete = False # Flag to indicate discovery finished
        self.discovery_timer_period = 2.0 # Check every 2 seconds
        self.discovery_attempts = 0
        self.max_discovery_attempts = 5 # Give up after 5 stable attempts

        # --- Dynamic Pubs/Subs Storage ---
        self.agent_pubs = {'cmd_vel': {}, 'male_joint': {}, 'female_joint': {}}
        self.agent_subs = {'imu': {}, 'range': {}, 'alignment': {}, 'male_joint_state': {}}
        # Store latest sensor data keyed by agent_id
        self.agent_sensor_data = {'roll': {}, 'pitch': {}, 'range': {}, 'aligned': {}, 'male_locked': {}, 'male_unlocked': {}}
        self.agent_last_cmd_vel = {} # Cache last sent cmd_vel {agent_id: (linear, angular)}

        # --- State Machine Variables ---
        # Start in DISCOVERING state
        self.current_state = RobotState.DISCOVERING_ROBOTS
        self.state_enter_time = self.get_clock().now()
        self.get_logger().info(f"Node starting in state: {self.current_state.name}")

        # --- Constants and Configuration (Same as before) ---
        self.initial_delay_sec = 2.0
        self.robot_detection_threshold = 0.1
        self.connection_verification_threshold = 0.2
        self.close_distance_m = 0.15
        self.reset_distance_m = 0.3
        self.male_lock_angle = math.pi / 2.0
        self.male_unlock_angle = 0.0
        self.female_lift_angle = 0.0
        self.alignment_timeout_sec = 40.0
        self.docking_duration_sec = 10.0
        self.locking_timeout_sec = 5.0
        self.verification_duration_sec = 5.0
        self.confirmation_timeout_sec = 3.0
        self.kp_centering_forward = -0.05 

        # FSM Temp variables (Keep these as they relate to the FSM instance)
        self.initial_scan_range = None
        self.robot_confirm_scan_range = None
        self.locked_connection_range = None

        # --- REMOVED Hardcoded Publishers/Subscribers ---
        # Publishers/Subscribers are now created dynamically in _register_robot

        # --- Timers ---
        self.discovery_timer = self.create_timer(self.discovery_timer_period, self.discover_robots)
        self.fsm_loop_timer = self.create_timer(0.1, self.state_machine_tick) # Main FSM loop

        self.get_logger().info("Connect_Robots FSM node initialized, starting discovery.")

    # ==========================================================================
    # Robot Discovery (NEW)
    # ==========================================================================
    def discover_robots(self):
        """Periodically scans topics to find agent_N robots."""
        if self.discovery_complete:
            self.discovery_timer.cancel() # Stop discovery once done
            return

        self.get_logger().info(f"Discovery attempt {self.discovery_attempts + 1}...")
        found_new = False
        previous_ids = set(self.discovered_agent_ids) # Copy previous set

        try:
            topic_list = self.get_topic_names_and_types()
            # Look for a topic likely published by every agent
            agent_topic_pattern = re.compile(r'/agent_(\d+)/robot_description') # Or /imu/out

            current_ids = set()
            for topic_name, _ in topic_list:
                match = agent_topic_pattern.match(topic_name)
                if match:
                    agent_id = int(match.group(1))
                    current_ids.add(agent_id)
                    if agent_id not in self.discovered_agent_ids:
                        # Found a new robot
                        self.get_logger().info(f"Discovered agent_{agent_id}")
                        self._register_robot(agent_id)
                        self.discovered_agent_ids.add(agent_id) # Add to the main set
                        found_new = True

            # Update the main set with all currently found IDs
            # (Handles case where a robot might disappear, though not fully robustly)
            self.discovered_agent_ids = current_ids
            self.num_discovered_robots = len(self.discovered_agent_ids)

        except Exception as e:
            self.get_logger().error(f"Error during robot discovery: {e}")

        # Check for stability
        if not found_new and self.num_discovered_robots > 0:
             # If no new robots were found this time, increment stable count
             self.discovery_attempts += 1
             self.get_logger().info(f"Discovery stable ({self.discovery_attempts}/{self.max_discovery_attempts}). Found {self.num_discovered_robots} robots.")
             if self.discovery_attempts >= self.max_discovery_attempts:
                  self.get_logger().info(f"Discovery complete. Final count: {self.num_discovered_robots}. Robots: {sorted(list(self.discovered_agent_ids))}")
                  self.discovery_complete = True
                  # Transition the FSM state only if it was still discovering
                  if self.current_state == RobotState.DISCOVERING_ROBOTS:
                       self.change_state(RobotState.IDLE) # Move to original starting state
                  self.discovery_timer.cancel()
        elif found_new:
             # If we found a new robot, reset stability counter
             self.discovery_attempts = 0
             self.get_logger().info(f"Found new robot(s). Current count: {self.num_discovered_robots}. Resetting stability counter.")
        else: # No robots found yet
             self.discovery_attempts += 1
             if self.discovery_attempts >= self.max_discovery_attempts:
                 self.get_logger().error(f"Discovery timed out. No robots found.")
                 self.discovery_complete = True # Mark as complete even if failed
                 if self.current_state == RobotState.DISCOVERING_ROBOTS:
                      self.change_state(RobotState.FAILED) # Cannot proceed
                 self.discovery_timer.cancel()


    def _register_robot(self, agent_id):
        """(NEW) Create pubs/subs and initialize data storage for a robot."""
        if agent_id in self.robots: # Avoid re-registering (shouldn't happen with set logic)
            return

        ns = f"agent_{agent_id}"
        self.robots[agent_id] = {'ns': ns} # Store basic info
        self.get_logger().info(f"Registering resources for {ns}")

        # Initialize sensor data storage
        self.agent_sensor_data['roll'][agent_id] = 0.0
        self.agent_sensor_data['pitch'][agent_id] = 0.0
        self.agent_sensor_data['range'][agent_id] = None
        self.agent_sensor_data['aligned'][agent_id] = False
        self.agent_sensor_data['male_locked'][agent_id] = False
        self.agent_sensor_data['male_unlocked'][agent_id] = True
        self.agent_last_cmd_vel[agent_id] = (0.0, 0.0)

        # Create Publishers
        self.agent_pubs['cmd_vel'][agent_id] = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
        self.agent_pubs['male_joint'][agent_id] = self.create_publisher(JointTrajectory, f"/{ns}/male_joint_trajectory_controller/joint_trajectory", 10)
        self.agent_pubs['female_joint'][agent_id] = self.create_publisher(JointTrajectory, f"/{ns}/female_joint_trajectory_controller/joint_trajectory", 10)

        # Create Subscribers - Use partial to pass agent_id to the callback
        self.agent_subs['imu'][agent_id] = self.create_subscription(
            Imu, f"/{ns}/imu/out", partial(self.imu_callback, agent_id=agent_id), 10)
        # Assuming range sensor and alignment sensor are on the robot indexed by agent_id
        self.agent_subs['range'][agent_id] = self.create_subscription(
            Range, f"/{ns}/infrared_range", partial(self.range_callback, agent_id=agent_id), 10)
        self.agent_subs['alignment'][agent_id] = self.create_subscription(
            Bool, f"/{ns}/alignment", partial(self.alignment_callback, agent_id=agent_id), 10)
        self.agent_subs['male_joint_state'][agent_id] = self.create_subscription(
            JointTrajectoryControllerState, f"/{ns}/male_joint_trajectory_controller/controller_state", partial(self.male_joint_state_callback, agent_id=agent_id), 10)

    # ==========================================================================
    # State Machine Core Logic (MODIFIED)
    # ==========================================================================

    def state_machine_tick(self):
        """Main loop executed at regular intervals."""
        # --- MODIFIED: Wait for discovery to complete ---
        if not self.discovery_complete:
            # self.get_logger().info("Waiting for robot discovery to complete...", throttle_duration_sec=5.0)
            return

        # --- MODIFIED: Ensure needed robots (0 and 1) were discovered ---
        # Only run FSM if agents 0 and 1 exist, otherwise fail state?
        if 0 not in self.discovered_agent_ids or 1 not in self.discovered_agent_ids:
             if self.current_state not in [RobotState.FAILED, RobotState.DISCOVERING_ROBOTS]: # Avoid spamming error
                self.get_logger().error(f"Required agents 0 or 1 not discovered (Found: {self.discovered_agent_ids}). Cannot run FSM.")
                self.change_state(RobotState.FAILED)
             return

        # --- MODIFIED: Get sensor data for agents 0 and 1 ---
        # Use .get() with defaults for safety, though we checked they exist above
        current_range_agent0 = self.agent_sensor_data['range'].get(0, None)
        # Check for valid range before proceeding with logic that needs it
        if current_range_agent0 is None and self.current_state not in [RobotState.IDLE, RobotState.DISCOVERING_ROBOTS]:
            self.get_logger().info("Waiting for agent 0's range sensor...", throttle_duration_sec=5.0)
            # Stop robots if range is lost during critical phases?
            if self.current_state not in [RobotState.CONNECTED, RobotState.FAILED, RobotState.OBSTACLE_DETECTED_HALTED]:
                 self._send_velocity(0, 0.0, 0.0)
                 self._send_velocity(1, 0.0, 0.0)
            return

        now = self.get_clock().now()
        time_in_state = (now - self.state_enter_time).nanoseconds / 1e9

        # --- State Execution (Logic mostly unchanged, but uses new helpers/data access) ---
        if self.current_state == RobotState.DISCOVERING_ROBOTS:
             # Should not happen if discovery_complete check passes, but safe fallback
             return
        elif self.current_state == RobotState.IDLE:
            self._handle_idle_state(time_in_state)
        elif self.current_state == RobotState.INITIALIZING_ROBOTS:
            self._handle_initializing_robots_state()
        elif self.current_state == RobotState.MOVING_FORWARD:
            # Pass agent 0's range to the handler
            self._handle_moving_forward_state(current_range_agent0)
        elif self.current_state == RobotState.DETECTING_OBSTACLE:
            # Pass agent 0's range to the handler
            self._handle_detecting_obstacle_state(current_range_agent0)
        elif self.current_state == RobotState.CONFIRMING_ROBOT:
            # Pass agent 0's range to the handler
            self._handle_confirming_robot_state(time_in_state, current_range_agent0)
        elif self.current_state == RobotState.OBSTACLE_DETECTED_HALTED:
            self._handle_obstacle_detected_halted_state()
        elif self.current_state == RobotState.APPROACHING_TARGET:
            # Pass agent 0's range to the handler
            self._handle_approaching_target_state(current_range_agent0)
        elif self.current_state == RobotState.CHECKING_ALIGNMENT:
            self._handle_checking_alignment_state()
        elif self.current_state == RobotState.ALIGNING:
            self._handle_aligning_state(time_in_state)
        elif self.current_state == RobotState.DOCKING:
            self._handle_docking_state(time_in_state)
        elif self.current_state == RobotState.LOCKING:
            self._handle_locking_state(time_in_state)
        elif self.current_state == RobotState.VERIFYING_CONNECTION:
            # Pass agent 0's range to the handler
            self._handle_verifying_connection_state(time_in_state, current_range_agent0)
        elif self.current_state == RobotState.CONNECTED:
            self._handle_connected_state()
        elif self.current_state == RobotState.HANDLING_ALIGNMENT_FAILURE:
            self._handle_alignment_failure_state()
        elif self.current_state == RobotState.RESETTING:
            # Pass agent 0's range to the handler
            self._handle_resetting_state(current_range_agent0)
        elif self.current_state == RobotState.FAILED:
            self._handle_failed_state()

    def change_state(self, new_state: RobotState):
        """(Mostly Same) Helper function to transition to a new state."""
        # Don't allow state change if still discovering
        if self.current_state == RobotState.DISCOVERING_ROBOTS and new_state != RobotState.IDLE and new_state != RobotState.FAILED:
            self.get_logger().warn(f"Attempted state change to {new_state.name} while still discovering.")
            return

        if new_state != self.current_state:
            self.get_logger().info(f"Changing state from {self.current_state.name} to {new_state.name}")
            self.current_state = new_state
            self.state_enter_time = self.get_clock().now()
            # Reset state-specific variables if necessary
            if new_state != RobotState.CONFIRMING_ROBOT: self.robot_confirm_scan_range = None
            if new_state != RobotState.VERIFYING_CONNECTION: self.locked_connection_range = None

    # ==========================================================================
    # State Handling Functions (MODIFIED for data access and helpers)
    # ==========================================================================

    def _handle_idle_state(self, time_in_state):
        """Waits for the initial delay (after discovery)."""
        if time_in_state >= self.initial_delay_sec:
            self.get_logger().info("Initial delay complete.")
            self.change_state(RobotState.INITIALIZING_ROBOTS)

    def _handle_initializing_robots_state(self):
        """Set initial joint positions for agents 0 and 1."""
        self.get_logger().info("Initializing: Unlocking agent 0 male, setting agent 1 female.")
        self._set_joint(0, 'male', self.male_unlock_angle)  # Agent 0 is male
        self._set_joint(1, 'female', self.female_lift_angle) # Agent 1 is female
        time.sleep(0.1) # Short delay
        self.change_state(RobotState.MOVING_FORWARD)

    # --- Pass current_range_agent0 into handlers that need agent 0's range ---
    def _handle_moving_forward_state(self, current_range_agent0):
        """Move agent_0 forward and check for obstacles using its range."""
        self._send_velocity(0, 0.1, 0.0) # Move agent 0 forward
        self._centre_robot(0) # Center agent 0

        if self.initial_scan_range is None:
            if current_range_agent0 is not None and current_range_agent0 > 0.05:
                self.initial_scan_range = current_range_agent0
                self.get_logger().info(f"Stored initial scan range (Agent 0): {self.initial_scan_range:.3f} m")
            return # Wait for next tick

        # Check range change (using agent 0's range)
        if self.initial_scan_range > 0.05:
             # Ensure current range is valid
             valid_range = current_range_agent0 if current_range_agent0 is not None else self.initial_scan_range
             if valid_range < 0.05: return

             difference = abs(valid_range - self.initial_scan_range) / self.initial_scan_range
             if difference > self.robot_detection_threshold:
                 self.get_logger().info(f"Agent 0 range change detected (diff: {difference:.2f}). Potential obstacle.")
                 self._send_velocity(0, 0.0, 0.0) # Stop agent 0
                 self.change_state(RobotState.DETECTING_OBSTACLE)

    def _handle_detecting_obstacle_state(self, current_range_agent0):
        """Store agent 0's range to the detected obstacle."""
        if current_range_agent0 is not None and current_range_agent0 > 0.05:
            self.robot_confirm_scan_range = current_range_agent0
            self.get_logger().info(f"Stored obstacle scan range (Agent 0): {self.robot_confirm_scan_range:.3f} m.")
            self.change_state(RobotState.CONFIRMING_ROBOT)
        else:
            self.get_logger().warn("Agent 0 range invalid while detecting obstacle. Reverting.")
            self.initial_scan_range = None
            self.change_state(RobotState.MOVING_FORWARD)

    def _handle_confirming_robot_state(self, time_in_state, current_range_agent0):
        """Move agent_1 slightly and check agent 0's range reading."""
        self._send_velocity(1, 0.1, 0.0) # Move agent 1 forward
        self._centre_robot(1) # Center agent 1

        if self.robot_confirm_scan_range is None: # Should be set
             self.get_logger().error("Error: robot_confirm_scan_range not set!")
             self.change_state(RobotState.FAILED)
             return

        # Ensure current range (from agent 0) is valid
        if current_range_agent0 is None or current_range_agent0 < 0.05:
            self.get_logger().warn("Agent 0 range lost during confirmation.", throttle_duration_sec=5.0)
            # Let timeout handle if range doesn't recover
        else:
            # Check if agent 0's range changes significantly
            if self.robot_confirm_scan_range > 0.05:
                difference = abs(current_range_agent0 - self.robot_confirm_scan_range) / self.robot_confirm_scan_range
                if difference > (1.5 * self.robot_detection_threshold):
                    self.get_logger().info(f"Agent 0 range changed (diff: {difference:.2f}). Agent 1 confirmed.")
                    self._send_velocity(1, 0.0, 0.0) # Stop agent 1
                    self.change_state(RobotState.APPROACHING_TARGET)
                    return

        # Timeout check
        if time_in_state > self.confirmation_timeout_sec:
            self.get_logger().warning("Robot confirmation timed out.")
            self._send_velocity(1, 0.0, 0.0) # Stop agent 1
            self.change_state(RobotState.OBSTACLE_DETECTED_HALTED)

    def _handle_obstacle_detected_halted_state(self):
        """Stop agents 0 and 1."""
        self._send_velocity(0, 0.0, 0.0)
        self._send_velocity(1, 0.0, 0.0)
        self.get_logger().info("Obstacle detected (not Agent 1). Halting.", once=True)

    def _handle_approaching_target_state(self, current_range_agent0):
        """Move agent_0 towards agent_1 using agent 0's range."""
        self._centre_robot(0) # Center agent 0

        # Ensure range is valid before using
        valid_range = current_range_agent0 if current_range_agent0 is not None else self.close_distance_m * 2

        if valid_range < self.close_distance_m:
            self.get_logger().info(f"Agent 0 reached close distance ({valid_range:.3f} m).")
            self._send_velocity(0, 0.0, 0.0)
            self.change_state(RobotState.CHECKING_ALIGNMENT)
        else:
            speed = max(0.02, 0.1 * (valid_range - self.close_distance_m))
            self._send_velocity(0, speed, 0.0)

    def _handle_checking_alignment_state(self):
        """Check agent 0's alignment status."""
        self.get_logger().info("Checking alignment status (Agent 0 sensor)...")
        # Use agent 0's alignment data
        agent0_aligned = self.agent_sensor_data['aligned'].get(0, False)
        if agent0_aligned:
            self.get_logger().info("Agent 0 aligned with target.")
            self.change_state(RobotState.DOCKING)
        else:
            self.get_logger().info("Agent 0 not aligned. Aligning.")
            self.change_state(RobotState.ALIGNING)

    def _handle_aligning_state(self, time_in_state):
        """Rotate agent_1 based on agent 0's alignment sensor."""
        self._send_velocity(0, 0.0, 0.0) # Agent 0 waits
        self._centre_robot(1) # Agent 1 centers

        # Check agent 0's alignment sensor
        agent0_aligned = self.agent_sensor_data['aligned'].get(0, False)
        if agent0_aligned:
            self.get_logger().info("Alignment successful.")
            self._send_velocity(1, 0.0, 0.0) # Stop agent 1 rotation
            self.change_state(RobotState.DOCKING)
            return

        # Timeout check
        if time_in_state > self.alignment_timeout_sec:
            self.get_logger().error("Alignment timed out!")
            self._send_velocity(1, 0.0, 0.0)
            self.change_state(RobotState.HANDLING_ALIGNMENT_FAILURE)
            return

        # Rotate agent 1
        time_in_cycle = time_in_state % 4.0; angular_z = 0.0
        if time_in_cycle <= 1.0: angular_z = 0.15
        elif time_in_cycle <= 3.0: angular_z = -0.15
        else: angular_z = 0.15
        self._send_velocity(1, 0.0, angular_z)

    def _handle_docking_state(self, time_in_state):
        """Move agent 0 forward and agent 1 backward."""
        self.get_logger().info(f"Docking... (time: {time_in_state:.1f})", throttle_duration_sec=1.0)
        self._send_velocity(0, 0.01, 0.0)
        self._send_velocity(1, -0.015, 0.0)
        self._centre_robot(0)
        self._centre_robot(1)

        if time_in_state >= self.docking_duration_sec:
            self.get_logger().info("Docking complete. Locking.")
            self._send_velocity(0, 0.0, 0.0)
            self._send_velocity(1, 0.0, 0.0)
            self.change_state(RobotState.LOCKING)

    def _handle_locking_state(self, time_in_state):
        """Command agent 0's male joint to lock."""
        if time_in_state < 0.2:
            self.get_logger().info("Commanding agent 0 male joint to lock.")
            self._set_joint(0, 'male', self.male_lock_angle)

        # Check agent 0's lock status
        agent0_locked = self.agent_sensor_data['male_locked'].get(0, False)
        if agent0_locked:
            self.get_logger().info("Agent 0 male lock confirmed.")
            self.change_state(RobotState.VERIFYING_CONNECTION)
            return

        if time_in_state > self.locking_timeout_sec:
            self.get_logger().error("Locking timed out!")
            self.change_state(RobotState.FAILED)

    def _handle_verifying_connection_state(self, time_in_state, current_range_agent0):
        """Pull agent_0 back, check its range sensor."""
        if self.locked_connection_range is None:
            if current_range_agent0 is not None and current_range_agent0 > 0.01:
                self.locked_connection_range = current_range_agent0
                self.get_logger().info(f"Verifying. Locked range (Agent 0): {self.locked_connection_range:.3f}m.")
            else:
                if time_in_state > 1.0: self.change_state(RobotState.FAILED); self.get_logger().error("Verify fail - No valid range.")
                return

        self._send_velocity(0, -0.05, 0.0) # Agent 0 reverse
        self._send_velocity(1, 0.0, 0.0)   # Agent 1 stop
        self._centre_robot(0)

        if time_in_state >= self.verification_duration_sec:
            self._send_velocity(0, 0.0, 0.0) # Stop agent 0

            if self.locked_connection_range is None or current_range_agent0 is None:
                self.get_logger().error("Verify check fail - Invalid range.")
                self.change_state(RobotState.FAILED)
                return

            diff = abs(current_range_agent0 - self.locked_connection_range) / self.locked_connection_range
            self.get_logger().info(f"Verify check: Diff {diff:.2f}")

            if diff < self.connection_verification_threshold:
                self.get_logger().info("Connection verified!")
                self.change_state(RobotState.CONNECTED)
            else:
                self.get_logger().error("Verification failed!")
                self._set_joint(0, 'male', self.male_unlock_angle) # Unlock agent 0
                self.change_state(RobotState.RESETTING)

    def _handle_connected_state(self):
        """Stop agents 0 and 1."""
        self._send_velocity(0, 0.0, 0.0)
        self._send_velocity(1, 0.0, 0.0)
        self.get_logger().info("Robots 0 and 1 successfully connected.", once=True)

    def _handle_alignment_failure_state(self):
        """Stop agents 0/1, unlock agent 0 male."""
        self.get_logger().error("Alignment failed. Resetting.")
        self._send_velocity(0, 0.0, 0.0)
        self._send_velocity(1, 0.0, 0.0)
        self._set_joint(0, 'male', self.male_unlock_angle)
        self.change_state(RobotState.RESETTING)

    def _handle_resetting_state(self, current_range_agent0):
        """Move agent 0 back, agent 1 forward, using agent 0's range."""
        valid_range = current_range_agent0 if current_range_agent0 is not None else self.reset_distance_m
        self.get_logger().info(f"Resetting: Range {valid_range:.3f} / Target {self.reset_distance_m:.3f}", throttle_duration_sec=1.0)

        if valid_range < self.reset_distance_m:
            self._send_velocity(0, -0.01, 0.0) # Agent 0 reverse
            self._send_velocity(1, 0.02, 0.0)  # Agent 1 forward
            self._centre_robot(0)
            self._centre_robot(1)
        else:
            self.get_logger().info("Reset complete. Restarting.")
            self._send_velocity(0, 0.0, 0.0)
            self._send_velocity(1, 0.0, 0.0)
            self.initial_scan_range = None
            # Reset alignment/lock status in stored data? Assume callbacks will update.
            self.change_state(RobotState.IDLE)

    def _handle_failed_state(self):
        """Stop agents 0 and 1."""
        self.get_logger().error("Connection process FAILED.", once=True)
        self._send_velocity(0, 0.0, 0.0)
        self._send_velocity(1, 0.0, 0.0)


    # ==========================================================================
    # Helper Functions (MODIFIED to use agent_id)
    # ==========================================================================

    def _send_velocity(self, agent_id, linear_x, angular_z):
        """Sends velocity command to the specified agent, avoids duplicates."""
        if agent_id not in self.agent_pubs['cmd_vel']:
            # self.get_logger().warn(f"_send_velocity: No cmd_vel publisher for agent {agent_id}") # Can be noisy
            return

        # Check cache
        last_lin, last_ang = self.agent_last_cmd_vel.get(agent_id, (None, None))
        if linear_x == last_lin and angular_z == last_ang:
            return

        # self.get_logger().info(f"--- Sending cmd_vel to agent_{agent_id}: Lin={linear_x:.3f}, Ang={angular_z:.3f}")
        pub = self.agent_pubs['cmd_vel'][agent_id]
        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        pub.publish(cmd)
        self.agent_last_cmd_vel[agent_id] = (linear_x, angular_z)


    def _centre_robot(self, agent_id):
        """Centering logic for the specified agent."""
        if agent_id not in self.agent_sensor_data['roll'] or agent_id not in self.agent_pubs['cmd_vel']: return

        roll = self.agent_sensor_data['roll'].get(agent_id, 0.0)
        current_linear, last_angular = self.agent_last_cmd_vel.get(agent_id, (0.0, 0.0))
        is_reversing = current_linear < -0.001

        angular_z_cmd = 0.0
        roll_threshold_rad = math.radians(0.5)

        if abs(roll) > roll_threshold_rad:
            effective_k_p = -self.kp_centering_forward if is_reversing else self.kp_centering_forward
            angular_z_cmd = effective_k_p * roll
            max_correction_vel = 0.2
            angular_z_cmd = max(-max_correction_vel, min(max_correction_vel, angular_z_cmd))

        # Only send if angular command changes significantly
        if abs(angular_z_cmd - last_angular) > 0.005:
             # self.get_logger().debug(f"--- Centering agent_{agent_id}: Roll={roll:.2f}, CmdAng={angular_z_cmd:.3f}")
             self._send_velocity(agent_id, current_linear, angular_z_cmd)
        elif abs(angular_z_cmd) < 0.005 and abs(last_angular) > 0.005:
             # self.get_logger().debug(f"--- Centering agent_{agent_id}: Stopping angular correction.")
             self._send_velocity(agent_id, current_linear, 0.0)

    def _set_joint(self, agent_id, joint_type, position):
        """Commands 'male' or 'female' joint for the specified agent."""
        # --- FIX: Construct the correct dictionary key ---
        dict_key = f"{joint_type}_joint" # e.g., 'male_joint' or 'female_joint'

        # Check if the constructed key exists and the agent_id exists within that sub-dictionary
        if dict_key not in self.agent_pubs or agent_id not in self.agent_pubs[dict_key]:
             self.get_logger().warn(f"_set_joint: No {dict_key} publisher registered for agent {agent_id}")
             return

        # --- FIX: Use the correct dictionary key for retrieval ---
        pub = self.agent_pubs[dict_key][agent_id]
        joint_name = f'base_{joint_type}_joint' # This part was correct
        # self.get_logger().info(f"--- Setting agent_{agent_id} {joint_name} to {position:.3f}")

        msg = JointTrajectory()
        msg.joint_names = [joint_name]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(sec=2, nanosec=0)
        msg.points.append(point)
        pub.publish(msg)

        # Reset lock state flags when commanding male joint
        # --- FIX: Check using the correct dict_key ---
        if joint_type == 'male' and dict_key in self.agent_pubs and agent_id in self.agent_sensor_data['male_locked']: # Check key exists
             self.agent_sensor_data['male_locked'][agent_id] = False
             self.agent_sensor_data['male_unlocked'][agent_id] = False

        # Reset lock state flags when commanding male joint
        if joint_type == 'male' and agent_id in self.agent_sensor_data['male_locked']: # Check key exists
             self.agent_sensor_data['male_locked'][agent_id] = False
             self.agent_sensor_data['male_unlocked'][agent_id] = False

    def stop_all_robots(self):
        """(NEW) Sends zero velocity to all discovered robots."""
        self.get_logger().info("--- Stopping all discovered robots ---")
        for agent_id in self.discovered_agent_ids:
            self._send_velocity(agent_id, 0.0, 0.0)

    # ==========================================================================
    # Subscriber Callbacks (MODIFIED to use agent_id)
    # ==========================================================================
    def _calculate_imu_rp(self, msg: Imu):
        ax=msg.linear_acceleration.x; ay=msg.linear_acceleration.y; az=msg.linear_acceleration.z
        m=ax**2+ay**2+az**2
        if m<0.01: return None,None
        try: r=math.atan2(-ax,math.sqrt(ay**2+az**2)); p=math.atan2(ay,math.sqrt(ax**2+az**2)); return r,p
        except ValueError: return None,None

    def imu_callback(self, msg: Imu, agent_id: int):
        """Stores roll/pitch for the specific agent."""
        # self.get_logger().debug(f"IMU received for agent {agent_id}")
        roll, pitch = self._calculate_imu_rp(msg)
        if roll is not None:
            self.agent_sensor_data['roll'][agent_id] = roll
            self.agent_sensor_data['pitch'][agent_id] = pitch

    def range_callback(self, msg: Range, agent_id: int):
        """Stores range for the specific agent."""
        # self.get_logger().debug(f"Range received for agent {agent_id}: {msg.range}")
        valid_range = None
        if math.isfinite(msg.range) and msg.range >= msg.min_range and msg.range <= msg.max_range:
            valid_range = msg.range
        # Store None if invalid
        self.agent_sensor_data['range'][agent_id] = valid_range

    def alignment_callback(self, msg: Bool, agent_id: int):
        """Stores alignment status for the specific agent."""
        # self.get_logger().debug(f"Alignment received for agent {agent_id}: {msg.data}")
        self.agent_sensor_data['aligned'][agent_id] = msg.data

    def male_joint_state_callback(self, msg: JointTrajectoryControllerState, agent_id: int):
        """Stores male lock status for the specific agent."""
        # self.get_logger().debug(f"Male joint state received for agent {agent_id}")
        if not msg.feedback or not msg.feedback.positions: return
        pos=msg.feedback.positions[0]; tol=0.1
        unlocked=abs(pos-self.male_unlock_angle)<tol; locked=abs(pos-self.male_lock_angle)<tol
        self.agent_sensor_data['male_unlocked'][agent_id]=unlocked
        self.agent_sensor_data['male_locked'][agent_id]=locked


def main(args=None):
    rclpy.init(args=args)
    node = Connect_Robots()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: node.get_logger().info("Ctrl-C detected, shutting down.")
    except Exception as e: node.get_logger().fatal(f"Node error: {e}")
    finally:
        if rclpy.ok(): # Check if context is still valid
             node.stop_all_robots() # Ensure all discovered robots are stopped
             node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()