#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Range
from enum import Enum, auto
import time # Using time for delays/timeouts is often clearer

# Define the states using Enum for clarity
class RobotState(Enum):
    IDLE = auto()                   # Initial state, waiting for delay
    INITIALIZING_ROBOTS = auto()  # Setting initial joint positions, starting forward motion
    MOVING_FORWARD = auto()         # Moving agent_0 forward, looking for obstacles
    DETECTING_OBSTACLE = auto()     # Potential obstacle detected based on range change, storing scan
    CONFIRMING_ROBOT = auto()       # Moving agent_1 to confirm if obstacle is the other robot
    OBSTACLE_DETECTED_HALTED = auto() # Confirmed obstacle is NOT the target robot, process stops here.
    APPROACHING_TARGET = auto()     # Robot confirmed, agent_0 moving closer
    CHECKING_ALIGNMENT = auto()     # Close enough, check alignment status
    ALIGNING = auto()               # Not aligned, performing alignment maneuver with agent_1
    DOCKING = auto()                # Aligned, moving both robots gently together
    LOCKING = auto()                # Sending command to lock the male joint
    VERIFYING_CONNECTION = auto()   # Pulling back agent_0 slightly to verify lock
    CONNECTED = auto()              # Successfully connected
    HANDLING_ALIGNMENT_FAILURE = auto() # Alignment failed, preparing to reset
    RESETTING = auto()              # Moving robots apart after failure
    FAILED = auto()                 # Generic failure state (optional)

class Connect_Robots(Node):

    def __init__(self):
        super().__init__("connect_robots_fsm_node") # Node name reflects FSM

        # --- State Machine Variables ---
        self.current_state = RobotState.IDLE
        self.state_enter_time = self.get_clock().now() # Track when we entered the current state
        self.get_logger().info(f"Node started, initial state: {self.current_state.name}")

        # --- Constants and Configuration (Consider making these ROS Parameters later) ---
        self.initial_delay_sec = 2.0
        self.robot_detection_threshold = 0.1  # 10% difference threshold
        self.connection_verification_threshold = 0.2 # 20% difference threshold for connection test
        self.close_distance_m = 0.15 # Threshold to start alignment/docking
        self.reset_distance_m = 0.3  # Target separation distance during reset
        self.male_lock_angle = math.pi / 2.0 # 90 degrees
        self.male_unlock_angle = 0.0
        self.female_lift_angle = 0.0 # Angle for female joint during connection
        self.alignment_timeout_sec = 40.0 # Corresponds to original 400 count @ 0.1s
        self.docking_duration_sec = 10.0  # Corresponds to original 100 count @ 0.1s
        self.locking_timeout_sec = 5.0    # Time to wait for lock confirmation
        self.verification_duration_sec = 5.0 # Corresponds to original 50 count @ 0.1s
        self.confirmation_timeout_sec = 3.0 # Increased slightly from 2.0 -> 3.0 seconds

        # --- Sensor and State Variables ---
        self.pitch_agent_0 = 0.0
        self.roll_agent_0 = 0.0
        self.pitch_agent_1 = 0.0 # Added for agent 1
        self.roll_agent_1 = 0.0  # Added for agent 1
        self.range = None # Stores the latest reading from the front TOF sensor (agent_0)
        self.aligned = False # Updated by alignment_callback
        self.male_joint_locked = False # Updated by male_joint_state_callback
        self.male_joint_unlocked = False # Updated by male_joint_state_callback

        # Variables used during specific states
        self.initial_scan_range = None
        self.robot_confirm_scan_range = None
        self.locked_connection_range = None

        # Remember last commands to avoid redundant publishing
        self.last_linear_cmd_agent_0 = None
        self.last_angular_cmd_agent_0 = None
        self.last_linear_cmd_agent_1 = None
        self.last_angular_cmd_agent_1 = None

        # --- Publishers ---
        self.cmd_vel_publisher_agent_0_ = self.create_publisher(Twist, "/agent_0/cmd_vel", 10)
        self.cmd_vel_publisher_agent_1_ = self.create_publisher(Twist, "/agent_1/cmd_vel", 10)
        self.male_joint_publisher_ = self.create_publisher(JointTrajectory, '/agent_0/male_joint_trajectory_controller/joint_trajectory', 10)
        self.female_joint_publisher_ = self.create_publisher(JointTrajectory, '/agent_1/female_joint_trajectory_controller/joint_trajectory', 10)

        # --- Subscribers ---
        self.imu_subscriber_agent_0_ = self.create_subscription(Imu, "/agent_0/imu/out", self.imu_callback_agent_0, 10)
        self.imu_subscriber_agent_1_ = self.create_subscription(Imu, "/agent_1/imu/out", self.imu_callback_agent_1, 10) # Added Agent 1 IMU sub
        self.front_tof_subscriber_ = self.create_subscription(Range, "/agent_0/infrared_range", self.infrared_range_callback, 10)
        self.alignment_subscriber_ = self.create_subscription(Bool, "/agent_0/alignment", self.alignment_callback, 10)
        self.male_joint_subscriber_ = self.create_subscription(JointTrajectoryControllerState, '/agent_0/male_joint_trajectory_controller/controller_state', self.male_joint_state_callback, 10)

        # --- Main Loop Timer ---
        self.timer = self.create_timer(0.1, self.state_machine_tick) # 10 Hz loop

        self.get_logger().info("Connect_Robots FSM node initialized.")

    # ==========================================================================
    # State Machine Core Logic
    # ==========================================================================

    def state_machine_tick(self):
        """Main loop executed at regular intervals."""
        if self.range is None:
            # Wait for first sensor readings before starting logic
            self.get_logger().info("Waiting for initial sensor readings...", throttle_duration_sec=5.0)
            return

        # Calculate time spent in the current state
        now = self.get_clock().now()
        time_in_state = (now - self.state_enter_time).nanoseconds / 1e9 # Convert to seconds

        # --- State Execution ---
        if self.current_state == RobotState.IDLE:
            self._handle_idle_state(time_in_state)
        elif self.current_state == RobotState.INITIALIZING_ROBOTS:
            self._handle_initializing_robots_state()
        elif self.current_state == RobotState.MOVING_FORWARD:
            self._handle_moving_forward_state()
        elif self.current_state == RobotState.DETECTING_OBSTACLE:
            self._handle_detecting_obstacle_state()
        elif self.current_state == RobotState.CONFIRMING_ROBOT:
            self._handle_confirming_robot_state(time_in_state)
        elif self.current_state == RobotState.OBSTACLE_DETECTED_HALTED:
            self._handle_obstacle_detected_halted_state() 
        elif self.current_state == RobotState.APPROACHING_TARGET:
            self._handle_approaching_target_state()
        elif self.current_state == RobotState.CHECKING_ALIGNMENT:
            self._handle_checking_alignment_state()
        elif self.current_state == RobotState.ALIGNING:
            self._handle_aligning_state(time_in_state)
        elif self.current_state == RobotState.DOCKING:
            self._handle_docking_state(time_in_state)
        elif self.current_state == RobotState.LOCKING:
            self._handle_locking_state(time_in_state)
        elif self.current_state == RobotState.VERIFYING_CONNECTION:
            self._handle_verifying_connection_state(time_in_state)
        elif self.current_state == RobotState.CONNECTED:
            self._handle_connected_state()
        elif self.current_state == RobotState.HANDLING_ALIGNMENT_FAILURE:
            self._handle_alignment_failure_state()
        elif self.current_state == RobotState.RESETTING:
            self._handle_resetting_state()
        elif self.current_state == RobotState.FAILED:
            self._handle_failed_state()


    def change_state(self, new_state: RobotState):
        """Helper function to transition to a new state."""
        if new_state != self.current_state:
            self.get_logger().info(f"Changing state from {self.current_state.name} to {new_state.name}")
            self.current_state = new_state
            self.state_enter_time = self.get_clock().now()
            # Reset state-specific variables if necessary
            if new_state != RobotState.CONFIRMING_ROBOT:
                 self.robot_confirm_scan_range = None
            if new_state != RobotState.VERIFYING_CONNECTION:
                 self.locked_connection_range = None


    # ==========================================================================
    # State Handling Functions
    # ==========================================================================

    def _handle_idle_state(self, time_in_state):
        """Waits for the initial delay."""
        if time_in_state >= self.initial_delay_sec:
            self.get_logger().info("Initial delay complete.")
            self.change_state(RobotState.INITIALIZING_ROBOTS)

    def _handle_initializing_robots_state(self):
        """Set initial joint positions and start moving."""
        self.get_logger().info("Initializing: Unlocking male joint, setting female joint.")
        self._set_male_joint(self.male_unlock_angle)
        self._set_female_joint(self.female_lift_angle)
        self.change_state(RobotState.MOVING_FORWARD)


    def _handle_moving_forward_state(self):
        """Move agent_0 forward and check for obstacles."""
        self._send_velocity_command_agent_0(0.1, 0.0) # Move forward
        self._centre_robot_0() # Keep centered while moving

        if self.initial_scan_range is None:
            if self.range > 0.05: # Ensure initial scan is valid before storing
                self.initial_scan_range = self.range
                self.get_logger().info(f"Stored initial scan range: {self.initial_scan_range:.3f} m")
            return # Wait for next tick to compare or get a valid initial scan

        # Check if range has changed significantly from initial empty space scan
        if self.initial_scan_range > 0.05: # Avoid division by zero/small numbers
             # Use current range, ensure it's valid before calculation
             current_range = self.range if self.range is not None else self.initial_scan_range
             if current_range < 0.05: return # Skip comparison if current range is invalidly small

             difference = abs(current_range - self.initial_scan_range) / self.initial_scan_range
             if difference > self.robot_detection_threshold:
                 self.get_logger().info(f"Significant range change detected (difference: {difference:.2f}). Potential obstacle.")
                 self._send_velocity_command_agent_0(0.0, 0.0) # Stop agent 0
                 self.change_state(RobotState.DETECTING_OBSTACLE)


    def _handle_detecting_obstacle_state(self):
        """Store the range to the detected obstacle, prepare for confirmation."""
        # Ensure range is valid before storing
        if self.range is not None and self.range > 0.05:
            self.robot_confirm_scan_range = self.range
            self.get_logger().info(f"Stored obstacle scan range: {self.robot_confirm_scan_range:.3f} m. Attempting confirmation.")
            self.change_state(RobotState.CONFIRMING_ROBOT)
        else:
            # If range became invalid just as we detected, maybe retry detection?
            self.get_logger().warn("Range invalid while trying to store obstacle scan. Reverting to MOVING_FORWARD.")
            self.initial_scan_range = None # Reset initial scan too
            self.change_state(RobotState.MOVING_FORWARD)


    def _handle_confirming_robot_state(self, time_in_state):
        """Move agent_1 slightly to see if the range reading changes."""
        # Command agent 1 to move forward
        self._send_velocity_command_agent_1(0.1, 0.0)
        self._centre_robot_1() # Center agent 1 while it moves

        if self.robot_confirm_scan_range is None:
             self.get_logger().error("Error: robot_confirm_scan_range not set!")
             self.change_state(RobotState.FAILED)
             return

        # Ensure current range is valid for comparison
        current_range = self.range
        if current_range is None or current_range < 0.05:
            # If range is lost during confirmation, treat as failure? Or wait?
            # For now, let timeout handle it if range doesn't recover.
            self.get_logger().warn("Range reading lost during robot confirmation.", throttle_duration_sec=5.0)
            pass # Continue letting agent 1 move for the timeout duration
        else:
            # Check if range changes significantly, indicating the obstacle (agent_1) moved
            if self.robot_confirm_scan_range > 0.05: # Avoid division by zero/small numbers
                difference = abs(current_range - self.robot_confirm_scan_range) / self.robot_confirm_scan_range
                # Use a slightly larger threshold maybe, as agent 1 moving *towards* agent 0 changes range
                if difference > (1.5 * self.robot_detection_threshold):
                    self.get_logger().info(f"Range changed significantly (difference: {difference:.2f}). Robot confirmed.")
                    self._send_velocity_command_agent_1(0.0, 0.0) # Stop agent 1
                    self.change_state(RobotState.APPROACHING_TARGET)
                    return # Exit state handler

        # Timeout check: If time exceeds limit and robot wasn't confirmed
        if time_in_state > self.confirmation_timeout_sec:
            self.get_logger().warning("Robot confirmation timed out or range did not change sufficiently.")
            self._send_velocity_command_agent_1(0.0, 0.0) # Stop agent 1
            # --- Obstacle Detected (Not Target) ---
            # Transition to the new Halted state instead of failing/resetting
            self.change_state(RobotState.OBSTACLE_DETECTED_HALTED)


    def _handle_obstacle_detected_halted_state(self):
        """Stop robots and print message indicating an obstacle was detected and process stops."""
        # Ensure robots are stopped
        self._send_velocity_command_agent_0(0.0, 0.0)
        self._send_velocity_command_agent_1(0.0, 0.0)
        # Print message only once
        self.get_logger().info("Obstacle detected (not the target robot). Halting connection process.", once=True)
        # Stay in this state, no further actions related to connection


    def _handle_approaching_target_state(self):
        """Move agent_0 towards the confirmed target (agent_1)."""
        self._centre_robot_0()

        current_range = self.range
        if current_range < self.close_distance_m:
            self.get_logger().info(f"Reached close distance ({current_range:.3f} m). Stopping approach.")
            self._send_velocity_command_agent_0(0.0, 0.0)
            self.change_state(RobotState.CHECKING_ALIGNMENT)
        else:
            # Move forward slower as we get closer
            speed = max(0.02, 0.1 * (current_range - self.close_distance_m))
            self._send_velocity_command_agent_0(speed, 0.0)


    def _handle_checking_alignment_state(self):
        """Check the alignment status received from the callback."""
        self.get_logger().info("Checking alignment status...")
        if self.aligned:
            self.get_logger().info("Robots are aligned.")
            self.change_state(RobotState.DOCKING)
        else:
            self.get_logger().info("Robots are not aligned. Starting alignment procedure.")
            self.change_state(RobotState.ALIGNING)


    def _handle_aligning_state(self, time_in_state):
        """Perform the alignment maneuver by rotating agent_1."""
        self._send_velocity_command_agent_0(0.0, 0.0) # Agent 0 waits
        self._centre_robot_1() # Center agent 1 while aligning

        # Check if alignment succeeded (via callback)
        if self.aligned:
            self.get_logger().info("Alignment successful during maneuver.")
            self._send_velocity_command_agent_1(0.0, 0.0) # Stop agent 1 rotation
            self.change_state(RobotState.DOCKING)
            return

        # Timeout check
        if time_in_state > self.alignment_timeout_sec:
            self.get_logger().error("Alignment timed out!")
            self._send_velocity_command_agent_1(0.0, 0.0) # Stop agent 1 rotation
            self.change_state(RobotState.HANDLING_ALIGNMENT_FAILURE)
            return

        # Alignment turning logic (based on original counts translated to time)
        # Cycle: 1s left, 2s right, 1s left (total 4s per cycle)
        time_in_cycle = time_in_state % 4.0
        angular_z = 0.0
        if time_in_cycle <= 1.0: angular_z = 0.15
        elif time_in_cycle <= 3.0: angular_z = -0.15
        else: angular_z = 0.15
        self._send_velocity_command_agent_1(0.0, angular_z)


    def _handle_docking_state(self, time_in_state):
        """Move both robots gently towards each other."""
        self.get_logger().info(f"Docking maneuver... (time: {time_in_state:.1f}/{self.docking_duration_sec:.1f})", throttle_duration_sec=1.0)
        self._send_velocity_command_agent_0(0.01, 0.0)
        self._send_velocity_command_agent_1(-0.015, 0.0)
        self._centre_robot_0()
        self._centre_robot_1()

        if time_in_state >= self.docking_duration_sec:
            self.get_logger().info("Docking duration complete. Attempting lock.")
            self._send_velocity_command_agent_0(0.0, 0.0)
            self._send_velocity_command_agent_1(0.0, 0.0)
            self.change_state(RobotState.LOCKING)


    def _handle_locking_state(self, time_in_state):
        """Command the male joint to lock and wait for confirmation."""
        if time_in_state < 0.2: # Send command near the beginning
            self.get_logger().info("Commanding male joint to lock.")
            self._set_male_joint(self.male_lock_angle)

        if self.male_joint_locked:
            self.get_logger().info("Male joint lock confirmed by feedback.")
            self.change_state(RobotState.VERIFYING_CONNECTION)
            return

        if time_in_state > self.locking_timeout_sec:
            self.get_logger().error("Locking timed out! Joint did not confirm lock.")
            self.change_state(RobotState.FAILED)


    def _handle_verifying_connection_state(self, time_in_state):
        """Pull agent_0 back slightly and check if agent_1 follows (range stays constant)."""
        current_range = self.range
        if self.locked_connection_range is None:
            if current_range is not None and current_range > 0.01:
                self.locked_connection_range = current_range
                self.get_logger().info(f"Verifying connection. Stored locked range: {self.locked_connection_range:.3f} m.")
            else:
                self.get_logger().warn("Cannot start verification, range invalid.", throttle_duration_sec=5.0)
                # Option: Fail immediately? Or wait briefly for range to recover?
                if time_in_state > 1.0: # Wait 1s for range recovery
                    self.get_logger().error("Verification failed: Invalid range at start.")
                    self.change_state(RobotState.FAILED)
                return # Wait a bit longer

        self._send_velocity_command_agent_0(-0.05, 0.0)
        self._centre_robot_0()

        if time_in_state >= self.verification_duration_sec:
            self._send_velocity_command_agent_0(0.0, 0.0)

            if self.locked_connection_range is None or current_range is None:
                self.get_logger().error("Verification failed: Invalid range during check.")
                self.change_state(RobotState.FAILED)
                return

            connection_difference = abs(current_range - self.locked_connection_range) / self.locked_connection_range
            self.get_logger().info(f"Verification check: Current range {current_range:.3f}, Locked range {self.locked_connection_range:.3f}, Difference {connection_difference:.2f}")

            if connection_difference < self.connection_verification_threshold:
                self.get_logger().info("Connection verified successfully!")
                self.change_state(RobotState.CONNECTED)
            else:
                self.get_logger().error("Connection verification failed! Robots separated.")
                self._set_male_joint(self.male_unlock_angle) # Try unlocking
                self.change_state(RobotState.RESETTING)


    def _handle_connected_state(self):
        """Final state, robots are connected. Stop all motion."""
        self._send_velocity_command_agent_0(0.0, 0.0)
        self._send_velocity_command_agent_1(0.0, 0.0)
        self.get_logger().info("Robots successfully connected.", once=True)


    def _handle_alignment_failure_state(self):
        """Handles the case where alignment timed out."""
        self.get_logger().error("Alignment failed. Initiating reset procedure.")
        self._send_velocity_command_agent_0(0.0, 0.0)
        self._send_velocity_command_agent_1(0.0, 0.0)
        self._set_male_joint(self.male_unlock_angle)
        self.change_state(RobotState.RESETTING)


    def _handle_resetting_state(self):
        """Move robots apart to the defined reset distance."""
        current_range = self.range if self.range is not None else self.reset_distance_m # Assume target if invalid
        self.get_logger().info(f"Resetting: Moving robots apart. Current range: {current_range:.3f} m / Target: {self.reset_distance_m:.3f} m", throttle_duration_sec=1.0)

        if current_range < self.reset_distance_m:
            self._send_velocity_command_agent_0(-0.01, 0.0)
            self._send_velocity_command_agent_1(0.02, 0.0)
            self._centre_robot_0()
            self._centre_robot_1()
        else:
            self.get_logger().info("Reset complete. Robots separated.")
            self._send_velocity_command_agent_0(0.0, 0.0)
            self._send_velocity_command_agent_1(0.0, 0.0)
            self.initial_scan_range = None
            self.aligned = False
            self.male_joint_locked = False
            self.male_joint_unlocked = False
            self.change_state(RobotState.IDLE) # Restart process


    def _handle_failed_state(self):
        """Generic failure state. Stop robots and log."""
        self.get_logger().error("Connection process failed.", once=True)
        self._send_velocity_command_agent_0(0.0, 0.0)
        self._send_velocity_command_agent_1(0.0, 0.0)


    # ==========================================================================
    # Helper Functions (Movement, Joints, Centering)
    # ==========================================================================

    def _send_velocity_command_agent_0(self, linear_x, angular_z):
        """Publishes velocity commands to agent_0, avoiding duplicates."""
        if linear_x != self.last_linear_cmd_agent_0 or angular_z != self.last_angular_cmd_agent_0:
            cmd = Twist()
            cmd.linear.x = float(linear_x)
            cmd.angular.z = float(angular_z)
            self.cmd_vel_publisher_agent_0_.publish(cmd)
            self.last_linear_cmd_agent_0 = linear_x
            self.last_angular_cmd_agent_0 = angular_z

    def _send_velocity_command_agent_1(self, linear_x, angular_z):
        """Publishes velocity commands to agent_1, avoiding duplicates."""
        if linear_x != self.last_linear_cmd_agent_1 or angular_z != self.last_angular_cmd_agent_1:
            cmd = Twist()
            cmd.linear.x = float(linear_x)
            cmd.angular.z = float(angular_z)
            self.cmd_vel_publisher_agent_1_.publish(cmd)
            self.last_linear_cmd_agent_1 = linear_x
            self.last_angular_cmd_agent_1 = angular_z

    def _centre_robot_0(self):
            """Calculates and sends centering command based on agent_0's roll, adjusting for direction."""
            # --- Gain Configuration ---
            # NOTE: Assumes k_p=-0.05 produced the correct turn direction for FORWARD motion in the previous version.
            # This implies that a positive physical roll (e.g., right side down) resulted in a NEGATIVE self.roll_agent_0 value.
            k_p_forward = -0.05 # Original gain assumed correct for forward

            angular_z_cmd = 0.0
            roll_threshold_rad = math.radians(0.5) # ~0.5 degrees threshold

            # --- Determine current linear direction ---
            # Use the *intended* command, not just the last published one, if available,
            # but last published is usually sufficient here.
            current_linear = self.last_linear_cmd_agent_0 if self.last_linear_cmd_agent_0 is not None else 0.0
            is_reversing = current_linear < -0.001 # Use a small tolerance to avoid issues near zero

            # --- Calculate Correction ---
            if abs(self.roll_agent_0) > roll_threshold_rad:
                # If moving forward (or stationary), use the forward gain.
                # If moving backward, the required angular velocity sign to correct the roll is flipped.
                effective_k_p = -k_p_forward if is_reversing else k_p_forward # Flip gain sign if reversing

                # Calculate the desired angular velocity
                angular_z_cmd = effective_k_p * self.roll_agent_0

                # Clamp maximum angular velocity correction (optional but recommended)
                max_correction_vel = 0.2 # rad/s
                angular_z_cmd = max(-max_correction_vel, min(max_correction_vel, angular_z_cmd))

            # --- Publish Command ---
            # Only publish if the angular command *needs* changing significantly.
            # The linear part is set by the state machine logic calling this helper.
            # Check against the *last commanded* angular Z for this agent.
            last_angular = self.last_angular_cmd_agent_0 if self.last_angular_cmd_agent_0 is not None else 0.0
            if abs(angular_z_cmd - last_angular) > 0.005: # Publish if changed significantly
                # Note: We send the 'current_linear' which was the *last commanded* linear.
                # This assumes the state machine sets the desired linear speed, and centering
                # only adjusts the angular speed.
                self._send_velocity_command_agent_0(current_linear, angular_z_cmd)
            # If angular_z_cmd is close to the last command, do nothing to avoid jitter,
            # unless linear command is also changing (handled by _send_velocity_command check).


    def _centre_robot_1(self):
        """Calculates and sends centering command based on agent_1's roll, adjusting for direction."""
        # --- Gain Configuration ---
        k_p_forward = -0.05 # Original gain assumed correct for forward

        angular_z_cmd = 0.0
        roll_threshold_rad = math.radians(0.5) # ~0.5 degrees threshold

        # --- Determine current linear direction ---
        current_linear = self.last_linear_cmd_agent_1 if self.last_linear_cmd_agent_1 is not None else 0.0
        is_reversing = current_linear < -0.001 # Use a small tolerance

        # --- Calculate Correction ---
        if abs(self.roll_agent_1) > roll_threshold_rad:
            # Flip gain sign if reversing
            effective_k_p = -k_p_forward if is_reversing else k_p_forward

            # Calculate the desired angular velocity
            angular_z_cmd = effective_k_p * self.roll_agent_1

            # Clamp maximum angular velocity correction (optional but recommended)
            max_correction_vel = 0.2 # rad/s
            angular_z_cmd = max(-max_correction_vel, min(max_correction_vel, angular_z_cmd))

        # --- Publish Command ---
        last_angular = self.last_angular_cmd_agent_1 if self.last_angular_cmd_agent_1 is not None else 0.0
        if abs(angular_z_cmd - last_angular) > 0.005: # Publish if changed significantly
            self._send_velocity_command_agent_1(current_linear, angular_z_cmd)


    def _set_male_joint(self, position):
        """Commands the male joint to a specific position."""
        msg = JointTrajectory()
        msg.joint_names = ['base_male_joint']
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(sec=2, nanosec=0)
        msg.points.append(point)
        self.male_joint_publisher_.publish(msg)
        self.male_joint_locked = False
        self.male_joint_unlocked = False

    def _set_female_joint(self, position):
        """Commands the female joint to a specific position."""
        msg = JointTrajectory()
        msg.joint_names = ['base_female_joint']
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(sec=2, nanosec=0)
        msg.points.append(point)
        self.female_joint_publisher_.publish(msg)


    # ==========================================================================
    # Subscriber Callbacks
    # ==========================================================================

    def _calculate_imu_rp(self, msg: Imu):
        """Helper to calculate roll/pitch from accelerometer data."""
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        accel_mag_sq = ax**2 + ay**2 + az**2

        # Return (None, None) if acceleration is too small (prevents division by zero/instability)
        if accel_mag_sq < 0.01:
             # self.get_logger().warn("IMU acceleration near zero, cannot calculate roll/pitch.", throttle_duration_sec=10.0)
             return None, None

        # Calculate roll and pitch in radians using atan2 for robustness
        # Ensure axis definitions match your hardware setup!
        # Common convention: Roll is rotation around X (forward), Pitch around Y (left)
        try:
            # Roll (rad): Rotation around X. Using atan2(ay, az) is common but sensitive if az is near 0.
            # Pitch (rad): Rotation around Y. Using atan2(-ax, sqrt(ay^2 + az^2))
            # Using the definition from your original code more directly:
            # roll = math.atan2(-ax, math.sqrt(ay**2 + az**2)) # Your original 'roll' definition
            # pitch = math.atan2(ay, math.sqrt(ax**2 + az**2)) # Your original 'pitch' definition

            # Let's stick to your 'roll' calculation (atan2(-ax, sqrt(ay^2 + az^2))) for consistency
            roll_rad = math.atan2(-ax, math.sqrt(ay**2 + az**2))
            pitch_rad = math.atan2(ay, math.sqrt(ax**2 + az**2)) # Keep pitch calc too

            return roll_rad, pitch_rad

        except ValueError:
             self.get_logger().warn("Math domain error in IMU calculation", once=True)
             return None, None


    def imu_callback_agent_0(self, msg: Imu):
        """Updates roll and pitch based on agent_0's IMU data."""
        roll, pitch = self._calculate_imu_rp(msg)
        if roll is not None:
            self.roll_agent_0 = roll
            self.pitch_agent_0 = pitch

    def imu_callback_agent_1(self, msg: Imu):
        """Updates roll and pitch based on agent_1's IMU data."""
        roll, pitch = self._calculate_imu_rp(msg)
        if roll is not None:
            self.roll_agent_1 = roll
            self.pitch_agent_1 = pitch


    def infrared_range_callback(self, msg: Range):
        """Updates the stored range value from agent_0's TOF sensor."""
        # Check for valid range values before storing
        if math.isfinite(msg.range) and msg.range >= msg.min_range and msg.range <= msg.max_range:
            self.range = msg.range
        else:
            # If range is invalid, set to None to indicate bad reading
            if self.range is not None: # Log only when it changes to invalid
                 self.get_logger().warn(f"Invalid range reading received: {msg.range}. Setting range to None.", throttle_duration_sec=5.0)
            self.range = None


    def alignment_callback(self, msg: Bool):
        """Updates the alignment status flag."""
        if msg.data != self.aligned:
             self.aligned = msg.data
             self.get_logger().info(f"Alignment status changed to: {self.aligned}")


    def male_joint_state_callback(self, msg: JointTrajectoryControllerState):
        """Updates flags based on the male joint's current position."""
        if not msg.feedback or not msg.feedback.positions:
            return
        current_pos = msg.feedback.positions[0]
        tolerance = 0.1 # Radians

        is_unlocked = abs(current_pos - self.male_unlock_angle) < tolerance
        is_locked = abs(current_pos - self.male_lock_angle) < tolerance

        if is_unlocked != self.male_joint_unlocked or is_locked != self.male_joint_locked:
            # Log only on change
            # self.get_logger().info(f"Male joint state update: Unlocked={is_unlocked}, Locked={is_locked}")
            self.male_joint_unlocked = is_unlocked
            self.male_joint_locked = is_locked


def main(args=None):
    rclpy.init(args=args)
    node = Connect_Robots()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received, shutting down.")
    finally:
        node.get_logger().info("Stopping robots on shutdown.")
        node._send_velocity_command_agent_0(0.0, 0.0)
        node._send_velocity_command_agent_1(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()