#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu, Range
import math
import re # Needed for discovery pattern matching
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from builtin_interfaces.msg import Duration
from enum import Enum, auto
import time
from functools import partial # Needed for callbacks with arguments
from collections import defaultdict # Useful for initializing dicts
import traceback # For better error logging

# ==========================================================================
# Enums for States (Simplified for now)
# ==========================================================================
class RobotState(Enum):
    DISCOVERING_ROBOTS = auto()     # Finding robots on the network
    IDLE = auto()                   # Waiting after discovery before starting
    INITIALIZING_ROBOTS = auto()    # Setting initial joint positions and status
    MOVING_ALL_FORWARD = auto()     # Basic state: Move all active robots forward
    HALTED = auto()                 # Final state if connection logic isn't added or discovery fails
    FAILED = auto()                 # Generic failure state

class AgentStatus(Enum):
    """Individual status of a single robot."""
    UNKNOWN = auto()                # Initial status
    ACTIVE = auto()                 # Initialized and ready/moving
    # Add other statuses later (DETECTED_SOMETHING, CONNECTED, etc.)
    FAILED = auto()                 # Individual failure (e.g., resource creation)

# ==========================================================================
# Main Node Class (Combining Discovery/Init with Simple Structure)
# ==========================================================================
class Connect_Robots_Scalable_Simple(Node):

    def __init__(self):
        super().__init__("connect_robots_scalable_simple_node")

        # --- Discovery ---
        self.robots = {}                    # Basic info {agent_id: {'ns': ns}}
        self.discovered_agent_ids = set()
        self.num_discovered_robots = 0
        self.discovery_complete = False
        self.discovery_timer_period = 2.0
        self.discovery_attempts = 0
        self.max_discovery_attempts = 5     # Try 5 times to find stable robot count

        # --- Dynamic Pub/Sub/Data Storage (From Complex Script) ---
        self.agent_pubs = {'cmd_vel': {}, 'male_joint': {}, 'female_joint': {}}
        self.agent_subs = {'imu': {}, 'range': {}, 'alignment': {}, 'male_joint_state': {}}
        # Store latest sensor data keyed by agent_id, defaulting nested keys
        self.agent_sensor_data = defaultdict(lambda: defaultdict(lambda: None))
        self._initialize_sensor_data_keys() # Ensure top-level keys exist
        self.agent_last_cmd_vel = {} # {agent_id: (linear, angular)} to avoid spam

        # --- Multi-Robot State Tracking (From Complex Script) ---
        self.robot_status = {}           # {agent_id: AgentStatus}

        # --- State Machine ---
        self.current_state = RobotState.DISCOVERING_ROBOTS
        self.state_enter_time = self.get_clock().now()
        self.get_logger().info(f"Node starting in state: {self.current_state.name}")

        # --- Constants and Configuration ---
        # Timing
        self.initial_delay_sec = 2.0 # Delay in IDLE state before initializing
        # Speeds
        self.forward_speed = 0.1
        # Joints
        self.male_lock_angle = math.pi / 2.0
        self.male_unlock_angle = 0.0
        self.female_lift_angle = 0.0
        self.joint_pos_tolerance = 0.1 # Radians for joint state checking
        # Control
        self.kp_centering_forward = -0.05

        # --- Timers ---
        self.get_logger().info("Creating discovery timer...")
        self.discovery_timer = self.create_timer(self.discovery_timer_period, self.discover_robots)
        if not self.discovery_timer:
            self.get_logger().error("Failed to create discovery timer!")
            # Consider entering FAILED state immediately? Or let discovery timeout handle it.

        self.fsm_loop_timer = self.create_timer(0.1, self.state_machine_tick) # 10 Hz FSM loop

        self.get_logger().info("Scalable Simple Connect_Robots node initialized, starting discovery.")

    def _initialize_sensor_data_keys(self):
        """Ensure top-level keys for sensor data dictionary exist."""
        # Add keys as needed when subscribers/logic are added
        keys = ['roll', 'pitch', 'range', 'aligned', 'male_locked', 'male_unlocked']
        for key in keys:
            # Ensure the outer dictionary exists for each sensor type
            if key not in self.agent_sensor_data:
                 self.agent_sensor_data[key] = {} # Initialize as an empty dict

    # ==========================================================================
    # Robot Discovery (Copied from Complex Script)
    # ==========================================================================
    def discover_robots(self):
        """Periodically scans topics to find agent_N robots."""
        if self.discovery_complete:
            self.discovery_timer.cancel() # Should already be cancelled by completion logic
            return

        self.get_logger().info(f"Discovery attempt {self.discovery_attempts + 1}/{self.max_discovery_attempts}...")
        found_new = False
        current_ids_found_this_scan = set()

        try:
            topic_list = self.get_topic_names_and_types()
            # Using a common topic like imu/out or cmd_vel might be more robust than robot_description
            agent_topic_pattern = re.compile(r'/agent_(\d+)/imu/out') # Adjust if needed

            for topic_name, _ in topic_list:
                match = agent_topic_pattern.match(topic_name)
                if match:
                    agent_id = int(match.group(1))
                    current_ids_found_this_scan.add(agent_id)
                    if agent_id not in self.discovered_agent_ids:
                        self.get_logger().info(f"Discovered new agent_{agent_id}")
                        self._register_robot(agent_id) # Register its resources
                        self.discovered_agent_ids.add(agent_id)
                        found_new = True

            self.num_discovered_robots = len(self.discovered_agent_ids)

        except Exception as e:
            self.get_logger().error(f"Error during robot discovery scan: {e}", exc_info=True)
            # Optionally handle specific errors differently

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
                 if self.num_discovered_robots >= 1: # Need at least one to do anything
                      self.get_logger().info(f"Discovery finished. Found {self.num_discovered_robots} robot(s). Transitioning to IDLE.")
                      self.change_state(RobotState.IDLE)
                 else:
                      self.get_logger().error("Discovery finished, but no robots found. Entering FAILED state.")
                      self.change_state(RobotState.FAILED)
            # else: We might be in another state if something went wrong, let FSM handle it

    # ==========================================================================
    # Robot Resource Registration (Copied from Complex Script)
    # ==========================================================================
    def _register_robot(self, agent_id):
        """Creates publishers, subscribers, and initializes data for a newly discovered robot."""
        if agent_id in self.robots:
            self.get_logger().warn(f"Attempted to re-register agent {agent_id}.")
            return

        ns = f"agent_{agent_id}"
        self.robots[agent_id] = {'ns': ns}
        self.get_logger().info(f"Registering resources for {ns}")

        # Initialize sensor data storage using agent_id keys
        # Use defaultdict's behavior - access will create if needed with default value (None)
        self.agent_sensor_data['roll'][agent_id] = 0.0 # Initial guess
        self.agent_sensor_data['pitch'][agent_id] = 0.0 # Initial guess
        self.agent_sensor_data['range'][agent_id] = None
        self.agent_sensor_data['aligned'][agent_id] = False # Assume not aligned initially
        self.agent_sensor_data['male_locked'][agent_id] = False # Assume not locked initially
        self.agent_sensor_data['male_unlocked'][agent_id] = True # Assume unlocked initially
        self.agent_last_cmd_vel[agent_id] = (None, None) # Use None to force first command
        self.robot_status[agent_id] = AgentStatus.UNKNOWN # Status before initialization

        # Create Publishers
        try:
            # Check if pub already exists (e.g., from a previous failed registration attempt)
            if agent_id not in self.agent_pubs['cmd_vel']:
                 self.agent_pubs['cmd_vel'][agent_id] = self.create_publisher(Twist, f"/{ns}/cmd_vel", 10)
            if agent_id not in self.agent_pubs['male_joint']:
                 self.agent_pubs['male_joint'][agent_id] = self.create_publisher(JointTrajectory, f"/{ns}/male_joint_trajectory_controller/joint_trajectory", 10)
            if agent_id not in self.agent_pubs['female_joint']:
                 self.agent_pubs['female_joint'][agent_id] = self.create_publisher(JointTrajectory, f"/{ns}/female_joint_trajectory_controller/joint_trajectory", 10)

            # Create Subscribers - Use partial to pass agent_id to the callback
            # Check if sub already exists
            if agent_id not in self.agent_subs['imu']:
                 self.agent_subs['imu'][agent_id] = self.create_subscription(
                     Imu, f"/{ns}/imu/out", partial(self.imu_callback, agent_id=agent_id), 10)
            if agent_id not in self.agent_subs['range']:
                 self.agent_subs['range'][agent_id] = self.create_subscription(
                     Range, f"/{ns}/infrared_range", partial(self.range_callback, agent_id=agent_id), 10)
            if agent_id not in self.agent_subs['alignment']:
                 self.agent_subs['alignment'][agent_id] = self.create_subscription(
                     Bool, f"/{ns}/alignment", partial(self.alignment_callback, agent_id=agent_id), 10)
            if agent_id not in self.agent_subs['male_joint_state']:
                 self.agent_subs['male_joint_state'][agent_id] = self.create_subscription(
                     JointTrajectoryControllerState, f"/{ns}/male_joint_trajectory_controller/controller_state", partial(self.male_joint_state_callback, agent_id=agent_id), 10)

            self.get_logger().info(f"Successfully registered resources for agent {agent_id}")

        except Exception as e:
            self.get_logger().error(f"Failed to create resources for agent {agent_id}: {e}", exc_info=True)
            # Mark as FAILED if resources couldn't be created
            self.robot_status[agent_id] = AgentStatus.FAILED
            # Clean up partially created resources? Maybe difficult, rely on node shutdown for now.


    # ==========================================================================
    # State Machine Core Logic
    # ==========================================================================
    def state_machine_tick(self):
        """Main FSM loop, called periodically by a timer."""
        # --- Wait for Discovery ---
        if not self.discovery_complete:
            # discover_robots() handles the transition out of DISCOVERING_ROBOTS
            if self.current_state == RobotState.DISCOVERING_ROBOTS:
                self.get_logger().debug("Waiting for discovery to complete...", throttle_duration_sec=5.0)
            else:
                 # Should not happen if discovery timer is cancelled correctly
                 self.get_logger().warn(f"Discovery not complete, but in state {self.current_state.name}. Waiting.")
            return

        # --- Discovery is Complete, Proceed with FSM ---
        now = self.get_clock().now()
        time_in_state = (now - self.state_enter_time).nanoseconds / 1e9

        # --- State Execution Mapping ---
        state_handler_map = {
            RobotState.IDLE: self._handle_idle_state,
            RobotState.INITIALIZING_ROBOTS: self._handle_initializing_robots_state,
            RobotState.MOVING_ALL_FORWARD: self._handle_moving_all_forward_state,
            RobotState.HALTED: self._handle_halted_state,
            RobotState.FAILED: self._handle_failed_state,
        }

        handler = state_handler_map.get(self.current_state)

        if handler:
            try:
                # Pass time_in_state only if handler needs it (only IDLE for now)
                if self.current_state == RobotState.IDLE:
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


    def change_state(self, new_state: RobotState):
        """Helper function to transition to a new state."""
        if new_state != self.current_state:
            self.get_logger().info(f"Changing state from {self.current_state.name} to {new_state.name}")
            self.current_state = new_state
            self.state_enter_time = self.get_clock().now()
            # Reset any state-specific variables here if needed in the future

    # ==========================================================================
    # State Handling Functions (Simplified Set)
    # ==========================================================================

    def _handle_idle_state(self, time_in_state):
        """Waits for a short delay after discovery before initializing."""
        # Optional: Check if all registered robots have necessary pubs/subs ready
        all_ready = all(agent_id in self.agent_pubs.get('cmd_vel', {})
                        for agent_id in self.discovered_agent_ids
                        if self.robot_status.get(agent_id) != AgentStatus.FAILED)

        if not all_ready:
             self.get_logger().warn("Waiting for all cmd_vel publishers to be registered for non-failed robots...", throttle_duration_sec=5.0)
             # Check for permanent failure? If a robot stays FAILED, maybe proceed anyway?
             # For now, we wait.
             return

        if time_in_state >= self.initial_delay_sec:
            self.get_logger().info("Initial delay complete.")
            self.change_state(RobotState.INITIALIZING_ROBOTS)

    # --- Uses logic from Complex Script's _handle_initializing_robots_state ---
    def _handle_initializing_robots_state(self):
        """Sets initial joint positions and status for all discovered robots."""
        self.get_logger().info("Initializing all discovered robots...")
        initialization_done = True # Assume done unless proven otherwise

        for agent_id in self.discovered_agent_ids:
            current_status = self.robot_status.get(agent_id, AgentStatus.UNKNOWN)

            # Only initialize if UNKNOWN or FAILED (allow retry)
            if current_status in [AgentStatus.UNKNOWN, AgentStatus.FAILED]:
                self.get_logger().info(f"  Initializing agent_{agent_id}...")
                # Check if resources are actually available before commanding
                if agent_id in self.agent_pubs['male_joint'] and agent_id in self.agent_pubs['female_joint']:
                     self._set_joint(agent_id, 'male', self.male_unlock_angle)
                     self._set_joint(agent_id, 'female', self.female_lift_angle)
                     self.robot_status[agent_id] = AgentStatus.ACTIVE # Mark as ready
                     self.get_logger().info(f"  Agent {agent_id} set to ACTIVE, joints commanded.")
                else:
                    self.get_logger().error(f"  Cannot initialize agent {agent_id}: Missing joint publishers. Marking FAILED.")
                    self.robot_status[agent_id] = AgentStatus.FAILED
                    initialization_done = False # Failed for at least one

            elif current_status == AgentStatus.ACTIVE:
                self.get_logger().debug(f"  Agent {agent_id} already ACTIVE.", throttle_duration_sec=10.0)
                pass # Already initialized
            else:
                 self.get_logger().warn(f"  Agent {agent_id} has unexpected status {current_status.name} during initialization.")
                 # Decide how to handle this - maybe mark FAILED? For now, treat as not done.
                 initialization_done = False


        if initialization_done:
            # Check if *any* robot successfully became ACTIVE
            any_active = any(status == AgentStatus.ACTIVE for status in self.robot_status.values())
            if any_active:
                 self.get_logger().info("All applicable robots initialized.")
                 time.sleep(0.5) # Short delay for joints to potentially start moving
                 self.change_state(RobotState.MOVING_ALL_FORWARD)
            else:
                 self.get_logger().error("Initialization attempted, but no robots became ACTIVE. Entering FAILED state.")
                 self.change_state(RobotState.FAILED)
        # else: Stay in this state, will retry on next tick for those not yet ACTIVE


    def _handle_moving_all_forward_state(self):
        """Basic state: Move all ACTIVE robots forward."""
        self.get_logger().debug("In MOVING_ALL_FORWARD state.", throttle_duration_sec=5.0)
        active_robots_exist = False
        for agent_id in self.discovered_agent_ids:
            if self.robot_status.get(agent_id) == AgentStatus.ACTIVE:
                active_robots_exist = True
                # Send command and apply centering
                self._send_velocity(agent_id, self.forward_speed, 0.0)
                self._centre_robot(agent_id)
                # --- No detection logic here yet ---
                # In the next iteration, you would add checks here:
                # - Get agent_id's range sensor data
                # - Compare to initial/previous values
                # - If significant change -> trigger detection -> change state

        if not active_robots_exist and self.discovery_complete:
             # This case should ideally be caught earlier, but as a fallback:
             self.get_logger().warn("No active robots found in MOVING_ALL_FORWARD. Transitioning to HALTED.")
             self.change_state(RobotState.HALTED)


    def _handle_halted_state(self):
        """Final state for this simplified version. Stops robots."""
        self.get_logger().info("Reached HALTED state. Stopping all robots.", once=True)
        self.stop_all_robots()
        # Stay here. To add connection logic, you'd modify MOVING_ALL_FORWARD
        # to transition to detection/confirmation states instead.

    def _handle_failed_state(self):
        """Generic failure state. Stop robots and log."""
        self.get_logger().error("Connection process entered FAILED state.", once=True)
        self.stop_all_robots()
        # Stay here.


    # ==========================================================================
    # Helper Functions (Generalized from Complex/Simple Scripts)
    # ==========================================================================

    def stop_all_robots(self):
        """Sends zero velocity to all discovered robots."""
        self.get_logger().debug("--- Stopping all discovered robots ---")
        for agent_id in self.discovered_agent_ids:
             # Check if publisher exists before trying to send
             if agent_id in self.agent_pubs['cmd_vel']:
                self._send_velocity(agent_id, 0.0, 0.0)
             else:
                self.get_logger().warn(f"Cannot stop agent {agent_id}, cmd_vel publisher missing.", throttle_duration_sec=10.0)

    def _send_velocity(self, agent_id, linear_x, angular_z):
        """Sends velocity command to the specified agent, avoids duplicates."""
        pub = self.agent_pubs['cmd_vel'].get(agent_id)
        if not pub:
            # Warning logged during registration or stop_all_robots
            return

        # Check cache to avoid redundant sends
        last_lin, last_ang = self.agent_last_cmd_vel.get(agent_id, (None, None))

        # Use a tolerance for comparison, especially important for angular_z from centering
        lin_changed = abs(linear_x - (last_lin if last_lin is not None else -999)) > 0.001
        ang_changed = abs(angular_z - (last_ang if last_ang is not None else -999)) > 0.005 # Slightly larger tolerance for angular

        if lin_changed or ang_changed:
            cmd = Twist()
            cmd.linear.x = float(linear_x)
            cmd.angular.z = float(angular_z)
            try:
                pub.publish(cmd)
                self.agent_last_cmd_vel[agent_id] = (linear_x, angular_z) # Update cache
                # self.get_logger().debug(f"Sent vel to {agent_id}: lin={linear_x:.2f}, ang={angular_z:.2f}")
            except Exception as e:
                 self.get_logger().error(f"Failed to publish velocity for agent {agent_id}: {e}", throttle_duration_sec=5.0)


    def _centre_robot(self, agent_id):
        """Applies centering correction based on roll for the specified agent."""
        # Ensure necessary data and publisher exist
        roll = self.agent_sensor_data['roll'].get(agent_id) # Can be None if no data yet
        pub = self.agent_pubs['cmd_vel'].get(agent_id)

        if roll is None or pub is None:
             # Don't warn aggressively if roll isn't available yet, it might take time.
             # self.get_logger().debug(f"Cannot center agent {agent_id}: Missing roll data or cmd_vel publisher.", throttle_duration_sec=10.0)
             return

        current_linear, last_angular = self.agent_last_cmd_vel.get(agent_id, (0.0, 0.0))
        # Ensure current_linear is a float
        current_linear = current_linear if current_linear is not None else 0.0
        last_angular = last_angular if last_angular is not None else 0.0


        is_reversing = current_linear < -0.001
        roll_threshold_rad = math.radians(0.5)
        angular_z_cmd = 0.0 # Default to no correction

        if abs(roll) > roll_threshold_rad:
            # Correct gain application based on direction
            effective_k_p = -self.kp_centering_forward if is_reversing else self.kp_centering_forward
            angular_z_cmd = effective_k_p * roll
            max_correction_vel = 0.2
            angular_z_cmd = max(-max_correction_vel, min(max_correction_vel, angular_z_cmd))
            # Clamp calculated value
            angular_z_cmd = max(-max_correction_vel, min(max_correction_vel, angular_z_cmd))


        # Send velocity only if angular component changed significantly
        # The _send_velocity function itself handles the caching/duplicate check
        self._send_velocity(agent_id, current_linear, angular_z_cmd)


    def _set_joint(self, agent_id, joint_type, position):
        """Commands 'male' or 'female' joint for the specified agent."""
        if joint_type not in ['male', 'female']:
            self.get_logger().error(f"Invalid joint_type '{joint_type}' specified for agent {agent_id}.")
            return

        dict_key = f"{joint_type}_joint" # 'male_joint' or 'female_joint'
        pub = self.agent_pubs.get(dict_key, {}).get(agent_id)

        if not pub:
             self.get_logger().warn(f"_set_joint: No {dict_key} publisher registered for agent {agent_id}", throttle_duration_sec=10.0)
             return

        # Construct joint name based on convention
        joint_name = f'base_{joint_type}_joint'
        self.get_logger().debug(f"Setting agent_{agent_id} {joint_name} to {position:.3f}")

        msg = JointTrajectory()
        msg.joint_names = [joint_name]
        point = JointTrajectoryPoint()
        point.positions = [float(position)]
        point.time_from_start = Duration(sec=2, nanosec=0) # Adjust time as needed
        msg.points.append(point)

        try:
            pub.publish(msg)
            # Reset internal state flags when commanding male joint
            if joint_type == 'male':
                self.agent_sensor_data['male_locked'][agent_id] = False
                self.agent_sensor_data['male_unlocked'][agent_id] = False
        except Exception as e:
            self.get_logger().error(f"Failed to publish {joint_type} joint command for agent {agent_id}: {e}", throttle_duration_sec=5.0)


    # ==========================================================================
    # Subscriber Callbacks (Generalized using agent_id)
    # ==========================================================================

    def _calculate_imu_rp(self, msg: Imu):
        """Helper to calculate roll/pitch from IMU accelerometer data."""
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        accel_mag_sq = ax**2 + ay**2 + az**2
        if accel_mag_sq < 0.01: return None, None # Avoid division by zero near singularity
        try:
            # Roll around X (forward axis) - atan2(Y, Z) - common convention
            # roll = math.atan2(ay, az)
            # Pitch around Y (left/right axis) - atan2(-X, sqrt(Y^2+Z^2)) - common convention
            # pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))

            # Using the specific calculations from your simpler script for consistency:
            # 'Roll' used for centering was atan2(-ax, sqrt(ay^2 + az^2))
            calc_roll = math.atan2(-ax, math.sqrt(ay**2 + az**2))
            # 'Pitch' was atan2(ay, sqrt(ax^2 + az^2))
            calc_pitch = math.atan2(ay, math.sqrt(ax**2 + az**2))

            return calc_roll, calc_pitch
        except ValueError:
            self.get_logger().warn("Math domain error in IMU calculation", once=True)
            return None, None
        except Exception as e:
             self.get_logger().error(f"Unexpected error in IMU calculation: {e}", once=True)
             return None, None

    # Combined IMU Callback
    def imu_callback(self, msg: Imu, agent_id: int):
        """Stores roll/pitch for the specific agent."""
        roll, pitch = self._calculate_imu_rp(msg)
        # Only update if calculation was successful
        if roll is not None:
             # Check if value actually changed before storing (optional optimization)
             # if self.agent_sensor_data['roll'].get(agent_id) != roll:
             self.agent_sensor_data['roll'][agent_id] = roll
        if pitch is not None:
             # if self.agent_sensor_data['pitch'].get(agent_id) != pitch:
             self.agent_sensor_data['pitch'][agent_id] = pitch


    def range_callback(self, msg: Range, agent_id: int):
        """Stores valid range data for the specific agent."""
        valid_range = None
        # Check for validity (finite, within sensor limits)
        if math.isfinite(msg.range) and msg.range >= msg.min_range and msg.range <= msg.max_range:
             valid_range = msg.range
        else:
             # Log if the range *becomes* invalid
             if self.agent_sensor_data['range'].get(agent_id) is not None:
                 self.get_logger().warn(f"Agent {agent_id} received invalid range: {msg.range}. Storing None.", throttle_duration_sec=5.0)

        # Store the valid range or None if invalid
        # Optional: check if value changed before assignment
        # if self.agent_sensor_data['range'].get(agent_id) != valid_range:
        self.agent_sensor_data['range'][agent_id] = valid_range


    def alignment_callback(self, msg: Bool, agent_id: int):
        """Stores alignment status for the specific agent."""
        # Store only if changed
        if self.agent_sensor_data['aligned'].get(agent_id) != msg.data:
             self.agent_sensor_data['aligned'][agent_id] = msg.data
             # Log the change for debugging
             self.get_logger().debug(f"Agent {agent_id} alignment status changed to: {msg.data}")


    def male_joint_state_callback(self, msg: JointTrajectoryControllerState, agent_id: int):
        """Stores male lock/unlock status based on reported joint position."""
        # Prioritize 'actual' positions, fallback to 'feedback'
        pos_list = None
        if msg.actual and msg.actual.positions:
            pos_list = msg.actual.positions
        elif msg.feedback and msg.feedback.positions: # Fallback
            pos_list = msg.feedback.positions
        # Add other fallbacks if needed (e.g., desired?)

        if pos_list is None or not pos_list: # Check if list exists and is not empty
            # self.get_logger().warn(f"No valid position data in JointTrajectoryControllerState for agent {agent_id}", throttle_duration_sec=10.0)
            return # No position data available

        current_pos = pos_list[0] # Assuming single joint controller

        # Check against defined angles using tolerance
        is_unlocked = abs(current_pos - self.male_unlock_angle) < self.joint_pos_tolerance
        is_locked = abs(current_pos - self.male_lock_angle) < self.joint_pos_tolerance

        # Update internal state only if changed
        changed = False
        if self.agent_sensor_data['male_unlocked'].get(agent_id) != is_unlocked:
            self.agent_sensor_data['male_unlocked'][agent_id] = is_unlocked
            changed = True
        if self.agent_sensor_data['male_locked'].get(agent_id) != is_locked:
            self.agent_sensor_data['male_locked'][agent_id] = is_locked
            changed = True

        # if changed:
        #     self.get_logger().debug(f"Agent {agent_id} male joint state update: Unlocked={is_unlocked}, Locked={is_locked}")

# ==========================================================================
# Main Execution
# ==========================================================================
def main(args=None):
    rclpy.init(args=args)
    node = None # Initialize node to None
    try:
        # Use the updated class name
        node = Connect_Robots_Scalable_Simple()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node: node.get_logger().info("Ctrl-C detected, shutting down.")
    except Exception as e:
        # Log detailed exception info
        if node:
            tb_str = traceback.format_exc() # Get traceback string
            node.get_logger().fatal(f"Unhandled exception in node: {e}\nTraceback:\n{tb_str}")
        else:
            print(f"Exception during node initialization: {e}")
            traceback.print_exc()
    finally:
        # Ensure cleanup happens
        if node and rclpy.ok():
             node.get_logger().info("Stopping robots before shutdown...")
             node.stop_all_robots() # Use the generalized stop function
             node.destroy_node()
        # Ensure shutdown happens even if node creation failed or was interrupted early
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()