#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
import numpy as np
from sensor_msgs.msg import Imu
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Range

class Connect_Robots(Node):

    def __init__(self):
        super().__init__("move_robot_node")

        # Inital pitch and roll
        self.pitch = 0.0 # Store pitch
        self.roll = 0.0 # Store roll
        self.count = 0.0 # Count to wait before engaging lock
        self.robot_detection_count = 0.0 # Count to wait before determining robot moved
        self.align_count = 0 # Count to wait before checking alignment the opposite side
        self.last_linear_cmd_agent_0 = 0.0
        self.last_linear_cmd_agent_1 = 0.0
        self.last_angular_cmd_agent_0 = 0.0
        self.last_angular_cmd_agent_1 = 0.0

        # Initilise values
        self.initial_scan = None
        self.robot_scan = None
        self.robot_distance = None
        self.range = None
        self.imu = None

        # True false statements
        self.delay_occured = False
        self.initialized = False
        self.sonething_detected = False
        self.obstacle_detected = False
        self.aligned = False
        self.robot_detected = False
        self.unlocked = False
        self.locked = False
        self.connected = False
        self.close = False

        # 10% difference threshold for Lidar and IMU
        self.difference_threshold = 0.2  

        # Publishers
        self.cmd_vel_publisher_agent_0_ = self.create_publisher(Twist, "/agent_0/cmd_vel", 10)
        self.cmd_vel_publisher_agent_1_ = self.create_publisher(Twist, "/agent_1/cmd_vel", 10)
        self.male_joint_publisher = self.create_publisher(JointTrajectory, '/agent_0/male_joint_trajectory_controller/joint_trajectory', 10)
        self.female_joint_publisher = self.create_publisher(JointTrajectory, '/agent_1/female_joint_trajectory_controller/joint_trajectory', 10)

        # Subscribers always active
        self.imu_subscriber_ = self.create_subscription(Imu, "/agent_0/imu/out", self.imu_callback, 10)
        self.front_tof_subscriber_ = self.create_subscription(Range, "/agent_0/infrared_range", self.infrared_range, 10)
        self.alignment_subscriber = self.create_subscription(Bool, "/agent_0/alignment", self.alignment_callback, 10)
        self.male_joint_subscriber = self.create_subscription(JointTrajectoryControllerState, '/agent_0/male_joint_trajectory_controller/controller_state', self.male_joint_state_callback, 10)

        print("Aligning robots started")

        # 2 second initial delay
        if self.delay_occured is False:
            self.initial_delay_timer = self.create_timer(2.0, self.initial_delay)

        # Main Script
        self.timer = self.create_timer(0.1, self.connect_robots)

    def initial_delay(self):
        if not self.delay_occured:
            print('Initial delay complete')
            self.delay_occured = True
            # self.initial_delay_timer.cancel()

    def connect_robots(self):
        if not self.delay_occured:
            return
        
        # If not moving forward, move forward and initilise IMU, male and female angles
        if not self.initialized:
            self.initialized = True
            linear_x = 0.1
            self.send_velocity_command_agent_0(linear_x, self.last_angular_cmd_agent_0)
            print("Robot Initialized: Moving forward")

            # If lock in locked position, unlock
            if not self.unlocked:
                self.lock = 0.0
                self.lock_male()

            # Take an initial reading of the IMU
            self.initial_imu = [self.ax, self.ay, self.az]

            # Reset female linkage
            self.female_angle = 0.0
            self.lift_female()

        else:
            # if robots are connected
            if self.connected is True:
                print('Lifting female joint')

                self.female_angle = 3.14159/4
                self.lift_female()
                return

            # centre robot
            self.centre_robot()
            
            # robot detection
            self.robot_detection()
            # Three exits to robot detection, either yes robot detected, only object detected or no object detected
            
            if self.robot_detected is True:
                # Stop if distance is less than 2 cm
                if self.robot_distance < 0.15:
                    self.close = True
                    print("Within 2 cm of obstacle")
                    # check alignment
                    if self.aligned is True:
                        print('Robots aligned')

                        # Reverse robot 1 into robot 0
                        linear_x = -0.02
                        self.send_velocity_command_agent_1(linear_x, self.last_angular_cmd_agent_1)
                        self.count += 1

                        # Wait 8 seconds to ensure robots are connected
                        if self.unlocked is True and self.count > 100:
                            print('Lock unlocked - locking')

                            # Initiate lock
                            self.lock = 3.14159/2
                            self.lock_male()

                            # Check whether lock is in locked position
                            if self.locked is True: 
                                print('Locked')

                                # Test whether robots are successfully locked together
                                linear_x = 0.02
                                self.send_velocity_command_agent_1(linear_x, self.last_angular_cmd_agent_1)

                                # Compare whether IMU values change
                                self.current_imu = [self.ax, self.ay, self.az]
                                for initial, current in zip(self.initial_imu, self.current_imu):
                                    imu_difference = abs(current - initial)/initial
                                    if imu_difference > 5 * self.difference_threshold:
                                        self.connected = True
                                        print('Connected')
                                        linear_x = 0.0
                                        self.send_velocity_command_agent_1(linear_x, self.last_angular_cmd_agent_1)
                                        self.send_velocity_command_agent_0(linear_x, self.last_angular_cmd_agent_0)
                                        self.delay_occured = False
                    
                    else:
                        # ROBOTS NOT ALIGNED FILL IN
                        # Stop whilst aligning
                        linear_x = 0.0
                        self.send_velocity_command_agent_0(linear_x, self.last_angular_cmd_agent_0)
                        self.send_velocity_command_agent_1(linear_x, self.last_angular_cmd_agent_1)

                        self.align_robots()
                        return
                else:
                    self.close = False
                    linear_x = 0.02
                    self.send_velocity_command_agent_0(linear_x, self.last_angular_cmd_agent_0)


    def align_robots(self):
        print(f'Not aligned, alignment count: {self.align_count}')
        self.align_count += 1
        if self.align_count <= 99:
            angular_z = 0.15
            self.send_velocity_command_agent_1(self.last_linear_cmd_agent_1, angular_z)
        elif self.align_count <= 300:
            angular_z = -0.15
            self.send_velocity_command_agent_1(self.last_linear_cmd_agent_1, angular_z)
        elif self.align_count > 300:
            self.align_count = 0
            print('Robot alignment failed')


    def centre_robot(self):
        k_p = -0.02 # Proportional gain

        if self.roll > 0.1 or self.roll < -0.1:
            angular_z = k_p * self.roll
        else:
            angular_z = 0.0
        
        # print(f'roll: {self.roll} Angular velocity: {angular_z}')
        self.send_velocity_command_agent_0(self.last_linear_cmd_agent_0, angular_z)

    def send_velocity_command_agent_0(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z

        if self.last_linear_cmd_agent_0 != cmd.linear.x or self.last_angular_cmd_agent_0 != cmd.angular.z:
            self.cmd_vel_publisher_agent_0_.publish(cmd)
            self.last_linear_cmd_agent_0 = cmd.linear.x
            self.last_angular_cmd_agent_0 = cmd.angular.z
            print(f'Published command to agent 0: linear_x = {cmd.linear.x}, angular_z = {cmd.angular.z}')
        
    def send_velocity_command_agent_1(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z

        if self.last_linear_cmd_agent_1 != cmd.linear.x or self.last_angular_cmd_agent_1 != cmd.angular.z:
            self.cmd_vel_publisher_agent_1_.publish(cmd)
            self.last_linear_cmd_agent_1 = cmd.linear.x
            self.last_angular_cmd_agent_1 = cmd.angular.z
            print(f'Published command to agent 1: linear_x = {cmd.linear.x}, angular_z = {cmd.angular.z}')    

    
    def imu_callback(self, msg: Imu):
        # Calculate pitch and roll from acclerometer data
        self.ax = msg.linear_acceleration.x
        self.ay = msg.linear_acceleration.y
        self.az = msg.linear_acceleration.z

        self.pitch = math.degrees(math.atan2(self.ay, math.sqrt(self.ax**2 + self.az**2)))  # Roll calculation
        self.roll = math.degrees(math.atan2(-self.ax, math.sqrt(self.ay**2 + self.az**2)))  # Pitch calculation
    
    def robot_detection(self):
        # Two exits to robot detection, either yes robot detected or only object detected
        
        # If robot has already been detected, skip
        if not self.robot_detected:
            # Take an initial scan of LiDAR
            if self.initial_scan is None:
                self.initial_scan = self.range
                print(f"Initial LiDAR scan stored: {self.initial_scan}")
                return

            # Calculate the range as the mean difference between the inital scan and the lidar scan. ANGLE NOT USED YET

            difference = abs(self.range - self.initial_scan) / self.initial_scan
            if difference > self.difference_threshold:
                print("Obstacle Detected")
                object_range = self.range
                self.sonething_detected = True

            if self.sonething_detected is True:
                print('Determining whether obstacle is a robot')

                # Stop robot when obstacle detected
                linear_x = 0.0
                self.send_velocity_command_agent_0(linear_x, self.last_angular_cmd_agent_0)

                # When object detected. Store scan of object
                if self.robot_scan is None:
                    self.robot_scan = self.range
                    print(f'Robot scan stored: {self.robot_scan}')
                    return

                # Move robot 1 forward
                linear_x = 0.1
                self.send_velocity_command_agent_1(linear_x, self.last_angular_cmd_agent_1)

                robot_difference = abs(self.range - self.robot_scan) / self.robot_scan
                if robot_difference > self.difference_threshold:
                    print('Robot detected')
                    self.robot_detected = True
                
                if self.robot_detected is True:
                    self.robot_distance = self.range

                    # Stop robot 1
                    linear_x = 0.0
                    self.send_velocity_command_agent_1(linear_x, self.last_angular_cmd_agent_1)

                    # Start robot 0
                    linear_x = 0.05
                    self.send_velocity_command_agent_0(linear_x, self.last_angular_cmd_agent_0)
                    # EXIT ONE: ROBOT DETECTED
                elif self.robot_detection_count > 5:
                    print('Robot not detected - obstacle detected')
                    self.obstacle_detected = True
                    # Thfis is triggering immediately. Create a count to wait to 2 loops before concluding this
                    linear_x = 0.0
                    self.send_velocity_command_agent_1(linear_x, self.last_angular_cmd_agent_1)
                    # EXIT TWO: OBJECT DETECTED

                self.robot_detection_count = self.robot_detection_count + 1
            
            else:
                self.obstacle_detected = False
                print('No obstacle detected')
                # EXIT THREE: NO OBJECT DETECTED
                return
        else:
            self.robot_distance = self.range
            return

    def infrared_range(self, msg: Range):
        # Two exits to robot detection, either yes robot detected or only object detected
        self.range = msg.range

    
    def alignment_callback(self, msg: Bool):
        if msg.data is True:
            self.aligned = True
        if msg.data is False:
            self.aligned = False

    def male_joint_state_callback(self, msg):
        # Input position
        self.reference_position = list(msg.reference.positions)
        
        # Actual position
        self.reference_feedback = list(msg.feedback.positions)

        # Output position
        self.output_postion = list(msg.output.positions)

        if len(self.reference_feedback) > 0:
            if -0.1 < self.reference_feedback[0]< 0.1:
                print('Unlocked')
                self.unlocked = True
            if 1.5 < self.reference_feedback[0] < 1.6:
                print('Locked')
                self.locked = True
    
    def lock_male(self):
        msg = JointTrajectory()
        msg.joint_names = ['base_male_joint']

        point = JointTrajectoryPoint()
        point.positions = [self.lock]
        point.time_from_start = Duration(sec=3, nanosec=0)

        msg.points.append(point)
        self.male_joint_publisher.publish(msg)

    def lift_female(self):
        msg = JointTrajectory()
        msg.joint_names = ['base_female_joint']

        point = JointTrajectoryPoint()
        point.positions = [self.female_angle]
        point.time_from_start = Duration(sec=10, nanosec=0)

        msg.points.append(point)
        self.female_joint_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Connect_Robots()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
