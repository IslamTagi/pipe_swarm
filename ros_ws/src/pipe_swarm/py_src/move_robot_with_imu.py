#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from functools import partial
from sensor_msgs.msg import Imu
import math
from sensor_msgs.msg import Range

# Code to move 1x Robot
class MyNode(Node):

    def __init__(self):
        super().__init__("move_robot_node")
        self.previous_x = 0.0
        self.last_cmd = None
        self.initialized = False
        self.left_distance = None
        self.right_distance = None
        self.pitch = 0.0 # Store pitch
        self.roll = 0.0 # Store roll

        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/agent_0/cmd_vel", 10)
        self.odometry_subscriber_ = self.create_subscription(Odometry, "/agent_0/odom", self.odometry_callback, 10)
        self.imu_subscriber_ = self.create_subscription(Imu, "/agent_0/imu/out", self.imu_callback, 10)

        # Create subscriptions for ToF sensors
        self.left_tof_subscriber_ = self.create_subscription(Range, "/agent_0/left_infrared_range", self.left_tof_callback, 10)
        self.right_tof_subscriber_ = self.create_subscription(Range, "/agent_0/right_infrared_range", self.right_tof_callback, 10)
        
        self.get_logger().info("Robot controller with imu has started")

    def left_tof_callback(self, msg: Range):
        self.left_distance = msg.range

    def right_tof_callback(self, msg: Range):
        self.right_distance = msg.range
    
    def imu_callback(self, msg: Imu):
        # Calculate pitch and roll from acclerometer data
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        self.roll = math.atan2(ay, az)  # Roll calculation
        self.pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))  # Pitch calculation

    def send_velocity_command(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z

        if self.last_cmd != linear_x:
            self.cmd_vel_publisher_.publish(cmd)
            self.last_cmd = linear_x
            self.get_logger().info(f'Published command: linear_x = {linear_x}, roll = {math.degrees(self.roll):.2f}°, pitch = {math.degrees(self.pitch):.2f}°')
            print(f'Left Distance: {self.left_distance}, Right Distance: {self.right_distance}')
            self.get_logger().info(f'Position -> x: {self.x:.2f}, y: {self.y:.2f}, z: {self.z:.2f}')

        
    def odometry_callback(self, msg: Odometry):
        cmd = Twist()
        position = msg.pose.pose.position
        self.x = position.x
        self.y = position.y
        self.z = position.z

        if not self.initialized:
            self.initialized = True
            self.moving_forward = True
            self.send_velocity_command(0.1, 0.0)
            self.get_logger().info("Initialized: Moving forward with intial velocity")
            return

        # Move forward until x >= 3
        if self.x >= 1.0 and self.moving_forward:
            self.moving_forward = False
            self.send_velocity_command(-0.1, 0.0) 
            self.get_logger().info("Reached x = 5, switching to move backwards")

        # Move Backwards unitl x <= 0
        elif self.x <= 0 and not self.moving_forward:
            self.moving_forward = True
            self.send_velocity_command(0.1, 0.0)
            self.get_logger().info("Reached x = 0, switching to move forward")
            

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()