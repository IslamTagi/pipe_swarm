#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from sensor_msgs.msg import Imu
import math
from nav_msgs.msg import Odometry

class MyNode(Node):

    def __init__(self):
        super().__init__("centre_robot_node")
        self.last_linear_x_cmd = None
        self.last_angular_z_cmd = None
        
        # Create publisher for velocity commands
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/agent_0/cmd_vel", 10)
        
        # Create subscriptions for ToF sensors
        # self.left_tof_subscriber_ = self.create_subscription(Range, "/agent_0/left_infrared_range", self.left_tof_callback, 10)
        # self.right_tof_subscriber_ = self.create_subscription(Range, "/agent_0/right_infrared_range", self.right_tof_callback, 10)
        self.imu_subscriber_ = self.create_subscription(Imu, "/agent_0/imu/out", self.imu_callback, 10)
        self.odometry_subscriber_ = self.create_subscription(Odometry, "/agent_0/odom", self.odometry_callback, 10) # Used for graph
        
        self.left_distance = None
        self.right_distance = None
        self.get_logger().info("Robot centre controller has started")

        # Timer to periodically publish adjust_position (every 0.1 seconds)
        self.create_timer(0.1, self.adjust_position)

    # def left_tof_callback(self, msg: Range):
    #     self.left_distance = msg.range

    # def right_tof_callback(self, msg: Range):
    #     self.right_distance = msg.range

    def odometry_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        self.x = position.x
        self.y = position.y
        self.z = position.z
    
    def imu_callback(self, msg: Imu):
        # Calculate pitch and roll from acclerometer data
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        self.pitch = math.degrees(math.atan2(ay, math.sqrt(ax**2 + az**2)))  # Roll calculation
        self.roll = math.degrees(math.atan2(-ax, math.sqrt(ay**2 + az**2)))  # Pitch calculation

    def adjust_position(self):
        k_p = -0.2 # Proportional gain

        if self.roll > 0.1 or self.roll < -0.1:
            angular_z = k_p * self.roll
        else:
            angular_z = 0.0

        print(f'roll: {self.roll} x: {self.x} y: {self.y} Angular velocity: {angular_z}')
        # Publish velocity command
        # cmd = Twist()
        linear_x = 0.1
        # cmd.linear.x = 0.4  # Move forward at a constant speed
        # cmd.angular.z = angular_z
        
        self.send_velocity_command(linear_x, angular_z)
        # self.get_logger().info(f'Published command: linear_x = {cmd.linear.x}, angular_z = {cmd.angular.z}')
    
    def send_velocity_command(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z

        if self.last_linear_x_cmd != linear_x or self.last_angular_z_cmd != angular_z:
            self.cmd_vel_publisher_.publish(cmd)
            self.last_linear_x_cmd = linear_x
            self.last_angular_z_cmd = angular_z
            # print(f'Published command: linear_x: {linear_x}, angular_z: {angular_z}')


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
