#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from functools import partial
from sensor_msgs.msg import Imu

# Code to move 1x Robot
class MyNode(Node):

    def __init__(self):
        super().__init__("move_robot_node")
        self.previous_x = 0.0
        self.last_cmd = None
        self.initialized = False

        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/agent_0/cmd_vel", 10)
        self.odometry_subscriber_ = self.create_subscription(Odometry, "/agent_0/odom", self.odometry_callback, 10)
        self.imu_subscriber_ = self.create_subscription(Imu, "/agent_0/imu")
        self.get_logger().info("Robot controller with imu has started")

    def send_velocity_command(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z

        if self.last_cmd != linear_x:
            self.cmd_vel_publisher_.publish(cmd)
            self.last_cmd = linear_x
            self.get_logger().info(f'Published command: linear_x = {linear_x}')

        
    def odometry_callback(self, msg: Odometry):
        cmd = Twist()
        position = msg.pose.pose.position
        x = position.x
        y = position.y
        z = position.z
        self.get_logger().info(f'Position -> x: {x:.2f}, y: {y:.2f}, z: {z:.2f}')

        if not self.initialized:
            self.initialized = True
            self.send_velocity_command(0.5, 0.0)
            self.get_logger().info("Initialized: Moving forward with intial velocity")
            return

        # Move forward until x >= 3
        if x >= 3 and self.previous_x < 3:
            self.previous_x = x
            self.send_velocity_command(-0.5, 0.0) 
            self.get_logger().info("Reached x = 5, switching to move backwards")

        # Move Backwards unitl x <= 0
        elif x <= 0:
            self.previous_x = x
            self.send_velocity_command(0.5, 0.0)
            self.get_logger().info("Reached x =0, switching to move forward")

        # Update previous_x to track progress
        else:
            self.previous_x = x
            

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()