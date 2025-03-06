#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np

class MyNode(Node):

    def __init__(self):
        super().__init__("move_robot_node")
        self.previous_x = 0.0
        self.last_cmd = None
        self.initialized = False
        self.obstacle_detected = False
        self.initial_scan = None
        self.difference_threshold = 0.1  # 10% difference threshold

        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/agent_0/cmd_vel", 10)
        self.odometry_subscriber_ = self.create_subscription(Odometry, "/agent_0/odom", self.odometry_callback, 10)
        self.lidar_subscriber_ = self.create_subscription(LaserScan, "/agent_0/laserscan", self.lidar_callback, 10)
        
        self.get_logger().info("Robot controller with LiDAR obstacle detection started")

    def send_velocity_command(self, linear_x, angular_z):
        cmd = Twist()
        if self.obstacle_detected:
            cmd.linear.x = 0.0  # Stop when obstacle is detected
            self.get_logger().info("Obstacle detected! Stopping robot.")
        else:
            cmd.linear.x = linear_x
            cmd.angular.z = angular_z

        if self.last_cmd != cmd.linear.x:
            self.cmd_vel_publisher_.publish(cmd)
            self.last_cmd = cmd.linear.x
            self.get_logger().info(f'Published command: linear_x = {cmd.linear.x}')

    def odometry_callback(self, msg: Odometry):
        position = msg.pose.pose.position
        x = position.x
        self.get_logger().info(f'Position -> x: {x:.2f}')

        if not self.initialized:
            self.initialized = True
            self.send_velocity_command(0.5, 0.0)
            self.get_logger().info("Initialized: Moving forward")
            return

    def lidar_callback(self, msg: LaserScan):
        if self.initial_scan is None:
            self.initial_scan = msg.ranges
            self.send_velocity_command(0.5, 0.0)  # Start moving immediately
            self.get_logger().info("Initial LiDAR scan stored.")
            return

        robot_ranges = []
        for i in range(len(msg.ranges)):
            if msg.ranges[i] > 0 and self.initial_scan[i] > 0:
                difference = abs(msg.ranges[i] - self.initial_scan[i]) / self.initial_scan[i]
                if difference > self.difference_threshold:
                    robot_ranges.append(msg.ranges[i])
        
        if robot_ranges:
            robot_distance = np.mean(robot_ranges)
            self.obstacle_detected = True
            self.get_logger().info(f'Robot detected at {robot_distance:.2f}m, stopping.')
        else:
            self.obstacle_detected = False

        self.send_velocity_command(self.last_cmd, 0.0)
        

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
