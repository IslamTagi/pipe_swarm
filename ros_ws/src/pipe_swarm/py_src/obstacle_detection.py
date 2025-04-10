#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy.time
from sensor_msgs.msg import Range
import time
# from sensor_msgs.msg import LaserScan
import numpy as np

class MyNode(Node):

    def __init__(self):
        super().__init__("move_robot_node")
        self.previous_x = 0.0
        self.last_cmd = None
        self.initialized = False
        self.initial_time = 0.0
        self.obstacle_detected = False
        self.obstacle_identified = False
        self.initial_scan_heights = []
        self.initial_scan_height = None
        self.previous_height = None
        self.initial_scan_horizontal_distances = []
        self.initial_scan_horizontal_distance = None
        self.outlierBuffer = []
        self.height_check = None
        self.length_check = 0.0
        self.current_distance = None
        self.x_coordinate = None
        self.y_coordinate = None
        self.obstacle_identifier_start_time = None
        self.bridge_stop_time = 0.0
        self.difference_threshold = 0.1  # 10% difference threshold
        self.obstacle_difference_threshold = 0.0001

        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/agent_0/cmd_vel", 10)
        self.odometry_subscriber_ = self.create_subscription(Odometry, "/agent_0/odom", self.odometry_callback, 10)
        self.lidar_subscriber_ = self.create_subscription(Range, "/agent_0/infrared_range", self.lidar_callback, 10)
        
        self.get_logger().info("Robot controller with LiDAR obstacle detection started")

    def send_velocity_command(self, linear_x, angular_z):
        cmd = Twist()
        if self.obstacle_detected:
            if self.x_coordinate == None:
                current_time = time.time()
                cmd.linear.x = 0.0
                self.cmd_vel_publisher_.publish(cmd)
                self.get_logger().info("Obstacle detected! Stopping robot.")
                time_taken = current_time - self.initial_time - self.bridge_stop_time
                distance = 0.2 * time_taken
                self.x_coordinate = (distance + self.outlierBuffer[3]) * 100
            else:
                cmd.linear.x = 0.02
                if self.obstacle_identifier_start_time == None:
                    self.obstacle_identifier_start_time = time.time()
                self.get_logger().info(f"x_coordinate = {self.x_coordinate}")
                if self.height_check is not None:
                    self.y_coordinate = (self.initial_scan_height - self.height_check) * 100
                    self.get_logger().info(str(self.y_coordinate))
            if (self.x_coordinate != None) and (self.y_coordinate != None):
                cmd.linear.x = 0.0
                current_time = time.time()
                obstacle_identifier_distance = 0.02 * (current_time - self.obstacle_identifier_start_time) * 100
                if self.current_distance == None:
                    self.current_distance = self.length_check + obstacle_identifier_distance
                self.get_logger().info("coordinates: " + str(self.x_coordinate) + ", " + str(self.y_coordinate))
                self.get_logger().info("pipe length: " + str(self.length_check))
                self.get_logger().info("current position: " + str(self.current_distance))
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
        # self.get_logger().info(f'Position -> x: {x:.2f}')

        if not self.initialized:
            self.initialized = True
            # self.send_velocity_command(0.2, 0.0)
            self.get_logger().info("Initialized")
            return

    def lidar_callback(self, msg: Range):
        cmd = Twist()
        height_measurement = math.sin((math.pi)/12) * msg.range
        horizontal_distance_measurement = math.cos((math.pi)/12) * msg.range

        if len(self.initial_scan_heights) <= 4:
            self.initial_scan_heights.append(height_measurement)
            if len(self.initial_scan_heights) == 5:
                total = 0
                for temp in self.initial_scan_heights:
                    total = total + temp
                average = total / 5
                self.initial_scan_height = average
                self.initial_time = time.time()
                self.send_velocity_command(0.2, 0.0)  # Start moving immediately
                self.get_logger().info("Initial LiDAR scan stored.")
            return
        if len(self.initial_scan_horizontal_distances) <= 4:
            self.initial_scan_horizontal_distances.append(horizontal_distance_measurement)
            if len(self.initial_scan_horizontal_distances) == 5:
                total = 0
                for temp in self.initial_scan_horizontal_distances:
                    total = total + temp
                average = total / 5
                self.initial_scan_horizontal_distance = average
                self.send_velocity_command(0.2, 0.0)  # Start moving immediately
                self.get_logger().info("Initial LiDAR scan stored.")
            return


        # robot_ranges = []
        # for i in range(len(msg.ranges)):
        #     if msg.ranges[i] > 0 and self.initial_scan_height[i] > 0:
        #         difference = abs(msg.ranges[i] - self.initial_scan_height[i]) / self.initial_scan_height[i]
        #         if difference > self.difference_threshold:
        #             robot_ranges.append(msg.ranges[i])
        
        difference = abs(height_measurement - self.initial_scan_height) / self.initial_scan_height
        if (difference > self.difference_threshold) and len(self.outlierBuffer) < 4:
            self.outlierBuffer.append(horizontal_distance_measurement)
        elif len(self.outlierBuffer) > 1 and self.obstacle_detected != True:
            for anomolies in range(len(self.outlierBuffer)):
                self.outlierBuffer.pop()
        try:
            if (self.outlierBuffer[0] > self.initial_scan_horizontal_distance) and self.length_check == 0.0:
                    current_time = time.time() - self.initial_time - 0.01
                    gap_distance = horizontal_distance_measurement - self.initial_scan_horizontal_distance
                    length_distance = ((0.2 * current_time) + (horizontal_distance_measurement - gap_distance))
                    self.length_check = length_distance * 100
        except IndexError:
            current_time = time.time()
        # if robot_ranges:
        #     robot_distance = np.mean(robot_ranges)
        #     self.obstacle_detected = True
        #     self.get_logger().info(f'Robot detected at {robot_distance:.2f}m, stopping.')
        # else:
        #     self.obstacle_detected = False

        if len(self.outlierBuffer) > 3:
            self.obstacle_detected = True
            self.get_logger().info(f'Obstacle detected, stopping.')
            obstacle_difference = abs(height_measurement - self.previous_height) / self.previous_height
            self.get_logger().info(str(obstacle_difference))
            if obstacle_difference < self.obstacle_difference_threshold:
                self.get_logger().info(str(height_measurement))
                self.height_check = height_measurement
        else:
            self.obstacle_detected = False
        
        self.previous_height = height_measurement
        self.send_velocity_command(self.last_cmd, 0.0)
        

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()