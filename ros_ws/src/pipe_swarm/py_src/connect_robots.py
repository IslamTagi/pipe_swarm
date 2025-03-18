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

class Connect_Robots(Node):

    def __init__(self):
        super().__init__("move_robot_node")

        # Inital pitch and roll
        self.pitch = 0.0 # Store pitch
        self.roll = 0.0 # Store roll

        # Initilise values
        self.last_cmd = None
        self.initial_scan = None
        self.robot_distance = None
        self.initialized = False
        self.obstacle_detected = False
        self.aligned = False

        # 10% difference threshold for Lidar
        self.difference_threshold = 0.1  

        # Publishers
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/agent_0/cmd_vel", 10)

        # Subscribers
        self.lidar_subscriber_ = None # Lidar initally inactive
        self.alignment_subscriber = None # Alignment initally inactive 
        self.imu_subscriber_ = self.create_subscription(Imu, "/agent_0/imu/out", self.imu_callback, 10) # Always active

        self.get_logger().info("Connecting robots started")

        # Main Script
        self.timer = self.create_timer(0.1, self.connect_robots)

    def connect_robots(self):
        if not self.initialized:
            self.initialized = True
            linear_x = 0.1
            self.send_velocity_command(linear_x, 0.0)
            print("Robot Initialized: Moving forward")
        else:
            self.centre_robot()
            self.lidar_subscriber_ = self.create_subscription(LaserScan, "/agent_0/laserscan", self.lidar_callback, 10)
            if self.obstacle_detected is True and self.robot_distance < 0.3:
                # Activate obstacle detection
                self.get_logger().info("Within 2 cm of obstacle")
                self.alignment_subscriber = self.create_subscription(Bool, "/agent_0/alignment", self.alignment_callback, 10)
                if self.aligned is True:
                    linear_x = 0.01
                    self.send_velocity_command(linear_x, 0.0)
                    # SO ROBOT ARE CONNECTED. CHECK WHETHER LOCK CAN LOCK IF DISTANCE IS SMALL ENOUGH.
                    # ADD LOCIG SO THAT IF THEY ARE NOT ALIGNED, TRIES AGAIN

    def centre_robot(self):
        k_p = 0.1 # Proportional gain

        if self.roll > 5 or self.roll < -5:
            angular_z = k_p * self.roll
            print(f'Angular velocity: {angular_z}')
        else:
            angular_z = 0.0
            self.get_logger().info("Robot centring not needed")
        
        self.send_velocity_command(self.last_cmd, angular_z)

    def send_velocity_command(self, linear_x, angular_z):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z

        if self.last_cmd != cmd.linear.x:
            self.cmd_vel_publisher_.publish(cmd)
            self.last_cmd = cmd.linear.x
            print(f'Published command: linear_x = {cmd.linear.x}, angular_z = {cmd.angular.z}')
    
    def imu_callback(self, msg: Imu):
        # Calculate pitch and roll from acclerometer data
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        self.roll = math.atan2(ay, az)  # Roll calculation
        self.pitch = math.atan2(-ax, math.sqrt(ay**2 + az**2))  # Pitch calculation

    def lidar_callback(self, msg: LaserScan):
        if not self.initial_scan:
            self.initial_scan = msg.ranges
            self.get_logger().info("Initial LiDAR scan stored.")
            return

        robot_ranges = []
        for i in range(len(msg.ranges)):
            if msg.ranges[i] > 0 and self.initial_scan[i] > 0:
                difference = abs(msg.ranges[i] - self.initial_scan[i]) / self.initial_scan[i]
                if difference > self.difference_threshold:
                    robot_ranges.append(msg.ranges[i])
        
        if robot_ranges:
            self.robot_distance = np.mean(robot_ranges)
            self.obstacle_detected = True
            self.get_logger().info(f"Obstacle Detected. Distance to object: {self.robot_distance}")
        else:
            self.obstacle_detected = False
    
    def alignment_callback(self, msg: Bool):
        if msg.data is True:
            self.get_logger().info("Aligned")
            self.aligned = True
        else:
            self.get_logger().info("Not aligned")
        

def main(args=None):
    rclpy.init(args=args)
    node = Connect_Robots()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
