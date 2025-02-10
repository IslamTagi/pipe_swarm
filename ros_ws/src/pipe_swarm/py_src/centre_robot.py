#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class MyNode(Node):

    def __init__(self):
        super().__init__("centre_robot_node")
        self.last_linear_x_cmd = None
        self.last_angular_z_cmd = None
        
        # Create publisher for velocity commands
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/agent_0/cmd_vel", 10)
        
        # Create subscriptions for ToF sensors
        self.left_tof_subscriber_ = self.create_subscription(Range, "/agent_0/left_infrared_range", self.left_tof_callback, 10)
        self.right_tof_subscriber_ = self.create_subscription(Range, "/agent_0/right_infrared_range", self.right_tof_callback, 10)
        
        self.left_distance = None
        self.right_distance = None
        self.get_logger().info("Robot centre controller has started")

        # Timer to periodically publish adjust_position (every 0.1 seconds)
        self.create_timer(1.0, self.adjust_position)

    def left_tof_callback(self, msg: Range):
        self.left_distance = msg.range

    def right_tof_callback(self, msg: Range):
        self.right_distance = msg.range

    def adjust_position(self):
        print(f'Left Distance: {self.left_distance}, Right Distance: {self.right_distance}')
        
        if self.left_distance is None or self.right_distance is None:
            return
        
        # Compute the error (difference between left and right distances)
        error = self.left_distance - self.right_distance
        
        # Set a proportional gain
        k_p = 0.1  # Adjust this value based on how aggressively you want to correct
        angular_z = k_p * error  # Negative to turn towards the closer wall
        print(f'Angular Velocity: {angular_z}') # +ve value means moving left, -ve means moving right
        
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
            self.get_logger().info(f'Published command: linear_x = {linear_x}, angular_z = {angular_z}')


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
