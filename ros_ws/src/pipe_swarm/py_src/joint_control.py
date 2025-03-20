#!/usr/bin/env python3
import rclpy
import rclpy.duration
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from time import sleep
from builtin_interfaces.msg import Duration

class JointControlNode(Node):
    def __init__(self):
        super().__init__('joint_control_node')

        # Initilise joint position dictionary
        self.angle = -3.14159/2
        self.current_position = {}

        self.male_joint_subscriber = self.create_subscription(JointTrajectoryControllerState, '/agent_0/male_joint_trajectory_controller/state', self.joint_state_callback, 10)
        self.male_joint_publisher = self.create_publisher(JointTrajectory, '/agent_0/male_joint_trajectory_controller/joint_trajectory', 10)
        
        self.timer = self.create_timer(3.0, self.publish_male_joint)  # Publish every second

    def joint_state_callback(self, msg):
        # Extract actual and desired positions
        for i, name in enumerate(msg.joint_names):
            if len(msg.actual.positions) > i:
                self.current_position[name] = msg.actual.positions[i]

        # Print the joint positions
        if 'base_male_joint' in self.current_position:
            actual_pos = self.current_position['base_male_joint']
            self.get_logger().info(f'Actual position: {actual_pos:.4f} rad')

    def publish_male_joint(self):
        msg = JointTrajectory()
        msg.joint_names = ['base_male_joint']

        point = JointTrajectoryPoint()
        point.positions = [self.angle]  # 90 degrees in radians
        point.time_from_start = Duration(sec=3, nanosec=0)

        msg.points.append(point)

        self.male_joint_publisher.publish(msg)
        self.get_logger().info(f'Published command to rotate base_male_joint to {self.angle} radians.')

def main(args=None):
    rclpy.init(args=args)
    node = JointControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()