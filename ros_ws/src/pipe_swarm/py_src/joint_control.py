#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class JointControlNode(Node):
    def __init__(self):
        super().__init__('joint_control_node')
        self.publisher = self.create_publisher(Float64MultiArray, '/agent_0/position_controller/commands', 10)
        self.timer = self.create_timer(1.0, self.publish_command)  # Publish every second

    def publish_command(self):
        msg = Float64MultiArray()
        msg.data = [1.5708]  # 90 degrees in radians
        self.publisher.publish(msg)
        self.get_logger().info('Published command to rotate base_female_joint to 90 degrees.')

def main(args=None):
    rclpy.init(args=args)
    node = JointControlNode()
    rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()