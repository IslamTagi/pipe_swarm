#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
import cv2
import numpy as np
from cv_bridge import CvBridge

class RedDetector(Node):
    def __init__(self):
        super().__init__('red_detector')
        
        # Subscribe to camera feed
        self.subscription = self.create_subscription(
            Image,
            '/agent_0/camera_sensor/image_raw',
            self.image_callback,
            10)
        
        # Publisher for red detection result
        self.publisher = self.create_publisher(Bool, '/agent_0/alignment', 10)
        
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Convert to HSV and create a mask for red color
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define lower and upper bounds for red in HSV
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        
        # Combine both masks
        mask = mask1 + mask2
        
        # Check if red is detected
        red_detected = np.any(mask > 0)

        # Publish True if red is detected, False otherwise
        msg = Bool()
        msg.data = bool(red_detected)
        self.publisher.publish(msg)
        self.get_logger().info(f'Red detected: {red_detected}')

def main(args=None):
    rclpy.init(args=args)
    node = RedDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
