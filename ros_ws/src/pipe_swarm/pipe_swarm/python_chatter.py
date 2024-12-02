#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PyTalker(Node):

    def __init__(self):
        super().__init__("py_talker")
        self.get_logger().info("Python talker active")
        self.string_publisher = self.create_publisher(String, "/py_string", 20)

    def sendPythonMessage(self, message):
        str_msg = String()
        str_msg.data = message
        self.string_publisher.publish(str_msg)
        self.get_logger().info("Py -> Py: "+str_msg.data)

class PyListener(Node):

    def __init__(self):
        super().__init__("py_listener")
        self.get_logger().info("Python listener Active")
        self.string_subscriber = self.create_subscription(String, "/py_string", self.receivePythonMessage, 20)

    def receivePythonMessage(self, message : String):
        self.get_logger().info("Py <- Py: "+message.data)
