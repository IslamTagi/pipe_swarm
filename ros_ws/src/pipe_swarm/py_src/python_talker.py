#!/usr/bin/env python3
import rclpy
from pipe_swarm.python_chatter import PyTalker

QUE_SIZE = 10

def main(args=None):
    rclpy.init()
    node = PyTalker()
    node.sendPythonMessage("Hello from Python")
    # rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()