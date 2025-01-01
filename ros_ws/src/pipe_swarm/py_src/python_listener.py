#!/usr/bin/env python3
import rclpy
from pipe_swarm.python_chatter import PyListener

QUE_SIZE = 10

def main(args=None):
    rclpy.init()
    node = PyListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()