#!/usr/bin/env python3
import rclpy
from pipe_swarm.python_chatter import PyListener
from pipe_swarm.link_manager import LinkService
from pipe_swarm_interfaces.srv import LinkAgents

QUE_SIZE = 10

def main(args=None):
    rclpy.init(args=args)
    link_service = LinkService()
    rclpy.spin(link_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()