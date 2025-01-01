import rclpy
from rclpy.node import Node
from pipe_swarm_interfaces.srv import LinkAgents

class LinkService(Node):

    def __init__(self):
        super().__init__('link_service')
        self.srv = self.create_service(LinkAgents, 'link_agents', self.link_agents_callback)

    def link_agents_callback(self, request:LinkAgents.Request, response:LinkAgents.Response):
        response.message = f'linking: {request.parent_agent} and {request.child_agent} through {request.parent_link} and {request.child_link}'                                                   # CHANGE
        self.get_logger().info('Incoming request\nLink: %s' % (request.link))

        return response