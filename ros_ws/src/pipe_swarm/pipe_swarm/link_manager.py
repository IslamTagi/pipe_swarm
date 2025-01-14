import rclpy
from rclpy.node import Node
from pipe_swarm_interfaces.srv import LinkAgents
from gazebo_msgs.srv import SpawnEntity#, DeleteEntity

class LinkService(Node):

    def __init__(self):
        super().__init__('link_service')
        self.service = self.create_service(LinkAgents, 'link_agents', self.handle_link_service)
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')

    async def handle_link_service(self, request:LinkAgents.Request, response:LinkAgents.Response):
        if(True == self.isHardware()):
            # TODO (IT): add implementation
            self.get_logger().error('No implementation for hardware handling...use service with simulation\n')
        else:
            if(True == request.link_agents):
                self.get_logger().info(f'Linking agents: {request.parent_agent} and {request.child_agent}...')
                success, message = await self.link_agents_gazebo(request)

            else:
                self.get_logger().info(f'Unlinking agents: {request.parent_agent} and {request.child_agent}...')
        
        response.success = success
        response.message = message
        return response
        
    def isHardware(self):
        return False
    
    async def link_agents_gazebo(self, request:LinkAgents.Request):
        joint_name = f"{request.parent_agent}_{request.child_agent}_joint"
        urdf = self.generate_joint_urdf(joint_name, request.parent_agent, request.parent_link, 
                                        request.child_agent, request.child_link,
                                        request.joint_type)

        print(urdf)

        # Wait for spawn_entity service to become available
        if not self.spawn_client.service_is_ready():
            await self.spawn_client.wait_for_service()

        # Create spawn request
        spawn_request = SpawnEntity.Request()
        spawn_request.name = 'dynamic_joint'
        spawn_request.xml = urdf
        spawn_request.robot_namespace = request.parent_agent
        spawn_response = await self.spawn_client.call_async(spawn_request)

        if spawn_response.success:
            return True, f"Joint {joint_name} created successfully."
        else:
            return False, f"Failed to create joint: {spawn_response.status_message}"
    
    def generate_joint_urdf(self, joint_name, parent_agent, parent_link, child_agent, child_link, joint_type):
        """Generate a URDF string for the dynamic joint."""
        return f"""
        <robot name="dynamic_joint">
          <link name="dummy_link"/>
          <joint name="{joint_name}" type="fixed">
            <origin xyz="0 0 0" rpy="0 0 0"/>
            <parent link="agent_0::agent_0_male_link" />
            <child link="agent_1::agent_1_female_link" />
          </joint>
        </robot>
        """