#!/usr/bin/env python3
import rclpy
from pipe_swarm.obstacle_optimization import Obstacle, Pipe, ModelPredictiveControl
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray 

class GoalStateNode(Node):
    def __init__(self):
        super().__init__("pipe_swarm")
        self.get_logger().info("Goal State Solver Active")
        self.string_publisher = self.create_publisher(Float64MultiArray , "/modular_goal_state", 20)

    def send_goal_state(self, message):
        goal_state_msg = Float64MultiArray ()
        goal_state_msg.data = [float(x) for x in message]
        self.string_publisher.publish(goal_state_msg)
        self.get_logger().info(f"Sent Goal State: {goal_state_msg.data}")

QUE_SIZE = 10

# define agent
l_agent = 15 #cm
m_agent = 0.175 # kg
n_agents = 3    # number of agents

# define pipe
pipe_radius = 15
pipe_length = 60
pipe_thickness = 5

pipe = Pipe(pipe_length, pipe_radius, pipe_thickness, [-40,0])
obstacle = pipe.get_obstacle()

step_pipe = Pipe(pipe_length, pipe_radius, pipe_thickness, [20,5])
step_pipe_obstacle = step_pipe.get_obstacle()

obstacle_x, obstacle_y = obstacle.get_overall_obstacle(step_pipe_obstacle)
obstacle = Obstacle(obstacle_x, obstacle_y)

# step_endpoints = ((4, 3, 3), (0, 2, 0))
# step = Obstacle(step_endpoints[0], step_endpoints[1])

# gap_endpoints = ((0, 0, 2, 2), (0, -1, -1, 0))
# gap = Obstacle(gap_endpoints[0], gap_endpoints[1], 'gap')

# ground_endpoints = ((-2, -2, 5, 5), (0, -2, -2, 0))
# ground = Obstacle(ground_endpoints[0], ground_endpoints[1])

# obstacle_x, obstacle_y = ground.get_overall_obstacle(step)
# obstacle = Obstacle(obstacle_x, obstacle_y)

# obstacle_x, obstacle_y = obstacle.get_overall_obstacle(gap, 'gap')
# obstacle = Obstacle(obstacle_x, obstacle_y)

def main(args=None):

    rclpy.init()
    node = GoalStateNode()
    
    mpc = ModelPredictiveControl(n_agents, l_agent, m_agent, obstacle)
    
    theta_solution = mpc.inverse_kinematics_with_constraints((20, 20))
    node.get_logger().info(f'Goal Endpoints: {mpc.model_config.endpoints}')
    if theta_solution is not None:
        ros_solution = mpc.model_config.reformat_solution(theta_solution)
        node.send_goal_state(ros_solution)
        mpc.model_config.visualize_agent_configuration(obstacle)
    else:
        node.get_logger().warn("Failed to find goal state")
        mpc.model_config.visualize_agent_configuration(obstacle)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()