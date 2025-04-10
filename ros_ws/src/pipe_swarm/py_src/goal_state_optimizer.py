#!/usr/bin/env python3
import rclpy
from pipe_swarm.obstacle_optimization import Obstacle, Pipe, ModelPredictiveControl
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray 

class GoalStateNode(Node):
    def __init__(self):
        super().__init__("pipe_swarm")
        self.get_logger().info("Goal State Solver Active")
        self.goal_state_publisher = self.create_publisher(Float64MultiArray , "/modular_goal_state", 20)
        self.pipe_pos_publisher = self.create_publisher(Float64MultiArray , "/define_pipe_positions", 20)

    def send_goal_state(self, message):
        goal_state_msg = Float64MultiArray ()
        goal_state_msg.data = [float(x) for x in message]
        self.goal_state_publisher.publish(goal_state_msg)
        self.get_logger().info(f"Sent Goal State: {goal_state_msg.data}")
    
    def send_pipe_pos(self, message):
        pipe_pos_msg = Float64MultiArray ()
        pipe_pos_msg.data = [float(x) for x in message]
        self.pipe_pos_publisher.publish(pipe_pos_msg)
        self.get_logger().info(f"Sent Pipe Pos: {pipe_pos_msg.data}")

QUE_SIZE = 10


# define agent
l_agent = 15 #cm
thickness_agent = 6.2 + 1.5 # base_h + wheel_r
m_agent = 0.175 # kg
n_agents = 3    # number of agents

# define pipe
pipe_1_x = 0 # tail
pipe_1_origin = [pipe_1_x, 0]
pipe_radius = 7.5
pipe_thickness = 5
pipe_length = n_agents*l_agent

pipe_2_x = pipe_length + pipe_1_x
pipe_2_origin = [pipe_length + pipe_1_x, 2.5]

pipe = Pipe(pipe_length, pipe_radius, pipe_thickness, pipe_1_origin)
obstacle = pipe.get_obstacle()

step_pipe = Pipe(pipe_length, pipe_radius, pipe_thickness, pipe_2_origin)
step_pipe_obstacle = step_pipe.get_obstacle()

obstacle_x, obstacle_y = obstacle.get_overall_obstacle(step_pipe_obstacle)
obstacle = Obstacle(obstacle_x, obstacle_y)


def main(args=None):

    rclpy.init()
    node = GoalStateNode()

    pipe_pos = [pipe_length, pipe_radius, pipe_thickness]
    pipe_pos.extend(pipe_1_origin) # x,y
    pipe_pos.extend(pipe_2_origin) # x,y
    node.send_pipe_pos(pipe_pos)
    
    mpc = ModelPredictiveControl(n_agents, l_agent, thickness_agent, m_agent, obstacle)
    theta_solution = mpc.inverse_kinematics_with_constraints((pipe_2_x+3, 3))

    node.get_logger().info(f'Goal Endpoints: {mpc.model_config.endpoints}')
    if theta_solution is not None:
        ros_solution = mpc.model_config.reformat_solution(theta_solution)
        node.send_goal_state(ros_solution)
        mpc.model_config.visualize_agent_configuration(obstacle)
    else:
        node.get_logger().warn("Failed to find goal state")
        mpc.model_config.visualize_agent_configuration(obstacle)
    # mpc.model_config.visualize_agent_configuration(obstacle)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()