#!/usr/bin/env python3
import rclpy
from pipe_swarm.obstacle_optimization import Obstacle, Pipe, ModelPredictiveControl
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Bool

class GoalStateNode(Node):
    def __init__(self):
        super().__init__("pipe_swarm")
        self.get_logger().info("Goal State Solver Active")
        self.goal_state_publisher = self.create_publisher(Float64MultiArray , "/modular_goal_state", 20)
        self.pipe_pos_publisher = self.create_publisher(Float64MultiArray , "/define_pipe_positions", 20)
        self.goal_plan_subscriber = self.create_subscription(Bool, "/plan_successful", self.get_plan_accepted, 20)
        self.got_plan_response = False
        self.plan_accepted = False

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
    
    def get_plan_accepted(self, message : Bool):
        self.got_plan_response = True
        self.plan_accepted = message.data
        print(self.plan_accepted)
        self.get_logger().info(f"Goal Plan Accepted: {self.plan_accepted}")

QUE_SIZE = 10

def get_robot_plan(x_starting_pos, obstacle, goal_endpoint, 
                   pipe_1_origin, pipe_2_origin):
    
    success = False
    num_agents_init = 3
    max_retries_of_n_agents = 15
    max_agents = 5

    theta_solution = None

    for num_agents in range(num_agents_init, max_agents+1):
        for i in range(1, max_retries_of_n_agents +1):
            
            print(f'\nIteration: {i} of {num_agents} agents')
            new_starting_pos = x_starting_pos-(num_agents-1)*l_agent # for 3 agents, start 2 agents behind origin
            print(f'New starting pos: {new_starting_pos}')
            
            mpc = ModelPredictiveControl(num_agents, l_agent, thickness_agent, m_agent, new_starting_pos, obstacle)
            theta_solution = mpc.inverse_kinematics_with_constraints(goal_endpoint)
            
            if theta_solution is not None:
                # if found a potential solution that is closer than 1cm
                if mpc.objective(theta_solution) > 1:
                    continue
                print(f"SUCCESS FOUND WITH: {theta_solution}")

                # send solution to ROS and await confirmation of planned state
                ros_solution = mpc.model_config.reformat_solution(theta_solution)
                node.send_goal_state(ros_solution)
                while not node.got_plan_response:
                    rclpy.spin_once(node, timeout_sec=0.001) # refresh for callback
                node.got_plan_response = False # clear for potential fail
                if(True == node.plan_accepted):
                    node.plan_accepted = False # reset
                    success = True
                    break
        
        if (True == success):
            break
        else:
            theta_solution = None # clear from any successes

    mpc.model_config.visualize_agent_configuration(obstacle)

    return num_agents, theta_solution
        


# rclpy.init()
node = GoalStateNode()

# define agent
l_agent = 15 #cm
thickness_agent = 6.2 + 1.5 # base_h + wheel_r
m_agent = 0.175 # kg


def main(args=None):

    # define agent

    # define pipe
    pipe_radius = 7.5
    pipe_thickness = 5
    pipe_length = 75
    
    pipe_1_origin = [-25, 0]
    pipe_2_origin = [70, 5]

    pipe = Pipe(pipe_length, pipe_radius, pipe_thickness, pipe_1_origin)
    obstacle = pipe.get_obstacle()

    step_pipe = Pipe(pipe_length, pipe_radius, pipe_thickness, pipe_2_origin)
    step_pipe_obstacle = step_pipe.get_obstacle()

    obstacle_x, obstacle_y = obstacle.get_overall_obstacle(step_pipe_obstacle)
    obstacle = Obstacle(obstacle_x, obstacle_y)

    pipe_pos = [pipe_length, pipe_radius, pipe_thickness]
    pipe_pos.extend(pipe_1_origin) # x,y
    pipe_pos.extend(pipe_2_origin) # x,y
    node.send_pipe_pos(pipe_pos)

    x_pos = 35

    n_agents, solution = get_robot_plan(x_pos, obstacle, (70, 5), pipe_1_origin, pipe_2_origin)
    if solution is not None:
        print(f"{n_agents} Agents Needed")
    else:
        print("NO SOLUTION FOUND")

    
    # mpc = ModelPredictiveControl(n_agents, l_agent, thickness_agent, m_agent, obstacle)
    # theta_solution = mpc.inverse_kinematics_with_constraints((pipe_2_x+3, 3))

    # node.get_logger().info(f'Goal Endpoints: {mpc.model_config.endpoints}')
    # if theta_solution is not None:
    #     ros_solution = mpc.model_config.reformat_solution(theta_solution)
    #     node.send_goal_state(ros_solution)
    #     mpc.model_config.visualize_agent_configuration(obstacle)
    # else:
    #     node.get_logger().warn("Failed to find goal state")
    #     mpc.model_config.visualize_agent_configuration(obstacle)
    # # mpc.model_config.visualize_agent_configuration(obstacle)
    
    # rclpy.shutdown()

if __name__ == '__main__':
    main()