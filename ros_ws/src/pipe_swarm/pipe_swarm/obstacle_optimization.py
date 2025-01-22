import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# define obstacle
obstacle_endpoints = ((2, 2, 4, 4), (0, 2, 2, 0))


class ModularConfiguration():

    def __init__(self, theta_np, x_pos, l_agent):

        # global reference frame
        self.global_coordinates = (0, 0)
        self.theta_global = 0

        # modular robot parameters
        self.n_agents = len(theta_np)
        self.l_agent = l_agent
        self.get_coordinate_representation(theta_np, x_pos)

    def reset_modular_robot(self, theta_np, x_pos):
        self.x_pos = x_pos
        self.theta = []
        for i in range(len(theta_np)):
            self.theta.append(theta_np[i])
        self.theta.insert(0,self.theta_global)
        self.x = np.zeros(len(self.theta))
        self.y = np.zeros(len(self.theta))
        self.x[0] = self.global_coordinates[0] + self.x_pos
        self.y[0] = self.global_coordinates[1]
        self.endpoints = [self.x.copy(), self.y.copy()]
        self.com = []

    def get_coordinate_representation(self, theta_np, x_pos):
        self.reset_modular_robot(theta_np, x_pos)

        # calculate co-ordinate representation skipping global coordinates
        for i in range(1, len(self.theta)):
            theta_i = np.deg2rad(self.theta[i])
            theta_i_1 = np.deg2rad(self.theta[i-1])
            
            if(1 == i):
                # for first link, no previous link length to take into account
                self.x[i] = self.x[i-1] + (0.5 * self.l_agent * np.cos(theta_i))
                self.y[i] = self.y[i-1] + (0.5 * self.l_agent * np.sin(theta_i))
            else:
                self.x[i] = self.x[i-1] + (0.5 * self.l_agent * (np.cos(theta_i) + np.cos(theta_i_1)))
                self.y[i] = self.y[i-1] + (0.5 * self.l_agent * (np.sin(theta_i) + np.sin(theta_i_1)))

            self.endpoints[0][i] = self.endpoints[0][i-1] + (self.l_agent * np.cos(theta_i)) #x
            self.endpoints[1][i] = self.endpoints[1][i-1] + (self.l_agent * np.sin(theta_i)) #y

        # get average x,y position without global position reference
        self.com = (np.mean(self.x[1:]), np.mean(self.y[1:]))

        return self.x, self.y, self.endpoints, self.com
    
    def visualize_agent_configuration(self, obstacle):
        if self.theta is None:
            return
        
        # ignore global theta coordinate
        self.get_coordinate_representation(self.theta[1:], self.x_pos)

        plt.figure(figsize=(8, 6))
        
        # Ground line
        plt.axhline(0, color='black', linestyle='--', label='Ground')
        
        # Links
        plt.plot(self.endpoints[0], self.endpoints[1], '-o', color='blue', markersize=8, linewidth=2, label='Link')
        
        # Center of Mass
        plt.scatter(self.x[1:], self.y[1:], marker='x', color='green', label='Link Centre of Mass')
        plt.plot(self.com[0], self.com[1], '-x', color='green', markersize=8, linewidth=2, label='Centre of Mass')
        
        # # Obstacle
        plt.plot(obstacle[0], obstacle[1], '-s', color='red', markersize=8, linewidth=2, label='Obstacle')
        plt.fill_between(obstacle[0], obstacle[1], color='orange')
        
        # Formatting
        plt.title("Snake Robot Configuration (Centered Links)", fontsize=14)
        plt.xlabel("Horizontal Position (m)", fontsize=12)
        plt.ylabel("Vertical Position (m)", fontsize=12)
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.show()

class ModelPredictiveControl():

    def __init__(self, n_agents, l_agent, 
                 x_pos_min=-100, x_pos_max=100, theta_min=-90, theta_max=90):

        # defining model
        self.n_agents = n_agents

            # initial theta0 guess -- control parameters
        self.theta = (10, 20, 50) # TODO (IT): randomize based on n_agents
        self.x_pos = 0
        self.theta0 = [self.x_pos]
        self.theta0.extend(self.theta)
            
            # define thetha0 boundaries
        self.theta0_bounds = [(theta_min, theta_max) for _ in range(n_agents)] # theta bounds
        self.theta0_bounds.insert(0, (x_pos_min, x_pos_max)) # x pos bounds
        
        self.model_config = ModularConfiguration(self.theta, self.x_pos, l_agent)

          # objective function
        self.pos_desired = (0, 0)

    def objective(self, theta0):
        # Compute current end-effector position
        self.model_config.get_coordinate_representation(theta0[1:], theta0[0])
        x = self.model_config.endpoints[0][-1]
        y = self.model_config.endpoints[1][-1]
        error = np.sqrt((x - self.pos_desired[0])**2
                        + (y - self.pos_desired[1])**2)  # Minimize position error
        return error
    
    # Constraints
    def ground_constraint(self, theta0):
        self.model_config.get_coordinate_representation(theta0[1:], theta0[0])
        return_val = 0
        for i in self.model_config.endpoints[1]:
            if i < 0:
                return_val = -1 # fail if any endpoint below ground
                # TODO (IT): make sure no length along link underground
                break
        return return_val  # endpoint > 0

    def inverse_kinematics_with_constraints(self, pos_desired, 
                                        max_iter=10000, tolerance=5e-6):
        
        # objective function
        self.pos_desired = pos_desired
        
        constraints = [
            {'type': 'ineq', 'fun': self.ground_constraint},
            # TODO (IT): implement com constraint
            # TODO (IT): implement obstacle constraint
            # TODO (IT): implement torque constraint
        ]

        # Solve the optimization problem
        result = minimize(
            self.objective, 
            self.theta0, 
            bounds=self.theta0_bounds, 
            constraints=constraints,
            tol=tolerance,
            options={"maxiter": max_iter, "disp": True}
        )

        if result.success:
            print("Optimization successful!")
            print(result.x)
            return result.x
        else:
            print("Optimization failed.")
            return None
    
l_agent = 2
n_agents = 3    # number of agents

mpc = ModelPredictiveControl(n_agents, l_agent)
theta_solution = mpc.inverse_kinematics_with_constraints((5, 3))
mpc.model_config.visualize_agent_configuration(obstacle_endpoints)
