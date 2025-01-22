import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import minimize

# define agent properties
l_agent = 2     # agent length
n_agents = 3    # number of agents

# define global reference frame
global_coordinates = (0, 0)
theta_global = 0

# define obstacle
obstacle_endpoints = ((2, 2, 4, 4), (0, 2, 2, 0))

def get_coordinate_representation(l_agent, theta_np, x_pos):
    # world position
    theta = []
    for i in range(len(theta_np)):
        theta.append(theta_np[i])
    theta.insert(0,theta_global)
    x = np.zeros(len(theta))
    y = np.zeros(len(theta))
    x[0] = x_pos
    y[0] = global_coordinates[1]
    endpoints = [x.copy(),y.copy()]

    # calculate co-ordinate representation skipping global coordinates
    for i in range(1, len(theta)):
        theta_i = np.deg2rad(theta[i])
        theta_i_1 = np.deg2rad(theta[i-1])
        
        if(1 == i):
            # for first link, no previous link length to take into account
            x[i] = x[i-1] + (0.5 * l_agent * np.cos(theta_i))
            y[i] = y[i-1] + (0.5 * l_agent * np.sin(theta_i))
        else:
            x[i] = x[i-1] + (0.5 * l_agent * (np.cos(theta_i) + np.cos(theta_i_1)))
            y[i] = y[i-1] + (0.5 * l_agent * (np.sin(theta_i) + np.sin(theta_i_1)))

        endpoints[0][i] = endpoints[0][i-1] + (l_agent * np.cos(theta_i)) #x
        endpoints[1][i] = endpoints[1][i-1] + (l_agent * np.sin(theta_i)) #y

    # get average x,y position without global position reference
    x_com = np.mean(x[1:])
    y_com = np.mean(y[1:])

    return x, y, endpoints, (x_com, y_com)

def visualize_agent_configuration(l_agent, theta, obstacle):
    if theta is None:
        return

    x, y, endpoints, center_of_mass = get_coordinate_representation(l_agent, theta[1:], theta[0])

    plt.figure(figsize=(8, 6))
    
    # Ground line
    plt.axhline(0, color='black', linestyle='--', label='Ground')
    
    # Links
    plt.plot(endpoints[0], endpoints[1], '-o', color='blue', markersize=8, linewidth=2, label='Link')
    
    # Center of Mass
    plt.scatter(x[1:], y[1:], marker='x', color='green', label='Link Centre of Mass')
    plt.plot(center_of_mass[0], center_of_mass[1], '-x', color='green', markersize=8, linewidth=2, label='Centre of Mass')
    
    # Obstacle
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

def inverse_kinematics_with_constraints(x_target, y_target, n_agents, l_agent, 
                                        theta_min=-90, theta_max=90, 
                                        max_iter=10000, tolerance=5e-3):
    
    def objective(theta):
        # Compute current end-effector position
        x_, y_, endpoints, com_ = get_coordinate_representation(l_agent, theta[1:], theta[0])
        x = endpoints[0][-1]
        y = endpoints[1][-1]
        error = np.sqrt((x - x_target)**2 + (y - y_target)**2)  # Minimize position error
        return error

    # Define bounds for joint angles
    bounds = [(theta_min, theta_max) for _ in range(n_agents+1)]
    
    def ground_constraint(theta):
        x_, y_, endpoints, com_ = get_coordinate_representation(l_agent, theta[1:], theta[0])
        return_val = 0
        for i in endpoints[1]:
            if i < 0:
                return_val = -1 # fail if any endpoint below ground
                # TODO (IT): make sure no length along link underground
                break
        return return_val  # endpoint > 0

    constraints = [
        {'type': 'ineq', 'fun': ground_constraint},
        # TODO (IT): implement com constraint
        # TODO (IT): implement obstacle constraint
    ]

    # Initial guess
    initial_theta = (0, 10,20,30)

    # Solve the optimization problem
    result = minimize(
        objective, 
        initial_theta, 
        bounds=bounds, 
        constraints=constraints,
        options={"maxiter": max_iter, "disp": True}
    )

    if result.success:
        print("Optimization successful!")
        print(result.x)
        return result.x  # Optimal joint angles
    else:
        print("Optimization failed.")
        return None

theta_solution = inverse_kinematics_with_constraints(5, 1, n_agents, l_agent)
visualize_agent_configuration(l_agent, theta_solution, obstacle_endpoints)
