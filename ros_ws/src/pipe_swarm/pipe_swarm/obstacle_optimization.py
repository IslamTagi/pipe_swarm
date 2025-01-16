import numpy as np
import matplotlib.pyplot as plt

# define agent properties
l_agent = 2     # agent length
n_agents = 3    # number of agents

# define global reference frame
x_global = 0
y_global = 0
theta_global = 0

def get_coordinate_representation(l_agent, theta_np):
    # world position
    theta = []
    for i in range(len(theta_np)):
        theta.append(theta_np[i])
    theta.insert(0,theta_global)
    x = np.zeros(len(theta))
    y = np.zeros(len(theta))
    x[0] = x_global
    y[0] = y_global
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

def visualize_agent_configuration(l_agent, theta):
    x, y, endpoints, center_of_mass = get_coordinate_representation(l_agent, theta)

    # Add ground line
    plt.figure(figsize=(8, 6))
    plt.axhline(0, color='black', linestyle='--', label='Ground')
    
    # Plot configuration
    plt.plot(endpoints[0], endpoints[1], '-o', color='blue', markersize=8, linewidth=2, label='Link')
    plt.plot(center_of_mass[0], center_of_mass[1], '-x', color='red', markersize=8, linewidth=2, label='Centre of Mass')
    
    # Formatting
    plt.title("Snake Robot Configuration (Centered Links)", fontsize=14)
    plt.xlabel("Horizontal Position (m)", fontsize=12)
    plt.ylabel("Vertical Position (m)", fontsize=12)
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()

def inverse_kinematics(x_target, y_target, n_agents, l_agent, max_iterations=10000, learning_rate = 5, tolerance=5e-3):

    theta = np.zeros(n_agents)  # Start with all joint angles at 0 radians
    minimum_error = 10
    minimum_error_iteration = 0

    for iteration in range(max_iterations):
        # Compute current end-effector position
        x_, y_, endpoints, center_of_mass_ = get_coordinate_representation(l_agent, theta)
        x_curr = endpoints[0][-1]
        y_curr = endpoints[1][-1]

        # Compute the pos error between current and target positions
        p_error = np.array([x_target - x_curr, y_target - y_curr])
        p_error_normalized = np.linalg.norm(p_error) 

        # print(p_error_normalized)
        if(p_error_normalized < minimum_error):
            minimum_error = p_error_normalized
            minimum_error_iteration = iteration

        # Check if the normalized error is within tolerance
        if p_error_normalized <= tolerance:
            break
        else:
            # Compute the gradient (partial derivatives with respect to each θ)
            gradient = np.zeros(n_agents)

            for i in range(n_agents):
                partial_sum_x = 0  # Sum for ∂x_i/∂θ_i
                partial_sum_y = 0  # Sum for ∂y_i/∂θ_i
                for j in range(i+1):
                    partial_sum_x += -l_agent * np.sin(theta[j-1])
                    partial_sum_y += (l_agent * np.cos(theta[j-1]))

                # Compute the gradient for θ_i
                gradient[i] = (x_curr - x_target) * partial_sum_x + (y_curr - y_target) * partial_sum_y

            # Update the angles using gradient descent
            theta -= learning_rate * gradient
    
    success = False
    if p_error_normalized <= tolerance:
        print(f"Hooray! Solution found in {iteration} iterations!")
        success = True
    else:
        print("Maximum iterations reached without finding a solution.")
    print(f'Minimum error: {minimum_error} at iteration : {minimum_error_iteration}')
    
    return success, theta, minimum_error, minimum_error_iteration

success_, theta_solution, minimum_error_, minimum_error_iteration_ = inverse_kinematics(2, 4, n_agents, l_agent)
visualize_agent_configuration(l_agent, theta_solution)