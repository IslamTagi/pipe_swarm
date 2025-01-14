import numpy as np
import matplotlib.pyplot as plt

# define agent properties
l_agent = 2    # agent length

# define global reference frame
x_global = 0
y_global = 0
theta_global = 0

def get_coordinate_representation(l_agent, theta):
    # world position
    x = np.zeros(len(theta) + 1)
    y = np.zeros(len(theta) + 1)
    x[0] = x_global
    y[0] = y_global
    endpoints = [x.copy(),y.copy()]
    theta.insert(0,theta_global)

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

visualize_agent_configuration(l_agent, [0, 90, 0])
