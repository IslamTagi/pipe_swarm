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
    x = [x_global]
    y = [y_global]
    endpoints = [[x_global],[y_global]]
    theta.insert(0,theta_global)

    # calculate co-ordinate representation
    for i in range(len(theta) - 1):
        i+=1 #skip global reference link

        theta_i = np.deg2rad(theta[i])
        theta_i_1 = np.deg2rad(theta[i-1])
        
        if(1 == i):
            # for first link, no previous link length to take into account
            x_i = x[i-1] + (0.5 * l_agent * np.cos(theta_i))
            x.append(x_i)
            y_i = y[i-1] + (0.5 * l_agent * np.sin(theta_i))
            y.append(y_i)
        else:
            x_i = x[i-1] + (0.5 * l_agent * (np.cos(theta_i) + np.cos(theta_i_1)))
            x.append(x_i)
            y_i = y[i-1] + (0.5 * l_agent * (np.sin(theta_i) + np.sin(theta_i_1)))
            y.append(y_i)

        x_endpoint_i = endpoints[0][i-1] + (l_agent * np.cos(theta_i))
        y_endpoint_i = endpoints[1][i-1] + (l_agent * np.sin(theta_i))

        endpoints[0].append(x_endpoint_i)
        endpoints[1].append(y_endpoint_i)

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

visualize_agent_configuration(l_agent, [0, 0, 45])
