import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from shapely.geometry import LineString, Polygon as ShapelyPolygon
from scipy.optimize import minimize

def get_combined_coordinates(x_coordinates, y_coordinates):
    if len(x_coordinates) != len(y_coordinates):
        return None
    coordinates = []
    for i in range(len(x_coordinates)):
        coordinates.append((float(x_coordinates[i]), float(y_coordinates[i])))
    return coordinates

def get_intersection_points(robot_links:LineString, obstacle_polygon:ShapelyPolygon,
                            tolerance=2e-5):

    def extract_coordinates(geometry):
        coords_x, coords_y = [], []

        if geometry.is_empty:
            return coords_x, coords_y

        if geometry.geom_type == "Point":
            coords_x.append(geometry.x)
            coords_y.append(geometry.y)

        elif geometry.geom_type == "MultiPoint":
            for point in geometry.geoms:
                coords_x.append(point.x)
                coords_y.append(point.y)
        
        elif geometry.geom_type == "LineString":
            x_vals, y_vals = geometry.xy
            coords_x.extend(x_vals)
            coords_y.extend(y_vals)
        
        elif geometry.geom_type == "MultiLineString":
            for line in geometry.geoms:
                x_vals, y_vals = line.xy
                coords_x.extend(x_vals)
                coords_y.extend(y_vals)

        return coords_x, coords_y

    intersection_points = [[],[]]
    touching_points = [[],[]]

    # Define a tolerance for "touch"
    upscaled_obstacle = obstacle_polygon.buffer(tolerance, cap_style="flat")
    downscaled_obstacle = obstacle_polygon.buffer(-tolerance, cap_style="flat")

    for link in robot_links:
        link : LineString

        intersection = link.intersection(obstacle_polygon)
        upscaled_intersection = link.intersection(upscaled_obstacle)
        downscaled_intersection = link.intersection(downscaled_obstacle)

        if link.touches(obstacle_polygon) or link.touches(upscaled_obstacle) or link.touches(downscaled_obstacle) or \
            link.equals(obstacle_polygon) or link.equals(upscaled_obstacle) or link.equals(downscaled_obstacle) or \
            \
            (link.disjoint(downscaled_obstacle) and link.intersects(obstacle_polygon) and not link.touches(obstacle_polygon)) or \
            (link.disjoint(obstacle_polygon) and link.intersects(upscaled_obstacle) and not link.touches(upscaled_obstacle))\
                :
            x_touch, y_touch = extract_coordinates(intersection)
            touching_points[0].extend(x_touch)
            touching_points[1].extend(y_touch)
            
            upscaled_x, upscaled_y = extract_coordinates(upscaled_intersection)
            touching_points[0].extend(upscaled_x)
            touching_points[1].extend(upscaled_y)

            downscaled_x, downscaled_y = extract_coordinates(downscaled_intersection)
            touching_points[0].extend(downscaled_x)
            touching_points[1].extend(downscaled_y)

        elif link.intersects(downscaled_obstacle) and not link.touches(downscaled_obstacle):
            x_intersection, y_intersection = extract_coordinates(intersection)
            intersection_points[0].extend(x_intersection)
            intersection_points[1].extend(y_intersection)

    return intersection_points, touching_points
class Obstacle():
    def __init__(self, x_coordinates, y_coordinates):
        self.coordinates = [x_coordinates, y_coordinates]

    def get_polygon(self, border_colour='pink', fill_colour='orange'):
        return Polygon(
            get_combined_coordinates(self.coordinates[0], self.coordinates[1]),
            closed=True,
            edgecolor=border_colour,
            facecolor=fill_colour,
            linewidth=2,
            alpha=0.8
        )
    
    def get_shapley_polygon(self):
        return ShapelyPolygon(get_combined_coordinates(self.coordinates[0], self.coordinates[1]))
    
    def get_combined_obstacle(self, obstacle_polygon:ShapelyPolygon):
        combined_coordinates = list(self.get_shapley_polygon().union(obstacle_polygon).exterior.coords)
        x_coordinates, y_coordinates = zip(*combined_coordinates)
        return x_coordinates, y_coordinates

class ModularConfiguration():

    def __init__(self, sigma_np, x_pos, l_agent):

        # global reference frame
        self.global_coordinates = (0, 0)
        self.theta_global = 0

        # modular robot parameters
        self.n_agents = len(sigma_np)
        self.l_agent = l_agent
        self.get_coordinate_representation(sigma_np, x_pos)

    def reset_modular_robot(self, sigma_np, x_pos):
        self.x_pos = x_pos

        self.sigma = []
        for i in range(len(sigma_np)):
            self.sigma.append(sigma_np[i])
        self.sigma.insert(0,self.theta_global)
        self.theta = np.zeros(len(self.sigma))
        self.x = np.zeros(len(self.sigma))
        self.y = np.zeros(len(self.sigma))
        self.x[0] = self.global_coordinates[0] + self.x_pos
        self.y[0] = self.global_coordinates[1]
        self.endpoints = [self.x.copy(), self.y.copy()]
        self.com = []

    def get_coordinate_representation(self, sigma_np, x_pos):
        self.reset_modular_robot(sigma_np, x_pos)

        # calculate co-ordinate representation skipping global coordinates
        for i in range(1, len(self.sigma)):
            theta_i = np.deg2rad(self.sigma[i]) + np.deg2rad(self.theta[i-1])
            theta_i_1 = np.deg2rad(self.theta[i-1])
            self.theta[i] = np.rad2deg(theta_i)

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
    
    def get_line_shape(self):
        endpoints = get_combined_coordinates(self.endpoints[0], self.endpoints[1])
        links = []
        for i in range(len(endpoints)-1):
            links.append(LineString([endpoints[i], endpoints[i+1]]))
        
        return links
    
    def visualize_agent_configuration(self, obstacle: Obstacle):
        if self.sigma is None:
            return
        
        # ignore global theta coordinate
        self.get_coordinate_representation(self.sigma[1:], self.x_pos)
        
        intersection_points, touching_points = get_intersection_points(self.get_line_shape(), 
                                                      obstacle.get_shapley_polygon())

        plt.figure(figsize=(8, 6))
        
        # Ground line
        plt.axhline(0, color='black', linestyle='--', label='Ground')
        
        # Links
        plt.plot(self.endpoints[0], self.endpoints[1], '-o', color='blue', markersize=8, linewidth=2, label='Link')
        
        # Center of Mass
        plt.scatter(self.x[1:], self.y[1:], marker='x', color='green', label='Link Centre of Mass')
        plt.plot(self.com[0], self.com[1], 'x', color='green', markersize=8, linewidth=2, label='Centre of Mass')
        
        # Obstacle
        plt.plot(obstacle.coordinates[0], obstacle.coordinates[1], '-s', color='red', markersize=8, linewidth=2, label='Obstacle')
        plt.gca().add_patch(obstacle.get_polygon())

        # Collision
        plt.plot(touching_points[0], touching_points[1], 'v', color='purple', markersize=10, label='Touching Point')
        plt.plot(intersection_points[0], intersection_points[1], 'o', color='purple', markersize=10, label='Intersection Point')

        # Formatting
        plt.title("Snake Robot Configuration (Centered Links)", fontsize=14)
        plt.xlabel("Horizontal Position (m)", fontsize=12)
        plt.ylabel("Vertical Position (m)", fontsize=12)
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.show()

class ModelPredictiveControl():

    def __init__(self, n_agents, l_agent, obstacle:Obstacle,
                 x_pos_min=-100, x_pos_max=100, sigma_min=-90, sigma_max=90):

        # defining model
        self.n_agents = n_agents

            # initial sigma0 guess -- control parameters
        self.sigma = np.zeros(n_agents) # TODO (IT): randomize based on n_agents --> self.sigma = np.random.uniform(low=sigma_min, high=sigma_max, size=n_agents)
        self.x_pos = 0
        self.sigma0 = [self.x_pos]
        self.sigma0.extend(self.sigma)
            
            # define thetha0 boundaries
        self.theta0_bounds = [(sigma_min, sigma_max) for _ in range(n_agents)] # sigma bounds
        self.theta0_bounds.insert(0, (x_pos_min, x_pos_max)) # x pos bounds
        
        self.model_config = ModularConfiguration(self.sigma, self.x_pos, l_agent)

          # objective function
        self.pos_desired = (0, 0)

        self.obstacle = obstacle

    def objective(self, sigma0):
        # Compute current end-effector position
        self.model_config.get_coordinate_representation(sigma0[1:], sigma0[0])
        x = self.model_config.endpoints[0][-1]
        y = self.model_config.endpoints[1][-1]
        error = np.sqrt((x - self.pos_desired[0])**2
                        + (y - self.pos_desired[1])**2)  # Minimize position error
        return error
    
    # Constraints
    def obstalce_collision_constraint(self, sigma0):
        self.model_config.get_coordinate_representation(sigma0[1:], sigma0[0])
        intersection_points, touching_points_ = get_intersection_points(self.model_config.get_line_shape(), 
                                                      self.obstacle.get_shapley_polygon())
        return -len(intersection_points[0]) # if any intersection points
    
    def inverse_kinematics_with_constraints(self, pos_desired,
                                        max_iter=10000, tolerance=2e-6):
        
        # objective function
        self.pos_desired = pos_desired
        
        constraints = [
            {'type': 'ineq', 'fun': self.obstalce_collision_constraint},
            # TODO (IT): implement com constraint
            # TODO (IT): implement torque constraint
            # TODO (IT): implement x position constraint to not start past obstacle
        ]

        # Solve the optimization problem
        result = minimize(
            self.objective, 
            self.sigma0, 
            bounds=self.theta0_bounds, 
            constraints=constraints,
            tol=tolerance,
            options={"maxiter": max_iter, "disp": True}
        )

        if result.success:
            print("Optimization successful!")
            print(f'Result sigma0: {result.x}')
            return result.x
        else:
            print("Optimization failed.")
            return None
    
l_agent = 2
n_agents = 3    # number of agents

# define obstacle
step_endpoints = ((2, 3, 3), (0, 2, 0))
step = Obstacle(step_endpoints[0], step_endpoints[1])

ground_endpoints = ((-5, -5, 5, 5), (0, -5, -5, 0))
ground = Obstacle(ground_endpoints[0], ground_endpoints[1])

obstacle_x, obstacle_y = ground.get_combined_obstacle(step.get_shapley_polygon())
obstacle = Obstacle(obstacle_x, obstacle_y)

mpc = ModelPredictiveControl(n_agents, l_agent, obstacle)
theta_solution = mpc.inverse_kinematics_with_constraints((3,0))
mpc.model_config.visualize_agent_configuration(obstacle)