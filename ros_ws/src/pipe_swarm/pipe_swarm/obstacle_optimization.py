import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from shapely.geometry import LineString, Polygon as ShapelyPolygon
from shapely.geometry import Polygon as ShapelyPolygon, MultiPolygon
from scipy.optimize import minimize
from shapely.ops import unary_union

def get_combined_coordinates(x_coordinates, y_coordinates):
    if len(x_coordinates) != len(y_coordinates):
        return None
    coordinates = []
    for x, y in zip(x_coordinates, y_coordinates):
        if x is not None and y is not None:
            coordinates.append((float(x), float(y)))
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

    def __init__(self, x_coordinates, y_coordinates, obstacle_type='solid'):
        self.coordinates = [x_coordinates, y_coordinates]
        self.type = obstacle_type

    # def get_polygon(self, border_colour='pink', fill_colour='orange'):
    #     return Polygon(
    #         get_combined_coordinates(self.coordinates[0], self.coordinates[1]),
    #         closed=True,
    #         edgecolor=border_colour,
    #         facecolor=fill_colour,
    #         linewidth=2,
    #         alpha=0.8
    #     )
    
    def get_polygon(self, border_colour='pink', fill_colour='orange'):
        patches = []
        x_coords, y_coords = self.coordinates
        current_polygon = []

        for x, y in zip(x_coords, y_coords):
            if x is None or y is None:
                if current_polygon:
                    polygon = Polygon(
                        current_polygon,
                        closed=True,
                        edgecolor=border_colour,
                        facecolor=fill_colour,
                        linewidth=2,
                        alpha=0.8
                    )
                    patches.append(polygon)
                    current_polygon = []
            else:
                current_polygon.append((x, y))

        # Add last polygon
        if current_polygon:
            polygon = Polygon(
                current_polygon,
                closed=True,
                edgecolor=border_colour,
                facecolor=fill_colour,
                linewidth=2,
                alpha=0.8
            )
            patches.append(polygon)

        return patches

    
    # def get_shapley_polygon(self):
    #     return ShapelyPolygon(get_combined_coordinates(self.coordinates[0], self.coordinates[1]))
    
    def get_shapley_polygon(self):
        # Interpret self.coordinates as potentially multiple polygons
        x_coords, y_coords = self.coordinates
        polygons = []
        current_polygon = []

        for x, y in zip(x_coords, y_coords):
            if x is None or y is None:
                if current_polygon:
                    polygons.append(ShapelyPolygon(current_polygon))
                    current_polygon = []
            else:
                current_polygon.append((x, y))

        # Add the last polygon if exists
        if current_polygon:
            polygons.append(ShapelyPolygon(current_polygon))

        # Return as MultiPolygon or Polygon
        if len(polygons) == 1:
            return polygons[0]
        elif len(polygons) > 1:
            return MultiPolygon(polygons)
        else:
            raise ValueError("No valid polygons found in coordinates.")

    
    # def get_combined_obstacle(self, obstacle_polygon:ShapelyPolygon):
    #     combined_coordinates = list(self.get_shapley_polygon().union(obstacle_polygon).exterior.coords)
    #     x_coordinates, y_coordinates = zip(*combined_coordinates)
    #     return x_coordinates, y_coordinates

    def get_combined_obstacle(self, obstacle_polygon:ShapelyPolygon):
        combined = self.get_shapley_polygon().union(obstacle_polygon)

        x_coordinates = []
        y_coordinates = []

        if combined.is_empty:
            return [], []

        if combined.geom_type == 'Polygon':
            coords = list(combined.exterior.coords)
            x_coordinates, y_coordinates = zip(*coords)

        elif combined.geom_type == 'MultiPolygon':
            for geom in combined.geoms:
                coords = list(geom.exterior.coords)
                xs, ys = zip(*coords)
                x_coordinates.extend(xs + (None,))  # None to separate shapes in plot
                y_coordinates.extend(ys + (None,))

        else:
            raise ValueError(f"Unexpected geometry type: {combined.geom_type}")

        return x_coordinates, y_coordinates

    
    def get_difference_obstacle(self, obstacle_polygon:ShapelyPolygon):
        combined_coordinates = list(self.get_shapley_polygon().difference(obstacle_polygon).exterior.coords)
        x_coordinates, y_coordinates = zip(*combined_coordinates)
        return x_coordinates, y_coordinates

    def get_overall_obstacle(self, obstacle, obstacle_type='solid'):
        if 'solid' == obstacle.type:
            return self.get_combined_obstacle(obstacle.get_shapley_polygon())
        elif 'gap' == obstacle.type:
            return self.get_difference_obstacle(obstacle.get_shapley_polygon())

class ModularConfiguration():

    def __init__(self, sigma_np, x_pos, l_agent, m_agent):

        # global reference frame
        self.global_coordinates = (0, 1e-6)
        self.theta_global = 0

        # modular robot parameters
        self.n_agents = len(sigma_np)
        self.l_agent = l_agent
        self.m_agent = m_agent
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
        # plt.gca().add_patch(obstacle.get_polygon())
        for patch in obstacle.get_polygon():
            plt.gca().add_patch(patch)


        # Collision
        plt.plot(touching_points[0], touching_points[1], 'v', color='purple', markersize=10, label='Touching Point')
        plt.plot(intersection_points[0], intersection_points[1], 'o', color='purple', markersize=10, label='Intersection Point')

        # Formatting
        plt.title("Snake Robot Configuration (Centered Links)", fontsize=14)
        plt.xlabel("Horizontal Position (cm)", fontsize=12)
        plt.ylabel("Vertical Position (cm)", fontsize=12)
        plt.axis('equal')
        plt.grid(True)
        plt.legend()
        plt.show()

class ModelPredictiveControl():

    def __init__(self, n_agents, l_agent, m_agent, obstacle:Obstacle,
                 x_pos_min=-100, x_pos_max=100, sigma_min=-45, sigma_max=45):

        # defining model
        self.n_agents = n_agents

            # initial sigma0 guess -- control parameters
        # self.sigma = np.zeros(n_agents) # TODO (IT): randomize based on n_agents --> self.sigma = np.random.uniform(low=sigma_min, high=sigma_max, size=n_agents)
        self.sigma = [0]
        self.sigma.extend(np.random.uniform(low=sigma_min, high=sigma_max, size=n_agents-1))
        print(f'Starting Seed: {self.sigma}')
        self.x_pos = 0
        self.sigma0 = [self.x_pos]
        self.sigma0.extend(self.sigma)
            
            # define thetha0 boundaries
        self.theta0_bounds = [(sigma_min, sigma_max) for _ in range(n_agents)] # sigma bounds
        self.theta0_bounds.insert(0, (x_pos_min, x_pos_max)) # x pos bounds
        
        self.model_config = ModularConfiguration(self.sigma, self.x_pos, l_agent, m_agent)

          # objective function
        self.pos_desired = (0, 0)

        self.obstacle = obstacle

    def get_grounded_robots(self, sigma0):
        self.model_config.get_coordinate_representation(sigma0[1:], sigma0[0])
        intersection_points_, touching_points = get_intersection_points(self.model_config.get_line_shape(), 
                                                      self.obstacle.get_shapley_polygon())

        if len(touching_points[0]) == 0:
            # No touches at all, return zeros per link
            return np.zeros(len(sigma0) - 1, dtype=int)

        touching_x = np.array(touching_points[0])  # shape: (P,)
        endpoints_x = np.array(self.model_config.endpoints[0])  # shape: (L+1,)

        link_starts = endpoints_x[:-1][:, None]  # shape: (L,1)
        link_ends = endpoints_x[1:][:, None]     # shape: (L,1)

        # Broadcast comparison, shape: (L, P)
        mask = (touching_x >= link_starts) & (touching_x <= link_ends)

        # Count touches per link
        grounded_touch = mask.sum(axis=1)
        # print(grounded_touch)
        return grounded_touch

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
    
    def grounded_angle_constraint(self, sigma0):
        return sigma0[1] # first link needs to be grounded
    
    def grounded_contact_constraint(self, sigma0):
        self.model_config.get_coordinate_representation(sigma0[1:], sigma0[0])
        intersection_points_, touching_points = get_intersection_points(self.model_config.get_line_shape(), 
                                                      self.obstacle.get_shapley_polygon())
        link_grounded = self.get_grounded_robots(sigma0)
        return link_grounded[0] - 2 # first link needs to be grounded (from both ends)
    
    def torque_constraint(self, sigma0):
        self.model_config.get_coordinate_representation(sigma0[1:], sigma0[0])
        link_grounded = self.get_grounded_robots(sigma0)
        grounded_boolean = link_grounded >= 2

        if not any(grounded_boolean):
            # No grounded robots at all
            return -1.0  # Violates constraint

        last_grounded_index = max(idx for idx, grounded in enumerate(grounded_boolean) if grounded)
        total_torque = 0.0
        link_mass = self.model_config.m_agent
        gravity = 9.81

        pivot_x = self.model_config.endpoints[0][last_grounded_index]

        for link in range(last_grounded_index, len(grounded_boolean)):
            # Centre of mass of the link
            com_x = self.model_config.x[link]
            distance = abs(com_x - pivot_x)
            torque = link_mass * gravity * (distance/10)
            total_torque += torque

        return 11 - total_torque  # total torque <= 11kg/cm
    
    def inverse_kinematics_with_constraints(self, pos_desired,
                                        max_iter=750, tolerance=2e-6):
        
        # objective function
        self.pos_desired = pos_desired
        
        constraints = [
            {'type': 'ineq', 'fun': self.obstalce_collision_constraint},
            {'type': 'ineq', 'fun': self.grounded_contact_constraint},
            {'type': 'ineq', 'fun': self.torque_constraint},
            {'type': 'eq', 'fun': self.grounded_angle_constraint},
            # TODO (IT): implement com constraint
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

        print(f'Result sigma0: {result.x}')
        if result.success:
            print("Optimization successful!")
            print(f'Starting Conditions: {self.sigma0}')
            return result.x
        else:
            print("Optimization failed.")
            return None

# define obstacle

class Pipe():
    def __init__(self, pipe_length, pipe_radius, pipe_thickness, coordinates):

        # defining model
        self.length = pipe_length
        self.radius = pipe_radius
        self.thickness = pipe_thickness
        self.coordinates = coordinates

        self.obstacle = self.initialize_obstacle()

    def initialize_obstacle(self):
        x = self.coordinates[0]
        y = self.coordinates[1]
        l = self.length
        thickness = self.thickness

        bottom_half_endpoints = ((x+l, x+l, x, x), (y, y-thickness, y-thickness, y))
        bottom_half = Obstacle(bottom_half_endpoints[0], bottom_half_endpoints[1])
        
        y += (2*self.radius)

        top_half_endpoints = ((x, x, x+l, x+l), (y+thickness, y, y, y+thickness))
        top_half = Obstacle(top_half_endpoints[0], top_half_endpoints[1])

        combined_shape = unary_union([top_half.get_shapley_polygon(), bottom_half.get_shapley_polygon()])

        if combined_shape.geom_type == 'Polygon':
            x_coords, y_coords = zip(*combined_shape.exterior.coords)
        elif combined_shape.geom_type == 'MultiPolygon':
            # Flatten all polygons
            x_coords, y_coords = [], []
            for geom in combined_shape.geoms:
                x, y = zip(*geom.exterior.coords)
                x_coords.extend(x + (None,))  # Add None for separator in plotting
                y_coords.extend(y + (None,))
        else:
            raise ValueError(f"Unexpected geometry type: {combined_shape.geom_type}")

        self.obstacle = Obstacle(x_coords, y_coords)

        return self.obstacle
    
    def get_obstacle(self):
        return self.obstacle

pipe_radius = 15
pipe_length = 60
pipe_thickness = 5

pipe = Pipe(pipe_length, pipe_radius, pipe_thickness, [-40,0])
obstacle = pipe.get_obstacle()

step_pipe = Pipe(pipe_length, pipe_radius, pipe_thickness, [20,2.5])
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

l_agent = 15 #cm
m_agent = 0.175 # kg
n_agents = 3    # number of agents

mpc = ModelPredictiveControl(n_agents, l_agent, m_agent, obstacle)
theta_solution = mpc.inverse_kinematics_with_constraints((30, 10))
print(mpc.model_config.endpoints)
mpc.model_config.visualize_agent_configuration(obstacle)

