import numpy as np
import sys
import argparse
from geometry import *
from threejs_group import threejs_group

# Define command-line argument parsing for RRT inputs
parser = argparse.ArgumentParser(description='RRT Path Planning with Obstacles')
parser.add_argument('--start', nargs=3, type=float, help='Start state q0 = (x, y, theta)')
parser.add_argument('--goal', nargs=3, type=float, help='Goal state qg = (x, y, theta)')
parser.add_argument('--map', type=str, help='Map file name containing obstacles')
args = parser.parse_args()

def load_obstacles(filename):
    obstacles = []
    with open(filename, 'r') as f:
        for idx, line in enumerate(f):
            parts = line.strip().split()
            x, y, z, r = map(float, parts)
            # Provide a unique name for each sphere based on its index
            obstacle_name = f"sphere_{idx}"
            obstacles.append(sphere(obstacle_name, r, [x, y, z], [1, 0, 0, 0]))
    return obstacles

def visualize_obstacles(obstacles, output_file="../out/obstacles.html"):
    viz_out = threejs_group(js_dir="../js")
    for obstacle in obstacles:
        viz_out.add_obstacle(obstacle, "0xff0000")  # Assuming all obstacles are red for visualization
    viz_out.to_html(output_file)
    print(f"Obstacle map generated at {output_file}")

# Constants
L = 1.5  # Wheelbase of the car
dt = 0.1  # Time step
total_time = 10  # Duration of simulation

# Dynamical model
def car_dynamics(state, control):
    x, y, theta = state
    v, phi = control
    dx = v * np.cos(theta)
    dy = v * np.sin(theta)
    dtheta = v / L * np.tan(phi)
    return np.array([dx, dy, dtheta])


def euler_integration(state, control, dt):
    """
    Perform one step of Euler integration based on the vehicle's dynamics model.

    Parameters:
    - state: The current state of the vehicle as (x, y, theta).
    - control: The control input as (velocity v, steering angle phi).
    - dt: The time step for integration.

    Returns:
    - The updated state of the vehicle as (x, y, theta).
    """
    x, y, theta = state
    v, phi = control

    # Update based on the dynamics model
    dx = v * np.cos(theta)
    dy = v * np.sin(theta)
    dtheta = v / L * np.tan(phi)

    # Integrate to get the new state
    new_x = x + dx * dt
    new_y = y + dy * dt
    new_theta = theta + dtheta * dt

    # Ensure theta stays within [-pi, pi] range
    new_theta = (new_theta + np.pi) % (2 * np.pi) - np.pi

    return (new_x, new_y, new_theta)

def create_node(position, parent=None):
    return {"position": position, "parent": parent}

def rrt(start, goal, obstacles, iter_max=5000):
    start_node = create_node(start)
    goal_node = create_node(goal)
    tree = [start_node]  # Initialize the tree with the start node
    
    for _ in range(iter_max):
        # Randomly sample a point with bias towards the goal
        sampled_point = sample_point(goal_node["position"], goal_bias=0.1,
                                     x_limits=(-10, 10), y_limits=(-10, 10), theta_limits=(0, 2*np.pi))
        
        # Find the nearest node in the tree to the sampled point
        nearest_node = find_nearest(tree, sampled_point)
        
        # Attempt to steer from nearest node towards sampled point
        new_position = steer(nearest_node["position"], sampled_point, L=1.5, dt=0.1, max_step_size=1.0, max_steering_angle=np.pi/4)
        
        # Create a new node for the new position with nearest_node as its parent
        new_node = create_node(new_position, nearest_node)
        
        # Check for collisions along the path from nearest to new node
        if is_collision_free(nearest_node["position"], new_node["position"], obstacles):
            tree.append(new_node)  # Add the new node to the tree if the path is clear
            
            # Check if the goal is reached
            if is_goal_reached(new_node["position"], goal_node["position"]):
                print("Goal reached.")
                return extract_path(new_node)
    
    return []  # If the goal is not reached within the iteration limit

def sample_point(goal, goal_bias=0.1, x_limits=(-10, 10), y_limits=(-10, 10), theta_limits=(0, 2*np.pi)):
    """
    Sample a 2D point on the ground with a bias towards the goal and include sampling for orientation.

    Parameters:
    - goal: The goal state as (x, y, theta).
    - goal_bias: The probability of sampling the goal point directly.
    - x_limits, y_limits: The sampling limits for x and y dimensions.
    - theta_limits: The limits for the orientation angle theta.

    Returns:
    - A sampled point as (x, y, theta).
    """
    if np.random.rand() < goal_bias:
        # Return the goal with a probability of 'goal_bias'
        return goal
    else:
        # Randomly sample a point and orientation on the ground
        x = np.random.uniform(x_limits[0], x_limits[1])
        y = np.random.uniform(y_limits[0], y_limits[1])
        theta = np.random.uniform(theta_limits[0], theta_limits[1])
        return (x, y, theta)
    
def circular_distance(theta1, theta2):
    """
    Compute the minimum distance between two angles.
    """
    return min(abs(theta1 - theta2), 2*np.pi - abs(theta1 - theta2))

def find_nearest(tree, point):
    """
    Find the nearest node in the tree to the given point.

    Parameters:
    - tree: A list of nodes in the tree, where each node is a dictionary with keys "position" and "parent".
    - point: The sampled point as a tuple (x, y, theta).

    Returns:
    - The nearest node in the tree to the given point.
    """
    min_distance = float('inf')
    nearest_node = None
    for node in tree:
        # Extract position from the node dictionary
        node_position = node["position"]
        
        # Calculate weighted Euclidean distance for (x, y)
        spatial_distance = np.linalg.norm(np.array(node_position[:2]) - np.array(point[:2]))
        
        # Calculate circular distance for theta
        orientation_distance = circular_distance(node_position[2], point[2])
        
        # Weighting factor for orientation distance can be adjusted
        distance = spatial_distance + orientation_distance * 0.1  # Assuming orientation has less weight
        
        if distance < min_distance:
            min_distance = distance
            nearest_node = node
    return nearest_node

def check_collision(cube_center, cube_half_extents, sphere_center, sphere_radius):
    cube_center = np.array(cube_center)
    cube_half_extents = np.array(cube_half_extents)
    sphere_center = np.array(sphere_center)

    # Check for sphere-cube face collision
    for i in range(3):
        if abs(cube_center[i] - sphere_center[i]) > cube_half_extents[i] + sphere_radius:
            return False  # No collision

    # Check collision with cube edges
    closest_point = np.clip(sphere_center, cube_center - cube_half_extents, cube_center + cube_half_extents)
    if np.linalg.norm(sphere_center - closest_point) < sphere_radius:
        return True  # Collision detected

    # Sphere-Vertex Collision
    for i in range(2):
        for j in range(2):
            for k in range(2):
                vertex = cube_center + cube_half_extents * np.array([(-1) ** i, (-1) ** j, (-1) ** k])
                if np.linalg.norm(sphere_center - vertex) < sphere_radius:
                    return True  # Collision detected
    return False

def steer(nearest_node, sampled_point, L=1.5, dt=0.1, max_step_size=1.0, max_steering_angle=np.pi/4):
    """
    Steer from nearest node towards sampled point considering car's dynamics.

    Parameters:
    - nearest_node: The nearest node in the tree to the sampled point (x, y, theta).
    - sampled_point: The point to steer towards (x, y, _).
    - L: Wheelbase of the car.
    - dt: Time step for the simulation.
    - max_step_size: The maximum distance the car can travel in one step.
    - max_steering_angle: The maximum steering angle.

    Returns:
    - A new node (x, y, theta) resulting from steering towards the sampled point.
    """
    nx, ny, ntheta = nearest_node
    sx, sy, _ = sampled_point
    
    # Calculate desired angle to the sampled point
    desired_angle = np.arctan2(sy - ny, sx - nx)
    angle_diff = desired_angle - ntheta
    
    # Determine the steering angle to turn the car towards the desired angle
    # This simplification assumes instant steering towards the desired direction
    steering_angle = np.clip(angle_diff, -max_steering_angle, max_steering_angle)
    
    # Simulate car movement based on the steering angle and a fixed velocity
    v = max_step_size / dt  # Choose velocity to cover max_step_size in one time step
    dtheta = v / L * np.tan(steering_angle) * dt
    
    new_x = nx + v * np.cos(ntheta) * dt
    new_y = ny + v * np.sin(ntheta) * dt
    new_theta = ntheta + dtheta
    
    # Ensure new_theta is within [0, 2*pi]
    new_theta = (new_theta + 2*np.pi) % (2*np.pi)

    return (new_x, new_y, new_theta)

def is_collision_free(nearest_node, new_node, obstacles):
    """
    Check if the path from nearest_node to new_node is collision-free considering the car's 3D dimensions.
    
    Parameters:
    - nearest_node: The starting node of the path segment (x, y, theta).
    - new_node: The ending node of the path segment (x, y, theta).
    - obstacles: A list of spherical obstacles (instances of 'sphere').
    
    Returns:
    - True if the path is collision-free, False otherwise.
    """
    # Car's dimensions as half extents: [length/2, width/2, height/2]
    car_half_extents = np.array([1, 0.5, 0.5])

    # Incrementally check along the path from nearest_node to new_node
    dx = new_node[0] - nearest_node[0]
    dy = new_node[1] - nearest_node[1]
    steps = max(abs(dx), abs(dy), 1)  # Ensure at least one step
    
    dx /= steps  # Incremental step in x
    dy /= steps  # Incremental step in y
    
    for step in range(int(steps) + 1):
        # Intermediate position between nearest_node and new_node
        intermediate_x = nearest_node[0] + dx * step
        intermediate_y = nearest_node[1] + dy * step
        
        # Cube (car) center at the intermediate position, z fixed at 0.5
        cube_center = np.array([intermediate_x, intermediate_y, 0.5])
        
        for obstacle in obstacles:
            sphere_center = np.array(obstacle.position)
            sphere_radius = obstacle.radius
            
            # Check collision using the provided check_collision function
            if check_collision(cube_center, car_half_extents, sphere_center, sphere_radius):
                return False  # Collision detected with an obstacle

    return True  # Path is collision-free

def is_goal_reached(current_node, goal_node, translation_threshold=0.1, rotation_threshold=0.5):
    """
    Check if the current_node is within a specified vicinity of the goal_node.
    
    Parameters:
    - current_node: The current state of the robot as (x, y, theta).
    - goal_node: The goal state as (x, y, theta).
    - translation_threshold: The maximum allowed translation distance to consider the goal as reached.
    - rotation_threshold: The maximum allowed rotation difference in radians.
    
    Returns:
    - True if the goal is considered reached, False otherwise.
    """
    x_current, y_current, theta_current = current_node
    x_goal, y_goal, theta_goal = goal_node
    
    # Calculate the Euclidean distance between the current position and the goal position
    translation_distance = np.sqrt((x_current - x_goal) ** 2 + (y_current - y_goal) ** 2)
    
    # Calculate the minimum rotation difference accounting for the circular nature of angles
    rotation_difference = min(abs(theta_current - theta_goal), 2 * np.pi - abs(theta_current - theta_goal))
    
    # Check if both translation and rotation are within their respective thresholds
    if translation_distance <= translation_threshold and rotation_difference <= rotation_threshold:
        return True
    else:
        return False
    
def extract_path(goal_node):
    """
    Extracts the path from the start node to the goal node by backtracking from the goal.
    
    Parameters:
    - goal_node: The goal node from which to start backtracking.
    
    Returns:
    - A list of nodes representing the path from the start node to the goal node, inclusive.
    """
    path = []
    current_node = goal_node
    while current_node is not None:
        # Insert the current node at the beginning of the path list
        path.insert(0, current_node['position'])  # Assuming we only need the position for the final path
        # Move to the parent of the current node
        current_node = current_node['parent']
    
    return path

def visualize_rrt_trajectory(viz, rrt_path, obstacles):
    if not rrt_path:
        print("No valid path found from start to goal.")
        return

    # Prepare the animation data
    animation_data = []
    for t, point in enumerate(rrt_path):
        position = [point[0], point[1], 0.5]  # z=0.5 for flat surface movement
        quaternion = [1, 0, 0, 0]  # Simplified quaternion for no tilt/roll
        color = "0xff0000"  # Red color; adjust as needed
        state = [t * dt, position, quaternion, color]
        animation_data.append(state)

    # Create the car object with initial position and orientation
    car = box("car", 2, 1, 1, animation_data[0][1], animation_data[0][2])

    # Add the car's animation to the visualization
    viz.add_animation(car, animation_data)

    # Add obstacles to the visualization
    for i, obstacle in enumerate(obstacles):
        # Generate a valid name for the sphere based on its index
        ob_name = f"obstacle_{i}"
        ob_geom = sphere(ob_name, obstacle.radius, obstacle.position, [1, 0, 0, 0])
        viz.add_obstacle(ob_geom, "0x00ff00")  # Green for obstacles

def main():
    # Parse command-line arguments
    start = args.start
    goal = args.goal
    map_file = args.map

    # Load obstacles from map file
    obstacles = load_obstacles(map_file)

    # Visualize obstacles
    visualize_obstacles(obstacles)

    # Run RRT algorithm
    rrt_path = rrt(start, goal, obstacles)

    # Initialize visualization
    viz = threejs_group(canvas_width=800, canvas_height=600, js_dir="../js/")

    # Visualize the RRT path
    visualize_rrt_trajectory(viz, rrt_path, obstacles)

    # Output the HTML file for the visualization
    viz.to_html("rrt_path.html", "../out/")  # Adjust the path as necessary

if __name__ == "__main__":
    main()