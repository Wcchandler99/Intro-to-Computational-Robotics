import argparse
import numpy as np
from geometry import *
from threejs_group import *

def parse_arguments():
    parser = argparse.ArgumentParser(description='PRM Path Planning')
    parser.add_argument('--robot', type=str, required=True, choices=['arm', 'freeBody'], help='Defines the robot to use: "arm" or "freeBody".')
    parser.add_argument('--start', nargs='+', type=float, required=True, help='Defines the robot’s start configuration as a series of numbers.')
    parser.add_argument('--goal', nargs='+', type=float, required=True, help='Defines the robot’s goal configuration as a series of numbers.')
    parser.add_argument('--map', type=str, required=True, help='Filename that contains the map (obstacles).')
    return parser.parse_args()

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

def generate_random_quaternion():
    """
    Generates a random quaternion representing a rotation.
    Quaternions are represented as [w, x, y, z].
    """
    u1, u2, u3 = np.random.random(3)
    w = np.sqrt(1 - u1) * np.sin(2 * np.pi * u2)
    x = np.sqrt(1 - u1) * np.cos(2 * np.pi * u2)
    y = np.sqrt(u1) * np.sin(2 * np.pi * u3)
    z = np.sqrt(u1) * np.cos(2 * np.pi * u3)
    return [w, x, y, z]

def generate_random_configuration(robot_type):
    """
    Generates a random configuration based on the robot type.
    For the 'arm' type, generates random joint angles.
    For the 'freeBody' type, generates a random position and orientation (quaternion).
    """
    if robot_type == 'arm':
        # Generate random angles for each of the arm's 3 joints
        return np.random.uniform(0, 2*np.pi, 3)
    elif robot_type == 'freeBody':
        # Generate random position within specified bounds
        position = np.random.uniform(-10, 10, 3)  # Example bounds
        # Generate a random quaternion for orientation
        orientation = generate_random_quaternion()
        return np.concatenate((position, orientation))
    else:
        raise ValueError("Unsupported robot type")
    
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

def check_collision_with_obstacles(configuration, obstacles, robot_type, safety_margin=0.1):
    if robot_type == "arm":
        # Adjust the dimensions for each link to incorporate safety margins
        transformations = forward_kinematics(configuration)
        for index, (position, _) in enumerate(transformations):
            if index == 0:
                # Adjust dimensions for base with safety margins
                cube_half_extents = [1 + safety_margin, 1 + safety_margin, 0.25 + safety_margin]
            else:
                # Adjust dimensions for links with safety margins
                cube_half_extents = [0.5 + safety_margin, 0.5 + safety_margin, 2 + safety_margin]

            for obstacle in obstacles:
                # Enlarge obstacle radius by the safety margin for collision checking
                if check_collision(position, cube_half_extents, obstacle.position, obstacle.radius + safety_margin):
                    return True  # Collision detected

    elif robot_type == "freeBody":
        # For the freeBody robot, treated as a cuboid with dimensions 2x1x1
        # Adjust the cuboid's half extents to include the safety margin
        cube_half_extents = [1 + safety_margin, 0.5 + safety_margin, 0.5 + safety_margin]
        position = configuration[:3]  # Extract the position part of the freeBody configuration

        for obstacle in obstacles:
            # Enlarge obstacle radius by the safety margin for collision checking
            if check_collision(position, cube_half_extents, obstacle.position, obstacle.radius + safety_margin):
                return True  # Collision detected

    return False

def quaternion_distance(q1, q2):
    norm_q1 = np.linalg.norm(q1)
    norm_q2 = np.linalg.norm(q2)
    if norm_q1 > 1e-8 and norm_q2 > 1e-8:  # Use a small threshold to avoid division by numbers close to zero
        q1n = q1 / norm_q1
        q2n = q2 / norm_q2
        dot_product = np.dot(q1n, q2n)
        dot_product = np.clip(dot_product, -1.0, 1.0)  # Ensure dot product is within valid range for arccos
        return 2 * np.arccos(abs(dot_product))
    else:
        # Handle the case where one or both quaternions are zero or near-zero
        return 0 if np.array_equal(q1, q2) else np.pi  # Max distance if not equal, else 0

# Function to create a quaternion from Y-axis rotation
def quaternion_from_angle_y(theta):
    return [np.cos(theta/2), 0, np.sin(theta/2), 0]

# Function to create a quaternion from Z-axis rotation
def quaternion_from_angle_z(theta):
    return [np.cos(theta/2), 0, 0, np.sin(theta/2)]

# Function to perform quaternion multiplication
def quaternion_multiply(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 + y1*w2 + z1*x2 - x1*z2
    z = w1*z2 + z1*w2 + x1*y2 - y1*x2
    return [w, x, y, z]

# Function to convert quaternion to transformation matrix
def quaternion_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*z*w, 2*x*z + 2*y*w],
        [2*x*y + 2*z*w, 1 - 2*x*x - 2*z*z, 2*y*z - 2*x*w],
        [2*x*z - 2*y*w, 2*y*z + 2*x*w, 1 - 2*x*x - 2*y*y]])

# Function to apply a transformation to a point
def apply_transform(position, quaternion, point):
    rot_matrix = quaternion_to_matrix(quaternion)
    transformed_point = np.dot(rot_matrix, point) + position
    return transformed_point.tolist()

# Forward kinematics calculation
def forward_kinematics(configuration):
    theta1, theta2, theta3 = configuration

    # Link dimensions (length, width, height)
    base_dim = [2, 2, 0.5]  # For the base
    link1_dim = [1, 1, 4]    # For Link 1
    link2_dim = [1, 1, 4]    # For Link 2

    # Base link position and orientation
    base_position = [0, 0, base_dim[2] / 2]  # Base is static and located at {J0}
    base_orientation = quaternion_from_angle_z(theta1)

    # Position and orientation for Link 1
    # Since {J1} is at the end of the base, it's position would be at the top of the base link
    j1_position = [0, 0, base_dim[2]]
    # Orientation of Link 1 is the base orientation combined with its own rotation around Y-axis
    link1_orientation = quaternion_multiply(base_orientation, quaternion_from_angle_y(theta2))
    # Position of Link 1 is at {J1} plus half of its length along the Z-axis after rotation
    link1_position = apply_transform(j1_position, link1_orientation, [0, 0, link1_dim[2] / 2])

    # Position and orientation for Link 2
    # Since {J2} is at the end of Link 1, it's position would be at the top of Link 1
    j2_position = apply_transform(j1_position, link1_orientation, [0, 0, link1_dim[2]])
    # Orientation of Link 2 is the orientation of Link 1 combined with its own rotation around Y-axis
    link2_orientation = quaternion_multiply(link1_orientation, quaternion_from_angle_y(theta3))
    # Position of Link 2 is at {J2} plus half of its length along the Z-axis after rotation
    link2_position = apply_transform(j2_position, link2_orientation, [0, 0, link2_dim[2] / 2])

    # The transformations are the positions and orientations of each link
    transformations = [
        (base_position, base_orientation),  # Transformation for the base
        (link1_position, link1_orientation),  # Transformation for Link 1
        (link2_position, link2_orientation),  # Transformation for Link 2
    ]

    return transformations

def generate_prm_nodes(robot_type, num_nodes, obstacles):
    nodes = []
    while len(nodes) < num_nodes:
        # Generate a random configuration based on the robot type
        node = generate_random_configuration(robot_type)
        
        # Check if the generated configuration collides with any obstacle
        if not check_collision_with_obstacles(node, obstacles, robot_type):
            nodes.append(node)
    return nodes

def distance_metric(node_a, node_b, robot_type):
    if robot_type == 'arm':
        # Euclidean distance for arm robot configurations
        return np.linalg.norm(np.array(node_a) - np.array(node_b))
    elif robot_type == 'freeBody':
        # Combined distance for freeBody robot configurations
        pos_a, quat_a = node_a[:3], node_a[3:]
        pos_b, quat_b = node_b[:3], node_b[3:]
        pos_dist = np.linalg.norm(np.array(pos_a) - np.array(pos_b))
        quat_dist = quaternion_distance(np.array(quat_a), np.array(quat_b))
        # Weight the orientation distance to adjust its impact
        return pos_dist + quat_dist  # Adjust this sum as needed for balance

def connection_radius(num_nodes, dimension, gamma=7.0, eta=float('inf')):
    return min(gamma * (np.log(num_nodes) / num_nodes) ** (1 / dimension), eta)

def construct_prm_star_graph(nodes, obstacles, robot_type, gamma=7.0, eta=float('inf')):
    graph = {'nodes': nodes, 'edges': []}
    dimension = 3 if robot_type == 'arm' else 7  # Dimension of configuration space
    radius = connection_radius(len(nodes), dimension, gamma, eta)
    print("Connection Radius",radius)
    for i, node in enumerate(nodes):
        for j, other_node in enumerate(nodes):
            if i == j:
                continue  # Skip self
            distance = distance_metric(node, other_node, robot_type)
            if distance <= radius:
                if is_path_collision_free(node, other_node, obstacles, robot_type):
                    graph['edges'].append((i, j))

    return graph


def is_path_collision_free(start_config, end_config, obstacles, robot_type):
    # This function checks if the straight-line path between start_config and end_config is collision-free
    # For simplicity, this example linearly interpolates between start and end configurations
    steps = 20  # Number of steps to interpolate between the start and end configurations
    for step in range(1, steps + 1):
        t = step / steps
        interpolated_config = start_config + t * (end_config - start_config)
        if check_collision_with_obstacles(interpolated_config, obstacles, robot_type):
            return False  # Collision detected along the path
    return True

import heapq

def heuristic(node_a, node_b, robot_type):
    """
    Calculates the heuristic between two nodes, differently for arm and freeBody types.
    """
    if robot_type == 'arm':
        # Use Euclidean distance for arm
        return np.linalg.norm(np.array(node_a) - np.array(node_b))
    elif robot_type == 'freeBody':
        # Use a combination of Euclidean distance for position and quaternion distance for orientation
        pos_a, quat_a = node_a[:3], node_a[3:]
        pos_b, quat_b = node_b[:3], node_b[3:]
        pos_dist = np.linalg.norm(np.array(pos_a) - np.array(pos_b))
        quat_dist = quaternion_distance(np.array(quat_a), np.array(quat_b))
        scaled_quat_dist = quat_dist * 1  # Example scaling factor; adjust as needed
        return pos_dist + scaled_quat_dist
    else:
        raise ValueError("Unsupported robot type for heuristic calculation.")


def find_path_prm_star(start_config, goal_config, graph, robot_type):
    """Finds a path from start_config to goal_config using the A* algorithm."""
    # Convert configurations to nodes in the graph
    start_node = nearest_node(start_config, graph['nodes'], robot_type)
    goal_node = nearest_node(goal_config, graph['nodes'], robot_type)
    
    open_set = []
    heapq.heappush(open_set, (0 + heuristic(graph['nodes'][start_node], graph['nodes'][goal_node], robot_type), start_node))
    
    came_from = {}
    g_score = {node: float('inf') for node in range(len(graph['nodes']))}
    g_score[start_node] = 0
    
    f_score = {node: float('inf') for node in range(len(graph['nodes']))}
    f_score[start_node] = heuristic(graph['nodes'][start_node], graph['nodes'][goal_node], robot_type)
    
    while open_set:
        _, current = heapq.heappop(open_set)
        
        if current == goal_node:
            return reconstruct_path(came_from, current, graph['nodes'])
        
        for neighbor in get_neighbors(current, graph['edges']):
            tentative_g_score = g_score[current] + heuristic(graph['nodes'][current], graph['nodes'][neighbor], robot_type)
            
            if tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = tentative_g_score + heuristic(graph['nodes'][neighbor], graph['nodes'][goal_node], robot_type)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
                
    return []  # Return empty path if no path is found

def nearest_node(config, nodes, robot_type):
    """Finds the nearest node in the graph to the given configuration."""
    nearest_index, min_dist = None, float('inf')
    for i, node in enumerate(nodes):
        if robot_type == 'arm':
            dist = np.linalg.norm(np.array(config) - np.array(node))
        elif robot_type == 'freeBody':
            # Split configuration into position and quaternion
            pos1, quat1 = config[:3], config[3:]
            pos2, quat2 = node[:3], node[3:]
            # Calculate Euclidean distance for position
            dist_pos = np.linalg.norm(np.array(pos1) - np.array(pos2))
            # Calculate quaternion distance for orientation
            dist_quat = quaternion_distance(np.array(quat1), np.array(quat2))
            # Combine distances (simple sum in this example)
            dist = dist_pos + dist_quat
        if dist < min_dist:
            nearest_index, min_dist = i, dist
    return nearest_index

def get_neighbors(node, edges):
    """Finds the neighbors of a node in the graph based on edges."""
    neighbors = []
    for edge in edges:
        if edge[0] == node:
            neighbors.append(edge[1])
        elif edge[1] == node:
            neighbors.append(edge[0])
    return neighbors

def reconstruct_path(came_from, current, nodes):
    """Reconstructs the path from the start node to the goal node."""
    total_path = [nodes[current]]
    while current in came_from:
        current = came_from[current]
        total_path.insert(0, nodes[current])
    return total_path

def visualize_arm_movement(path, obstacles):
    viz_group = threejs_group(js_dir="../js")

    # Define colors for different links
    base_color = "0x0000ff"  # Blue for the base
    link1_color = "0x00ff00"  # Green for link 1
    link2_color = "0xff0000"  # Red for link 2

    # Initialize the boxes for each link with their colors
    boxes = {
        'link0': box("base", 2, 2, 0.5, [0, 0, 0], quaternion_from_angle_z(0)),
        'link1': box("link1", 1, 1, 4, [0, 0, 0], quaternion_from_angle_y(0)),
        'link2': box("link2", 1, 1, 4, [0, 0, 0], quaternion_from_angle_y(0))
    }
    link_colors = {'link0': base_color, 'link1': link1_color, 'link2': link2_color}

    # Create the keyframe data for the animation
    keyframes = {name: [] for name in boxes}
    for t, config in enumerate(path):
        transformations = forward_kinematics(config)
        for i, (position, quaternion) in enumerate(transformations):
            keyframes[f'link{i}'].append({'time': t, 'position': position, 'quaternion': quaternion})

    # Add animations for each link using the keyframe data
    for name, keyframe_data in keyframes.items():
        animation_data = [(kf['time'], kf['position'], kf['quaternion'], link_colors[name]) for kf in keyframe_data]
        viz_group.add_animation(boxes[name], animation_data)

    # Add obstacles to the visualization
    for i, obstacle in enumerate(obstacles):
        # Generate a valid name for the sphere based on its index
        ob_name = f"obstacle_{i}"
        ob_geom = sphere(ob_name, obstacle.radius, obstacle.position, [1, 0, 0, 0])
        viz_group.add_obstacle(ob_geom, "0x00ff00")  # Green for obstacles

    # Generate HTML to visualize the robot arm with animation
    viz_group.to_html("prm_star_arm_movement_animation.html", "../out/")

def visualize_freeBody_path(path, obstacles):
    """
    Visualizes the robot's path (including position and orientation) and the obstacles in the scene.
    
    :param path: A list of configurations where each configuration is [x, y, z, qw, qx, qy, qz],
                 representing the robot's position and orientation.
    :param obstacles: The scene containing obstacles, expected to be a list of sphere objects.
    """
    viz_out = threejs_group(js_dir="../js")
    robot_name = "freeBody"
    robot_dimensions = [2, 1, 1]  # Dimensions for a "freeBody" robot
    robot_color = "0x0000ff"  # Blue for the robot's path

    # Prepare the trajectory with position and orientation for each step
    trajectory = []
    for t, config in enumerate(path):
        position = config[:3]  # Extract position
        quaternion = config[3:]  # Extract quaternion
        # Append each keyframe to the trajectory
        trajectory.append([t, position, quaternion, robot_color])

    # Create the initial robot box, assuming the first configuration for the initial setup
    initial_position = path[0][:3]
    initial_quaternion = path[0][3:]
    robot_box = box(robot_name, *robot_dimensions, initial_position, initial_quaternion)
    # Add animation to visualize the robot's movement
    viz_out.add_animation(robot_box, trajectory)

    # Add obstacles to the visualization,
    for i, obstacle in enumerate(obstacles):
        # Generate a valid name for the sphere based on its index
        ob_name = f"obstacle_{i}"
        ob_geom = sphere(ob_name, obstacle.radius, obstacle.position, [1, 0, 0, 0])
        viz_out.add_obstacle(ob_geom, "0x00ff00")  # Green for obstacles

    # Generate the HTML file for visualization
    viz_out.to_html("prm_star_vehicle_movement_animation.html", "../out/")

def visualize_prm(prm_graph, obstacles, filename="prm_star_with_obstacles_visualization.html"):
    """
    Visualizes the PRM graph alongside obstacles.
    
    :param prm_graph: The PRM graph, containing 'nodes' and 'edges'.
    :param obstacles: The list of obstacles in the scene.
    :param filename: The filename for the output HTML visualization.
    """
    viz_out = threejs_group(js_dir="../js")
    node_color = "0x00ff00"  # Green for nodes
    edge_color = "0x0000ff"  # Blue for edges
    obstacle_color = "0xff0000"  # Red for obstacles

    # Visualize nodes as small spheres
    for i, node in enumerate(prm_graph['nodes']):
        # Generate a unique name for each node
        node_name = f"node_{i}"
        node_geom = sphere(node_name, 0.1, node[:3], [1, 0, 0, 0])
        viz_out.add_obstacle(node_geom, node_color)
    
    # Visualize edges as lines
    for edge in prm_graph['edges']:
        start_node = prm_graph['nodes'][edge[0]]
        end_node = prm_graph['nodes'][edge[1]]
        viz_out.add_line([start_node[:3], end_node[:3]], edge_color)
    
    # Add obstacles to the visualization
    for i, obstacle in enumerate(obstacles):
        ob_name = f"obstacle_{i}"
        ob_geom = sphere(ob_name, obstacle.radius, obstacle.position, [1, 0, 0, 0])
        viz_out.add_obstacle(ob_geom, obstacle_color)

    # Generate the HTML file for visualization
    viz_out.to_html(filename, "../out/")

def visualize_prm_path(path_configs, obstacles, filename, robot_type):
    """
    Visualizes the path nodes and edges, alongside obstacles, using only the path information.
    
    :param path_configs: A list of configurations representing the path from start to goal.
                         For an arm robot, each configuration is a list of joint angles.
                         For a freeBody robot, each configuration is a list of [x, y, z, qw, qx, qy, qz].
    :param obstacles: The list of obstacles in the scene.
    :param filename: The filename for the output HTML visualization.
    :param robot_type: The type of robot ('arm' or 'freeBody').
    """
    viz_out = threejs_group(js_dir="../js")
    node_color = "0x00ff00"  # Green for path nodes
    edge_color = "0x0000ff"  # Blue for path edges
    obstacle_color = "0xff0000"  # Red for obstacles

    # Process each node in the path
    for i, config in enumerate(path_configs):
        if robot_type == "freeBody":
            # For freeBody, extract position and orientation from the configuration
            position = config[:3]
            orientation = config[3:]
            node_name = f"path_node_{i}"
            # Visualize path nodes with orientation
            node_geom = sphere(node_name, 0.1, config, [1, 0, 0, 0])  # Example dimensions for visualization
            viz_out.add_obstacle(node_geom, node_color)
        else:
            # For arm or other types, only position is considered
            node_name = f"path_node_{i}"
            node_geom = sphere(node_name, 0.1, config, [1, 0, 0, 0])  # Assuming config is a position for simplicity
            viz_out.add_obstacle(node_geom, node_color)

    # Visualize path edges as lines connecting consecutive nodes (only positions are considered)
    for i in range(len(path_configs) - 1):
        start_position = path_configs[i][:3]  # Only the position part is used for the line
        end_position = path_configs[i + 1][:3]
        viz_out.add_line([start_position, end_position], edge_color)

    # Add obstacles to the visualization
    for i, obstacle in enumerate(obstacles):
        ob_name = f"obstacle_{i}"
        ob_geom = sphere(ob_name, obstacle.radius, obstacle.position, [1, 0, 0, 0])
        viz_out.add_obstacle(ob_geom, obstacle_color)

    # Generate the HTML file for visualization
    viz_out.to_html(filename, "../out/")

def main_prm_star():
    args = parse_arguments()
    obstacles = load_obstacles(args.map)
    visualize_obstacles(obstacles)
    nodes = generate_prm_nodes(args.robot, 5000, obstacles)
    graph = construct_prm_star_graph(nodes, obstacles, args.robot)
    start_config = np.array(args.start)
    goal_config = np.array(args.goal)
    path = find_path_prm_star(start_config, goal_config, graph, args.robot)

    if path:
        print("Path found:")
        for node in path:
            print(node)
        print("Visualizing path...")
        if args.robot == 'arm':
            visualize_arm_movement(path, obstacles)
        elif args.robot == 'freeBody':
            visualize_freeBody_path(path, obstacles)
        visualize_prm(graph, obstacles, "prm_star_with_obstacles_visualization.html")
        visualize_prm_path(path, obstacles, "prm_star_path_visualization.html", args.robot)
    else:
        print("No path found.")

if __name__ == '__main__':
    main_prm_star()