import numpy as np

import argparse
# Import classes
from geometry import *
from threejs_group import threejs_group

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

# Function to calculate the distance from point q to the nearest obstacle
def distance_to_nearest_obstacle(q, obstacles):
    min_distance = float('inf')
    for obstacle in obstacles:
        # Extract x and y coordinates of the obstacle
        obstacle_x, obstacle_y, obstacle_z = obstacle
        # Calculate the Euclidean distance in 2D space
        distance = np.linalg.norm(np.array(q)[:3] - np.array([obstacle_x, obstacle_y, obstacle_z]))
        min_distance = min(min_distance, distance)
    return min_distance

# Gradient of the attraction potential function
def attraction_gradient(q, final_configuration, alpha):
    # Extract the x and y components of the configuration q
    q_x, q_y, q_t = q
    # Calculate the gradient in the x and y directions
    grad_x = alpha * (q_x - final_configuration[0])
    grad_y = alpha * (q_y - final_configuration[1])
    grad_theta = alpha * (q_t - final_configuration[2])
    return np.array([grad_x, grad_y, grad_theta])  # Append a zero for the z-component

# Gradient of the repulsion potential function
def repulsion_gradient(q, obstacles, rho_0, eta):
    rho_q = distance_to_nearest_obstacle(q, obstacles)
    if rho_q <= rho_0:
        grad = np.zeros_like(q)
        for i in range(len(q)):
            # Calculate the gradient contribution from each obstacle
            obstacle_grad = np.zeros_like(q)
            for obstacle in obstacles:
                # Treat the obstacle as a 3D point with z-coordinate as 0
                obstacle_3d = np.array([obstacle[0], obstacle[1], obstacle[2]])
                diff = q[i] - obstacle_3d[i]
                obstacle_grad[i] = eta * (1 - rho_q / rho_0) * (diff / np.linalg.norm(np.array(q) - obstacle_3d))**2
            # Accumulate the gradient contribution from all obstacles
            grad += obstacle_grad
        return grad
    else:
        return np.zeros_like(q)

# Combined gradient of the potential function
def potential_gradient(q, final_configuration, obstacles, alpha, eta, rho_0):
    return attraction_gradient(q, final_configuration, alpha) + repulsion_gradient(q, obstacles, rho_0, eta)

# Function to compute the path with gradient descent
def compute_arm_path(start_configuration, final_configuration, obstacles, alpha, eta, rho_0, steps=100, learning_rate=0.01, tolerance=1e-5):
    q = np.array(start_configuration)
    path = [q]
    for _ in range(steps):
        gradient = potential_gradient(q, final_configuration, obstacles, alpha, eta, rho_0)
        q_new = q - learning_rate * gradient
        path.append(q_new.tolist())
        if np.linalg.norm(q_new - q) < tolerance:
            break
        q = q_new
    return path

# Visualization function with animation
def visualize_arm_path(start_configuration, final_configuration, obstacles, alpha, eta, rho_0):
    # Initialize the visualization group
    viz_group = threejs_group(js_dir="../js")

    # Define colors for different links
    base_color = "0x0000ff"  # Blue for the base
    link1_color = "0x00ff00"  # Green for link 1
    link2_color = "0xff0000"  # Red for link 2
    
    # Compute the path of configurations
    path = compute_arm_path(start_configuration, final_configuration, obstacles, alpha, eta, rho_0)

    # Create the boxes for each link and store them in a dictionary with their colors
    boxes = {
        'link0': box("base", 2, 2, 0.5, [0, 0, 0], quaternion_from_angle_z(0)),
        'link1': box("link1", 1, 1, 4, [0, 0, 0], quaternion_from_angle_y(0)),
        'link2': box("link2", 1, 1, 4, [0, 0, 0], quaternion_from_angle_y(0))
    }
    link_colors = {
        'link0': base_color,
        'link1': link1_color,
        'link2': link2_color
    }

    # Create the keyframe data for the animation
    keyframes = {name: [] for name in boxes}  # Initialize a dictionary of keyframes for each link

    # Loop through each configuration in the path to create keyframe data
    for t, configuration in enumerate(path):
        transformations = forward_kinematics(configuration)
        for i, transformation in enumerate(transformations):
            position, quaternion = transformation
            keyframes[f'link{i}'].append({
                'time': t,  # Time of the keyframe
                'position': position,  # Position at the keyframe
                'quaternion': quaternion  # Quaternion at the keyframe
            })

    # Add animations for each link using the keyframe data and set the colors
    for name, keyframe_data in keyframes.items():
        animation_data = [(kf['time'], kf['position'], kf['quaternion'], link_colors[name]) for kf in keyframe_data]
        viz_group.add_animation(boxes[name], animation_data)

    red="0xff0000"
    geom = sphere("sphere_0", 1, [0, 0, 4], [1, 0, 0, 0]) # name, radius, position, rotation
    viz_group.add_obstacle(geom, red) # draws obstacle
    
    # Generate HTML to visualize the robot arm with animation
    viz_group.to_html("robot_arm_animation.html", "../out/")

def parse_arguments():
    parser = argparse.ArgumentParser(description='Robotic arm potential field navigation.')
    parser.add_argument('--start', nargs=3, type=float, help='Start configuration of the arm: theta1 theta2 theta3')
    parser.add_argument('--goal', nargs=3, type=float, help='Goal configuration of the arm: theta1 theta2 theta3')
    return parser.parse_args()

def main():
    args = parse_arguments()
    
    # Convert parsed argument lists to numpy arrays for easier manipulation
    start_configuration = np.array(args.start)
    goal_configuration = np.array(args.goal)

    # Define the single spherical obstacle
    obstacles = [(0, 0, 4)]  # Define obstacle positions
    alpha = 1.0  # Attraction potential constant
    eta = 1.0  # Repulsion potential constant
    rho_0 = 1.0  # Repulsion threshold distance
    
    # Call the visualize_arm_path function to generate the animation
    visualize_arm_path(start_configuration, goal_configuration, obstacles, alpha, eta, rho_0)

if __name__ == '__main__':
    main()
