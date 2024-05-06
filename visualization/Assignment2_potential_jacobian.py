import numpy as np

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

# Function to calculate the Euclidean distance in 3D space
def euclidean_distance_3d(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))

# Adjusted function to calculate the distance from point q to the nearest obstacle
def distance_to_nearest_obstacle(q, obstacles):
    # Assuming obstacles is a list of (x, y, z, radius) for each obstacle
    min_distance = float('inf')
    for obstacle in obstacles:
        obstacle_x, obstacle_y, obstacle_z, obstacle_radius = obstacle
        # Distance to the surface of the obstacle
        distance = euclidean_distance_3d(q, [obstacle_x, obstacle_y, obstacle_z]) - obstacle_radius
        min_distance = min(min_distance, distance)
    return min_distance

def attraction_gradient(end_effector_position, goal_position, alpha):
    grad = alpha * (np.array(end_effector_position) - np.array(goal_position))
    return grad

def repulsion_gradient_for_segment(segment_position, segment_orientation, obstacle, rho_0, eta):
    # Convert quaternion to rotation matrix for the segment
    rot_matrix = quaternion_to_matrix(segment_orientation)
    
    # Define the segment's vector in its local frame (assuming length is in the z-direction)
    segment_vector = np.array([0, 0, 1])  # Simplification
    
    # Convert the segment's vector to the global frame
    segment_vector_global = np.dot(rot_matrix, segment_vector)
    
    # Find the closest point on the segment to the obstacle (simplified approach)
    obstacle_pos = np.array(obstacle[:3])
    closest_point_on_segment = segment_position + segment_vector_global * 0.5  # Assuming midpoint for simplicity
    
    # Calculate the distance from the obstacle to this point
    distance = np.linalg.norm(closest_point_on_segment - obstacle_pos) - obstacle[3]  # Subtract obstacle radius
    
    if distance <= rho_0 and distance > 0:
        # Calculate gradient based on distance
        direction = (closest_point_on_segment - obstacle_pos) / distance
        grad = eta * (1/distance - 1/rho_0) * (1/distance**2) * direction
        return grad
    else:
        return np.zeros(3)


def potential_gradient(configuration, goal_position, obstacles, alpha, eta, rho_0):
    segment_transformations = forward_kinematics(configuration)
    
    # Initialize the gradient vector for the configuration
    gradient = np.zeros_like(configuration, dtype=np.float64)

    # Attraction to the goal (considering end-effector only for simplicity)
    end_effector_pos = segment_transformations[-1][0]
    attraction_vector = np.array(goal_position) - np.array(end_effector_pos)
    attraction_grad = alpha * attraction_vector  # Simplified attraction gradient in task space
    
    # Repulsion from obstacles (simplified for the closest point or end-effector)
    repulsion_grad = np.zeros(3)  # Initialize repulsion gradient in task space
    for obstacle in obstacles:
        obstacle_pos = np.array(obstacle[:3])
        distance = euclidean_distance_3d(end_effector_pos, obstacle_pos) - obstacle[3]
        if distance <= rho_0 and distance > 0:
            direction = (end_effector_pos - obstacle_pos) / distance
            repulsion_grad += eta * (1/distance - 1/rho_0) * (1/distance**2) * direction

    # Combine attraction and repulsion in task space
    combined_task_grad = attraction_grad - repulsion_grad

    # Calculate the Jacobian for the current configuration
    J = calculate_jacobian(configuration)
    
    # Map task space gradients back to configuration space
    config_space_grad = np.dot(J.T, combined_task_grad)  # J.T is the transpose of the Jacobian
    
    # Update the gradient for the configuration
    gradient += config_space_grad
    
    return gradient


def calculate_jacobian(configuration):
    theta1, theta2, theta3 = configuration
    l1 = 4  # Length of the first link
    l2 = 4  # Length of the second link
    
    # Assuming the end-effector is attached to the end of the second link.
    
    # Partial derivatives of end-effector position with respect to each theta
    # These are derived from the forward kinematics of the arm
    J = np.zeros((3, 3))
    
    # For a joint rotating around Z-axis
    J[0, 0] = -l1 * np.sin(theta1) - l2 * np.sin(theta1 + theta2)
    J[1, 0] = l1 * np.cos(theta1) + l2 * np.cos(theta1 + theta2)
    J[2, 0] = 0  # Rotation around Z doesn't affect Z position in this simple 2D planar case
    
    # For a joint rotating around Y-axis (first link)
    J[0, 1] = -l2 * np.sin(theta1 + theta2)  # Affects X position
    J[1, 1] = l2 * np.cos(theta1 + theta2)   # Affects Y position
    J[2, 1] = 0  # Assuming planar movement, Y rotation doesn't affect Z
    
    # For a joint rotating around Y-axis (second link)
    J[0, 2] = -l2 * np.sin(theta1 + theta2)  # Similar effect as first Y-axis rotation
    J[1, 2] = l2 * np.cos(theta1 + theta2)
    J[2, 2] = 0  # Assuming planar movement, Y rotation doesn't affect Z
    
    return J


def compute_arm_path(start_configuration, goal_position, obstacles, alpha, eta, rho_0, learning_rate=0.01, tolerance=1e-5, max_steps=100):
    q = np.array(start_configuration)
    path = [q.tolist()]

    for _ in range(max_steps):
        gradient = potential_gradient(q, goal_position, obstacles, alpha, eta, rho_0)
        
        # Normalize the gradient to have a constant step size, ensuring we move consistently in the steepest descent direction
        norm_gradient = gradient / (np.linalg.norm(gradient) + 1e-8)
        
        # Update configuration directly using the normalized gradient
        q_new = q - learning_rate * norm_gradient

        path.append(q_new.tolist())
        
        # Evaluate the stopping criterion based on the magnitude of configuration changes
        if np.linalg.norm(q_new - q) < tolerance:
            print(f"Stopping after {_ + 1} iterations due to small configuration change.")
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
    viz_group.add_obstacle(geom, red) 
    
    # Generate HTML to visualize the robot arm with animation
    viz_group.to_html("potential_arm_animation.html", "../out/")

# Example usage adjustments
initial_configuration = [-3.14, 1.0, 0.8]  # Starting configuration of the arm
goal_position = [np.pi, 0.5, 0.8]  # Goal position for the arm
obstacles = [(0, 0, 4, 1)]  # List of obstacles (x, y, z, radius)

# Constants for the potentials
alpha = 1.0  # Scaling factor for the attractive potential
eta = 1.0  # Scaling factor for the repulsive potential
rho_0 = 1  # Influence distance for the repulsive potential

# Call the visualize_arm_path function to generate the animation
visualize_arm_path(initial_configuration, goal_position, obstacles, alpha, eta, rho_0)