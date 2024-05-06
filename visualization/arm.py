import numpy as np

# Import classes
from geometry import box
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

# Forward kinematics calculation
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

# Function to compute the path
def compute_arm_path(start_configuration, end_configuration, steps=100):
    path = []
    # Linear interpolation between start and end configurations
    for t in np.linspace(0, 1, steps):
        interpolated_configuration = tuple(np.array(start_configuration) + t * (np.array(end_configuration) - np.array(start_configuration)))
        path.append(interpolated_configuration)
    return path

# Visualization function with animation
def visualize_arm_path(start_configuration, end_configuration):
    # Initialize the visualization group
    viz_group = threejs_group(js_dir="../js")

    # Define colors for different links
    base_color = "0x0000ff"  # Blue for the base
    link1_color = "0x00ff00"  # Green for link 1
    link2_color = "0xff0000"  # Red for link 2

    # Compute the path of configurations
    path = compute_arm_path(start_configuration, end_configuration)

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

    # Generate HTML to visualize the robot arm with animation
    viz_group.to_html("robot_arm_animation.html", "out/")

# Example usage
initial_configuration = (np.pi/2, np.pi/2, -np.pi/3)  # Replace with initial configuration
final_configuration = (np.pi/3, -np.pi/4, np.pi/4)  # Replace with final configuration

# Call the visualize_arm_path function to generate the animation
visualize_arm_path(initial_configuration, final_configuration)
