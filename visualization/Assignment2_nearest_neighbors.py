import argparse
import numpy as np

from geometry import *
from threejs_group import *

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

# Euclidean distance between vectors
def euclidean_distance(vec1, vec2):
    return np.linalg.norm(np.array(vec1) - np.array(vec2))

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


# Parsing command-line arguments
parser = argparse.ArgumentParser(description="Find the k nearest neighbors for a given robot configuration.")
parser.add_argument('--robot', type=str, choices=['arm', 'vehicle'], required=True, help='Robot type: "arm" or "vehicle"')
parser.add_argument('--target', nargs='+', type=float, required=True, help='Target configuration as a list of numbers')
parser.add_argument('-k', type=int, required=True, help='Number of nearest neighbors to find')
parser.add_argument('--configs', type=str, required=True, help='File containing robot configurations')
args = parser.parse_args()

# Read configurations from file
def read_configs(filename, robot_type):
    configs = []
    with open(filename, 'r') as f:
        for line in f:
            parts = line.strip().split()
            if robot_type == 'arm' and len(parts) == 3:
                configs.append(list(map(float, parts)))
            elif robot_type == 'vehicle' and len(parts) == 7:
                configs.append(list(map(float, parts)))
    return configs

# Distance calculation for arm
def arm_distance(config1, config2):
    return euclidean_distance(config1, config2)

# Distance calculation for vehicle
def vehicle_distance(target, config):
    translation_distance = euclidean_distance(target[:3], config[:3])
    rotation_distance = quaternion_distance(target[3:], config[3:])
    return translation_distance + rotation_distance

# Find nearest neighbors
def find_nearest_neighbors(target, configs, k, distance_func):
    distances = [distance_func(target, config) for config in configs]
    nearest_indices = np.argsort(distances)[:k]
    return [configs[i] for i in nearest_indices]

def visualize_arm_configurations(initial_configuration, neighbor_configurations):
    viz_group = threejs_group(js_dir="../js")  # Assume default js_dir or adjust as needed

    # Function to add a single arm's visualization
    def add_arm(configuration, suffix):
        transformations = forward_kinematics(configuration)
        # Link dimensions
        dimensions = [
            (2, 2, 0.5),  # Base dimensions
            (1, 1, 4),   # Link 1 dimensions
            (1, 1, 4)    # Link 2 dimensions
        ]
        
        # Decide the color based on the suffix
        # Use blue for the initial configuration, red for neighbor configurations
        link_color = "0x0000ff" if suffix == "initial" else "0xff0000"

        for i, (position, quaternion) in enumerate(transformations):
            dim = dimensions[i]
            link_name = f"link{i}_{suffix}"
            arm_link = box(link_name, dim[0], dim[1], dim[2], position, quaternion)
            viz_group.add_obstacle(arm_link, link_color)

    # Visualize initial configuration with a unique suffix
    add_arm(initial_configuration, "initial")

    # Visualize neighbor configurations with unique suffixes
    for index, config in enumerate(neighbor_configurations, start=1):
        add_arm(config, f"neighbor_{index}")

    # Output HTML for visualization
    viz_group.to_html("arm_configuration_visualization.html", "../out/")





def visualize_vehicle_configurations(initial_configuration, neighbor_configurations):
    viz_group = threejs_group(js_dir="../js")  # Adjust the js_dir path as needed

    # Function to add a single vehicle's visualization with unique naming
    def add_vehicle(configuration, color, suffix):
        position = configuration[:3]
        quaternion = configuration[3:]
        vehicle_name = f"vehicle_{suffix}"  # Unique name for each configuration
        vehicle = box(vehicle_name, 2, 1, 1, position, quaternion)  # Vehicle dimensions
        viz_group.add_obstacle(vehicle, color)

    # Visualize initial configuration with a specific suffix
    add_vehicle(initial_configuration, "0x0000ff", "initial")  # Blue for initial configuration

    # Visualize neighbor configurations with unique suffixes
    for index, config in enumerate(neighbor_configurations, start=1):
        add_vehicle(config, "0xff0000", f"neighbor_{index}")  # Red for neighbor configurations

    # Output HTML for visualization
    viz_group.to_html("vehicle_visualization.html", "../out/")  # Adjust the output path as needed


def main():
    print(f"Initial configuration for {args.robot}: {args.target}")
    configs = read_configs(args.configs, args.robot)
    if args.robot == 'arm':
        nearest_neighbors = find_nearest_neighbors(args.target, configs, args.k, arm_distance)
        print("Nearest Arm Configurations:")
        for i, neighbor in enumerate(nearest_neighbors, start=1):
            print(f"{i}: {neighbor}")
        visualize_arm_configurations(args.target, nearest_neighbors)
    elif args.robot == 'vehicle':
        nearest_neighbors = find_nearest_neighbors(args.target, configs, args.k, vehicle_distance)
        print("Nearest Vehicle Configurations:")
        for i, neighbor in enumerate(nearest_neighbors, start=1):
            print(f"{i}: {neighbor}")
        visualize_vehicle_configurations(args.target, nearest_neighbors)

if __name__ == '__main__':
    main()
