import numpy as np
import argparse
import matplotlib.pyplot as plt
import logging

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

# Argument parsing
parser = argparse.ArgumentParser(description='Run RRT algorithm')
parser.add_argument('--start', nargs=3, type=float, help='Start position and orientation (x y theta)')
parser.add_argument('--goal', nargs=3, type=float, help='Goal position and orientation (x y theta)')
parser.add_argument('--map', type=str, help='Map file name')
args = parser.parse_args()

# Constants and parameters
L = 1.5  # Car wheelbase
dt = 0.1  # Time step
goal_threshold = 0.1  # Goal proximity threshold in meters
theta_threshold = 0.5  # Goal orientation threshold in radians
max_iterations = 50  # Max iterations to avoid infinite loops
step_size = 5  # Step size towards the sampled point
goal_bias = 0.1  # Probability of sampling the goal

def load_map(filename):
    # Placeholder for obstacle loading
    return []

def motion_model(state, control):
    x, y, theta = state
    v, phi = control
    x_new = x + v * np.cos(theta) * dt
    y_new = y + v * np.sin(theta) * dt
    theta_new = theta + (v / L) * np.tan(phi) * dt
    return x_new, y_new, theta_new

def is_goal_reached(current, goal):
    distance = np.linalg.norm(np.array(current[:2]) - np.array(goal[:2]))
    if distance < goal_threshold and abs(current[2] - goal[2]) < theta_threshold:
        logging.info("Goal reached successfully.")
        return True
    return False

def sample_state(tree, x_bounds, y_bounds, theta_bounds, goal):
    if np.random.uniform(0,1) < goal_bias:
        x = goal[0]
        y = goal[1]
        theta = goal[2]
        
        sample = (x, y, theta)
        nearest, dist_to_prev_node = find_nearest(tree, sample, goal)
        if dist_to_prev_node < step_size:
            return sample, nearest, dist_to_prev_node
        for node_0 in tree:
                min_dist = float('inf')
                for node_1 in tree:
                    dist = np.sqrt((node_1[0] - node_0[0])**2 + (node_1[1] - node_0[1])**2 + 0.1*(node_1[2] - node_0[2])**2)
                    if dist < min_dist and dist != 0:
                        min_dist = dist
                if min_dist > step_size:
                    #print("Goal found but steps to big")
                    check = False
        # print("Goal sampled")
        # print("GOAL: ", goal[0], goal[1], goal[2])
        # print("Sample should be GOAL: ", x, y, theta)
    else:
        x = np.random.uniform(*x_bounds)
        y = np.random.uniform(*y_bounds)
        theta = np.random.uniform(*theta_bounds)
        # print("Point sampled: ", x, y, theta)
    sample = (x, y, theta)
    nearest, dist_to_prev_node = find_nearest(tree, sample, goal)
    # print("Dist to prev node: ", dist_to_prev_node)
    # if dist_to_prev_node > step_size:
    #     sample, nearest, dist_to_prev_node = sample_state(tree, [0, 10], [0, 10], [-np.pi, np.pi], goal)
    #     find_nearest(tree, sample, goal)
    return sample, nearest, dist_to_prev_node


def find_nearest(tree, sample, goal):
    closest_node = None
    min_dist = float('inf')
    goal_dist = np.sqrt((goal[0] - sample[0])**2 + (goal[1] - sample[1])**2 + 0.1*(goal[2] - sample[2])**2)
    for node in tree:
        dist = np.sqrt((node[0] - sample[0])**2 + (node[1] - sample[1])**2 + 0.1*(node[2] - sample[2])**2)
        if dist < min_dist:
            min_dist = dist
            closest_node = node 
    if min_dist > goal_dist:
        min_dist = goal_dist
        closest_node = goal
    dist_to_prev_node = np.sqrt((tree[-1][0] - sample[0])**2 + (tree[-1][1] - sample[1])**2 + 0.1*(tree[-1][2] - sample[2])**2)
        
    return closest_node, dist_to_prev_node

def steer_towards(nearest, sample):
    angle_to_sample = np.arctan2(sample[1] - nearest[1], sample[0] - nearest[0])
    dist = np.linalg.norm([sample[0] - nearest[0], sample[1] - nearest[1]])
    control_dist = min(dist, step_size)
    control_angle = angle_to_sample - nearest[2]
    return motion_model(nearest, (control_dist / dt, control_angle))

def is_collision_free(node, obstacles):
    return True  # Placeholder for real collision checking

def rrt(start, goal, map_file):
    obstacles = load_map(map_file)
    tree = [start, goal]
    path_found = False
    for iteration in range(max_iterations):
        check = True
        if is_goal_reached(tree[-1], goal):
            for node_0 in tree:
                min_dist = float('inf')
                for node_1 in tree:
                    dist = np.sqrt((node_1[0] - node_0[0])**2 + (node_1[1] - node_0[1])**2 + 0.1*(node_1[2] - node_0[2])**2)
                    if dist < min_dist and dist != 0:
                        min_dist = dist
                if min_dist > step_size:
                    #print("Goal found but steps to big")
                    check = False
            if check:
                path_found = True
                break
        sample, nearest, dist_to_prev_node = sample_state(tree, [0, 10], [0, 10], [-np.pi, np.pi], goal)
        #nearest = find_nearest(tree, sample, goal)
        new_node = steer_towards(nearest, sample)
        if is_collision_free(new_node, obstacles):
            # tree.insert(tree.index(nearest) - 1, new_node)
            tree.append(new_node)
            logging.info(f"Added new node: {new_node}")
    if not path_found:
        logging.warning("No path found within the iteration limit.")
    return tree, path_found

import matplotlib.pyplot as plt
from itertools import combinations
import math

def distance(point1, point2):
    """
    Calculate Euclidean distance between two points.
    """
    return math.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

def plot_and_connect(points, max_distance):
    """
    Plot points and connect those within a certain distance.
    """
    fig, ax = plt.subplots()
    
    # Plot all points
    for point in points:
        ax.plot(point[0], point[1], 'bo')
    
    # Connect points within the specified distance
    for pair in combinations(points, 2):
        if distance(pair[0], pair[1]) <= max_distance:
            ax.plot([pair[0][0], pair[1][0]], [pair[0][1], pair[1][1]], 'k-')
    
    ax.set_aspect('equal', adjustable='box')
    plt.plot([0, 0, 0][0], [0, 0, 0][1], 'go', markersize=10, label='Start')
    plt.plot([5, 5, 0][0], [5, 5, 0][1], 'ro', markersize=10, label='Goal')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('RRT Path Planning')
    plt.legend()
    plt.grid(True)
    plt.show()


# Main execution
if __name__ == "__main__":
    if args.start == None or args.goal == None or args.map == None:
        path, path_found = rrt([0, 0, 0], [5, 5, 0], "place holder")
        # Visualization of the path
        # Plotting the points
        # Plotting the points
        # Example data

        # Plot points and connect those within a certain distance
        plot_and_connect(path, step_size)
        
    else:
        path, path_found = rrt(args.start, args.goal, args.map)
        # Visualization of the path
        plt.figure(figsize=(10, 10))
        plt.plot([p[0] for p in path], [p[1] for p in path], '-o', label='Path')
        plt.plot(args.start[0], args.start[1], 'go', markersize=10, label='Start')
        plt.plot(args.goal[0], args.goal[1], 'ro', markersize=10, label='Goal')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.title('RRT Path Planning')
        plt.legend()
        plt.grid(True)
        plt.show()
