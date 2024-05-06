import json
import numpy as np
import os
from geometry import *
from threejs_group import *
from queue import PriorityQueue
import random
import math
from collision_checking import check_collision

def scene_from_file(filename):
    """Loads the scene data from a JSON file and wraps it in a dictionary if necessary."""
    with open(filename, 'r') as file:
        data = json.load(file)
    # Wrap the list in a dictionary under 'obstacles' if it's not already a dictionary
    if isinstance(data, list):
        return {'obstacles': data}
    return data

def interpolate_rigid_body(start_position, goal_position, steps=100):
    """
    Generates a linear interpolation between the start and goal positions.

    :param start_position: The starting [x, y, z] position.
    :param goal_position: The ending [x, y, z] position.
    :param steps: Number of steps for the interpolation.
    :return: A list of interpolated positions from start to goal.
    """
    interpolated_positions = []
    for step in range(steps + 1):
        fraction = step / steps
        interpolated_position = np.array(start_position) * (1 - fraction) + np.array(goal_position) * fraction
        interpolated_positions.append(interpolated_position.tolist())
    return interpolated_positions

def visualize_robot_path(path, scene):
    """
    Visualizes the robot's path and the obstacles in the scene, with the robot in blue and obstacles in green.
    
    :param path: A list of [x, y, z] positions representing the robot's path.
    :param scene: The scene containing obstacles.
    """
    viz_out = threejs_group(js_dir="../js")
    box_name = "robot"
    box_dimensions = [5, 5, 5]  # Assuming a cube for the robot
    robot_color = "0x0000ff"  # Blue for the robot's path

    # Convert the path into a trajectory format compatible with threejs_group
    trajectory = [[t, pos, [1, 0, 0, 0], robot_color] for t, pos in enumerate(path)]

    # Create and add the robot's trajectory to the visualization
    robot_box = box(box_name, *box_dimensions, path[0], [1, 0, 0, 0])
    viz_out.add_animation(robot_box, trajectory)

    # Add obstacles from the scene to the visualization, in green
    obstacle_color = "0x00ff00"  # Green for obstacles
    for obstacle in scene['obstacles']:
        ob_geom = sphere(obstacle['name'], obstacle['radius'], obstacle['position'], [1, 0, 0, 0])
        viz_out.add_obstacle(ob_geom, obstacle_color)

    # Generate the HTML file for visualization
    viz_out.to_html("robot_path_with_obstacles_visualization.html", "out/")


def A_star(start, goal, scene):
    """
    Implements the A* algorithm to find a path from start to goal in a 3D grid.

    :param start: The starting [x, y, z] position.
    :param goal: The goal [x, y, z] position.
    :param scene: The scene containing obstacles.
    :return: A list of positions forming the path from start to goal.
    """
    import heapq

    def heuristic(a, b):
        return np.linalg.norm(np.array(a) - np.array(b))

    def get_neighbors(position, scene):
        directions = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
        neighbors = []
        for dx, dy, dz in directions:
            neighbor_position = (position[0] + dx*5, position[1] + dy*5, position[2] + dz*5)
            if not check_collision_with_scene(neighbor_position, scene):
                neighbors.append(neighbor_position)
        return neighbors

    def check_collision_with_scene(position, scene):
        cube_half_extents = [2.5, 2.5, 2.5]  # Assuming a cube with side length of 5 units
        for obstacle in scene['obstacles']:
            sphere_center = obstacle['position']
            sphere_radius = obstacle['radius']
            if check_collision(position, cube_half_extents, sphere_center, sphere_radius):
                return True  # Collision detected
        return False


    open_set = []
    heapq.heappush(open_set, (0 + heuristic(start, goal), 0, start))
    came_from = {}
    g_score = {start: 0}

    while open_set:
        _, current_cost, current = heapq.heappop(open_set)

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]

        for neighbor in get_neighbors(current, scene):
            tentative_g_score = g_score[current] + heuristic(current, neighbor)
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score, tentative_g_score, neighbor))

    return []  # Return an empty path if no path is found

if __name__ == "__main__":
    scene_filename = "sparse_range_size_spheres.json"  # Adjust as necessary
    scene = scene_from_file(scene_filename)  # Load scene data
    start = (0, 0, 0)  # Adjust start position as necessary
    goal = (25, 100, 25)  # Adjust goal position as necessary

    # Assuming the scene data structure matches the expected format
    path = A_star(start, goal, scene)
    if path:
        print("Path found:", path)
        visualize_robot_path(path, scene)
    else:
        print("No path found.")

