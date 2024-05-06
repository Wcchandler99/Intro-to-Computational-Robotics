import numpy as np
from geometry import *
from threejs_group import *
from queue import PriorityQueue
import random
import math
from part1_updated import scene_from_file  # Make sure this imports your scene loading function correctly

def euclidean_distance(a, b):
    """Calculate the Euclidean distance between two points."""
    return np.linalg.norm(np.array(a) - np.array(b))

def get_neighbors(pos):
    """Generate neighbors for a given grid position."""
    moves = [(1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1)]
    return [(pos[0] + move[0], pos[1] + move[1], pos[2] + move[2]) for move in moves]

def interpolate_rigid_body(start_position, goal_position, steps=100):
    """Generates positions interpolating between start and goal positions."""
    return [tuple(np.array(start_position) + (np.array(goal_position) - np.array(start_position)) * t / steps) for t in range(steps + 1)]

def A_star(start, goal, scene, grid_size=5):
    """Find a path from start to goal using the A* algorithm."""
    open_set = PriorityQueue()
    open_set.put((0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: euclidean_distance(start, goal)}

    while not open_set.empty():
        current = open_set.get()[1]

        if current == goal:
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            return path[::-1]
        
        for neighbor in get_neighbors(current):
            tentative_g_score = g_score[current] + grid_size
            if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                f_score[neighbor] = g_score[neighbor] + euclidean_distance(neighbor, goal)
                if neighbor not in open_set.queue:
                    open_set.put((f_score[neighbor], neighbor))
    
    return None

def visualize_robot_path(path, cube_size=5):
    """Visualizes the path of the robot moving from start to goal."""
    viz_out = threejs_group(js_dir="../js")
    cube_trajectory = [(i, pos, [1, 0, 0, 0], "0x0000ff") for i, pos in enumerate(path)]
    cube = box('robot', cube_size, cube_size, cube_size, path[0], [1, 0, 0, 0])
    viz_out.add_animation(cube, cube_trajectory)
    output_html = "robot_path_visualization.html"
    viz_out.to_html(output_html, "out/")
    print(f"Visualization saved to: {output_html}")

if __name__ == "__main__":
    # Example usage
    start = (0, 0, 0)  # Start position in grid cells
    goal = (20, 20, 20)  # Goal position in grid cells
    scene = None  # Placeholder, replace with actual scene data
    path = A_star(start, goal, scene)
    if path:
        print("Path found:", path)
        visualize_robot_path(path)
    else:
        print("No path found.")
