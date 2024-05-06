import numpy as np
from geometry import sphere, box
from threejs_group import threejs_group
import random
import math
import json
import os

def scene_from_file(filename):
    """Reads the scene from a JSON file."""
    with open(filename, 'r') as file:
        return json.load(file)

def visualize_collisions_from_scene(scene_file, sim_length):
    scene = scene_from_file(scene_file)  # Load the scene from the file
    viz_out = threejs_group(js_dir="../js")
    robot_trajectory = []
    robot = box('robot', 2, 1, 1, [0, 0, .5], [1, 0, 0, 0])

    # Initialize obstacle dict and trajectories from the loaded scene
    obstacle_dict = {}
    obstacle_trajectories = {}
    for obj in scene:
        if obj['type'] == 'sphere':
            key = obj['name']
            radius = obj['radius']
            position = obj['position']
            obstacle_dict[key] = {'radius': radius, 'position': position}
            obstacle_trajectories[key] = []

    for t in np.arange(0, sim_length, 0.1):
        robot_state = [t, robot.position, [1, 0, 0, 0], "0x0000ff"]
        robot_trajectory.append(robot_state)
        
        for key, value in obstacle_dict.items():
            sphere_collision_color = "0x00ff00"  # Assume no collision initially
            obstacle_trajectories[key].append([t, value['position'], [1, 0, 0, 0], sphere_collision_color])

    # Add spheres with their trajectories to visualization
    for key, trajectory in obstacle_trajectories.items():
        radius = obstacle_dict[key]['radius']
        sphere_obstacle = sphere(key, radius, trajectory[0][1], trajectory[0][2])
        viz_out.add_animation(sphere_obstacle, trajectory)
    
    viz_out.add_animation(robot, robot_trajectory)
    viz_out.to_html(f"{os.path.splitext(scene_file)[0]}_collision_visualization.html", "out/")

if __name__ == "__main__":
    sim_length = 10
    scene_files = [
        "landmark_0.json",
        "landmark_1.json",
        "landmark_2.json",
        "landmark_3.json",
        "landmark_4.json"
    ]

    for scene_file in scene_files:
        visualize_collisions_from_scene(scene_file, sim_length)
        print(f"Visualization generated for {scene_file}")