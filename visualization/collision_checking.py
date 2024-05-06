import numpy as np
from geometry import sphere, box
from threejs_group import threejs_group
import random
import math
import json
import os


def distance_between_points(p1, p2):
    return np.linalg.norm(np.array(p1) - np.array(p2))

def check_collision(cube_center, cube_half_extents, sphere_center, sphere_radius):
    # Convert lists to NumPy arrays for operations
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
                vertex = np.array([
                    cube_center[0] + cube_half_extents[0] * ((-1) ** i),
                    cube_center[1] + cube_half_extents[1] * ((-1) ** j),
                    cube_center[2] + cube_half_extents[2] * ((-1) ** k)
                ])
                if np.linalg.norm(sphere_center - vertex) < sphere_radius:
                    return True  # Collision detected
    return False
def scene_from_file(filename):
    """Reads the scene from a JSON file."""
    with open(filename, 'r') as file:
        return json.load(file)

def visualize_collisions_from_scene(scene_file, sim_length):
    scene = scene_from_file(scene_file)  # Load the scene from the file
    viz_out = threejs_group(js_dir="../js")
    robot_trajectory = []
    robot = box('robot', 5, 5, 5, [0, 0, 0], [1, 0, 0, 0])

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
        robot.position = [random.uniform(0, 95), random.uniform(0, 95), random.uniform(0, 95)]
        robot_state = [t, robot.position, [1, 0, 0, 0], "0x0000ff"]
        robot_trajectory.append(robot_state)
        
        for key, value in obstacle_dict.items():
            sphere_collision_color = "0x00ff00"  # Assume no collision initially
            if check_collision(robot.position, [robot.width/2, robot.height/2, robot.depth/2], value['position'], value['radius']):
                sphere_collision_color = "0xff0000"  # Collision detected
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
        "dense_small_spheres.json",
        "sparse_large_spheres.json",
        "moderate_medium_spheres.json",
        "very_dense_very_small_spheres.json",
        "sparse_range_size_spheres.json"
    ]

    for scene_file in scene_files:
        visualize_collisions_from_scene(scene_file, sim_length)
        print(f"Visualization generated for {scene_file}")
