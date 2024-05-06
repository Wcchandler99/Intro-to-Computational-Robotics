import numpy as np
import random
import json
import os
from geometry import *
from threejs_group import *

def generate_scene(num_spheres, r_min, r_max):
    """Generates a scene with a specified number of spheres, each with a random radius and position."""
    scene = []
    for i in range(num_spheres):
        radius = random.uniform(r_min, r_max)
        position = [random.uniform(0, 100), random.uniform(0, 100), random.uniform(0, 100)] # change range for populating according to each axis
        scene.append({"type": "sphere", "name": f"sphere_{i}", "radius": radius, "position": position})
    return scene

def scene_to_file(scene, filename):
    """Writes the scene to a JSON file."""
    directory = os.path.dirname(filename)
    if directory and not os.path.exists(directory):
        os.makedirs(directory, exist_ok=True)
    with open(filename, 'w') as file:
        json.dump(scene, file)

def scene_from_file(filename):
    """Reads the scene from a JSON file."""
    with open(filename, 'r') as file:
        return json.load(file)

def visualize_scene(scene, output_filename):
    """Generates an HTML file for visualizing the scene."""
    viz_out = threejs_group(js_dir="../js") 
    for obj in scene:
        geom = sphere(obj["name"], obj["radius"], obj["position"], [1, 0, 0, 0])
        viz_out.add_obstacle(geom, "0x0000ff")  # Using blue for all spheres for consistency
    viz_out.to_html(output_filename, "out/")  

if __name__ == "__main__":
    scenes_params = [
        (70, 1, 3, "dense_small_spheres"),  # Dense scene with small spheres
        (10, 5, 10, "sparse_large_spheres"),  # Sparse scene with large spheres
        (30, 2, 5, "moderate_medium_spheres"),  # Moderately dense scene with medium spheres
        (100, 0.5, 2, "very_dense_very_small_spheres"),  # Very dense scene with very small spheres
        (15, 1, 10, "sparse_range_size_spheres")  # Sparse scene with a wide range of sphere sizes
    ]

    for num_spheres, r_min, r_max, scene_name in scenes_params:
        scene = generate_scene(num_spheres, r_min, r_max)  # Generate the scene
        scene_file = f"{scene_name}.json"  # Name the file based on the scene parameters
        scene_to_file(scene, scene_file)  # Save the scene to a file
        loaded_scene = scene_from_file(scene_file)  # Load the scene from the file
        
        # Generate visualization HTML file
        visualize_scene(loaded_scene, f"{scene_name}_visualization.html")
        
        print(f"Scene '{scene_name}' generated and saved to: {scene_file}")
