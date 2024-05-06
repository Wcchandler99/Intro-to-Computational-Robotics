import numpy as np
import random
import json
import os
from geometry import *
from threejs_group import *
import math
from queue import PriorityQueue

def generate_scene(num_spheres, r_min, r_max):
    """Generates a scene with a specified number of spheres, each with a random radius and position."""
    scene = []
    for i in range(num_spheres):
        radius = random.uniform(r_min, r_max)
        #position = [random.uniform(0, 100), random.uniform(0, 100), random.uniform(0, 100)]
        position = [random.uniform(0, 10), random.uniform(0, 10), 0]
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
    #viz_out = threejs_group(js_dir="../js")
    for obj in scene:
        geom = sphere(obj["name"], obj["radius"], obj["position"], [1, 0, 0, 0])
        viz_out.add_obstacle(geom, "0x0000ff")  # Using blue for all spheres for consistency
    #viz_out.to_html(output_filename, "out/")  # Ensure the "out/" directory exists # Removed to make compatable with visualize robot movement
    
def distance_point_to_line(point, line_start, line_end):
    line_vector = [line_end[i] - line_start[i] for i in range(3)]
    point_vector = [point[i] - line_start[i] for i in range(3)]

    t = sum(line_vector[i] * point_vector[i] for i in range(3)) / sum(line_vector[i] ** 2 for i in range(3))

    t = max(0, min(1, t))

    closest_point = [line_start[i] + t * line_vector[i] for i in range(3)]

    return closest_point

def distance_between_points(p1, p2):
    return np.linalg.norm(p1 - p2)

def check_collision(cube_center, cube_half_extents, sphere_center, sphere_radius): #, face_color, edge_color, vertex_color):             #COLISION DEBUG
    
    #Check for sphere-cube face collision
    for i in range(3):                     
        if abs(cube_center[i] - sphere_center[i]) > cube_half_extents[i] + sphere_radius:
            face_color = green
            return False #, face_color, edge_color, vertex_color  # No collision             #COLISION DEBUG
    
    # Check collision with cube edges
    closest_point = np.clip(sphere_center, cube_center - cube_half_extents, cube_center + cube_half_extents)
    
    if distance_between_points(sphere_center, closest_point) < sphere_radius:
        edge_color = green
        return True #, face_color, edge_color, vertex_color             #COLISION DEBUG
        
    # Sphere Vertex Collision
    for i in range(2):
        for j in range(2):
            for k in range(2):
                vertex = [cube_center[0] + cube_half_extents[0] * ((-1) ** i),
                          cube_center[1] + cube_half_extents[1] * ((-1) ** j),
                          cube_center[2] + cube_half_extents[2] * ((-1) ** k)]
                
                if math.sqrt(sum((sphere_center[l] - vertex[l]) ** 2 for l in range(3))) < sphere_radius:
                    vertex_color = green
                    return True #, face_color, edge_color, vertex_color             #COLISION DEBUG
    # face_color = red             #COLISION DEBUG
    # edge_color = red             #COLISION DEBUG
    # vertex_color = red                #COLISION DEBUG            
    return False #, face_color, edge_color, vertex_color             #COLISION DEBUG

def visualize_collision(robot_state, robot_geometry, scene):
    collision = []
    for obstacle in scene:
        if check_collision(np.array(robot_state), robot_geometry, np.array(obstacle["position"]), np.array(obstacle["radius"])):
            collision.append(True)
        else:
            collision.append(False)
    
    if any(collision):
        color = red
    else:
        color = green
    return color

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
    #viz_out = threejs_group(js_dir="../js")
    color = blue
    cube = box('robot', cube_size, cube_size, cube_size, path[0], [1, 0, 0, 0])
    cube_trajectory = []
    for i, pos in enumerate(path):
        cube_trajectory.append([i, pos, [1, 0, 0, 0], color])
        color = visualize_collision(pos, [cube_size/2, cube_size/2, cube_size/2], scene)
    viz_out.add_animation(cube, cube_trajectory)
    #output_html = "robot_path_visualization.html"
    #viz_out.to_html(output_html, "out/")
    #print(f"Visualization saved to: {output_html}")
    
if __name__ == "__main__":
    blue="0x0000ff"
    red="0xff0000"
    green="0x00ff00"
    num_spheres = 3
    min_rad = 1
    max_rad = 5
    
    viz_out = threejs_group(js_dir="../js")
    
    # Example usage
    start = (0, 0, 0)  # Start position in grid cells
    goal = (20, 20, 20)  # Goal position in grid cells
    scenes_params = [
    (50, 1, 3, "dense_small_spheres"),  # Dense scene with small spheres
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
    
    path = A_star(start, goal, scene)
    if path:
        print("Path found:", path)
        visualize_robot_path(path)
    else:
        print("No path found.")

    output_html = "robot_path_visualization.html"
    viz_out.to_html(output_html, "out/")
    print(f"Visualization saved to: {output_html}")

