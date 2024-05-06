import numpy as np
from geometry import * 
from threejs_group import *
import random
import math
import quaternion


def generate_scene(num_spheres, r_min, r_max):
    blue="0x0000ff"
    obstacle_dict = {}
    for i in range(num_spheres):
        radius = random.choice(range(r_min, r_max))
        #positionx, positiony, positionz = random.choice(range(0, 10)), random.choice((0, 10)), 0 # changed  from 100 to 10 to make smaller
        positionx, positiony, positionz = random.choice(range(0, 100)), random.choice((0, 100)), random.choice((0, 100))
        sphere0 = sphere(f"sphere_{i}", radius, [positionx, positiony, positionz], [1, 0, 0, 0])
        
        for t in np.arange(0, sim_length,.01, dtype=float):
            sphere_state = [t, sphere0.position, sphere0.quaternion, blue]
        
        obstacle_dict[f"sphere_{i}_state"] = sphere_state
        obstacle_dict[f"sphere_{i}_radius"] = radius
        
        viz_out.add_obstacle(sphere0, blue)
        
    return obstacle_dict


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

def visualize_collision(robot_state, robot_geometry, obstacle_state, obstacle_geometry):
    collision = []
    for i in range(num_spheres):
        if check_collision(np.array(robot_state[1]), robot_geometry, np.array(obstacle_state[f"sphere_{i}_state"][1]), np.array(obstacle_geometry[f"sphere_{i}_radius"])):
            collision.append(True)
        else:
            collision.append(False)
    
    if any(collision):
        color = red
    else:
        color = green
    return color


if __name__ == "__main__":
    collision_bool = False
    blue="0x0000ff"
    red="0xff0000"
    green="0x00ff00"
    trajectory0 = []
    trajectory1 = []
    trajectory_face = []
    trajectory_edge = []
    trajectory_vertex = []
    color0 = blue
    color1 = green
    face_color = blue
    edge_color = blue
    vertex_color = blue
    link0_width = 2
    link0_length = 2
    link0_height = .5
    link1_width = 1
    link1_length = 1
    link1_height = 4
    radius = 3
    min_radius = 1
    max_radius = 5
    distance_squared = 100
    sum_of_radii_squared = 1
    num_spheres = 2
    sim_length = 25
    theta0 = 0  # Initial angle
    theta1 = 0  # Initial angle
    
    viz_out = threejs_group(js_dir="../js")
    
    link1 = box('link1', link1_width, link1_length, link1_height, [0, 0, 0], [1, 0, 0, 0])
    sphere0 = sphere("sphere0", radius, [5, 5, 0], [1, 0, 0, 0])
    
    obstacle_dict = generate_scene(num_spheres, min_radius, max_radius)
    
    for t in np.arange(0, sim_length, .1):
        
        link1_state = [t, [t, 0, 0], [1, 0, 0, 0], color1]
        trajectory0.append(link1_state)  # Add new state to trajectory
        
        #-----------------------------
        #Combined generate scene and collision
        color1 = visualize_collision(link1_state, [link1_width/2, link1_length/2, link1_height/2], obstacle_dict, obstacle_dict)
        #-------------------------------------------------
    

    viz_out.add_animation(link1, trajectory0)

    
    # viz_out.add_animation(sphere_face, trajectory_face)             #COLISION DEBUG
    # viz_out.add_animation(sphere_edge, trajectory_edge)             #COLISION DEBUG
    # viz_out.add_animation(sphere_vertex, trajectory_vertex)             #COLISION DEBUG
    
    
    viz_out.to_html("animation.html", "out/");
        