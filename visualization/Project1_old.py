import numpy as np
from geometry import * 
from threejs_group import *
import random
import math


def generate_scene(num_spheres, r_min, r_max):
    if __name__ == "__main__":
        blue="0x0000ff"
        viz_out = threejs_group(js_dir="../js")

        for i in range(num_spheres):
            radius = random.choice(range(r_min, r_max))
            positionx, positiony, positionz = random.choice(range(0, 100)), random.choice((0, 100)), random.choice((0, 100))
            geom = sphere(f"sphere_{i}", radius, [positionx, positiony, positionz], [1, 0, 0, 0])
            viz_out.add_obstacle(geom, blue)
        
        viz_out.to_html("animation.html", "out/");
        
# def move_object(object, traj, rotation, color):
#     trajectory = []
#     # state: [time, position, quarternion (orientation), color]
#     state = [t, traj, rotation, color] 
#     # geom0.position[0] = geom0.position[0] + [t, 0, 0] #something like this
#     object.position[0] += traj[0]
#     object.position[1] += traj[1]
#     object.position[2] += traj[2]
#     trajectory.append(state) #trajectory is an array of states
    
#     return trajectory, state
        
#generate_scene(5, 3, 10)

def distance_point_to_line(point, line_start, line_end):
    line_vector = [line_end[i] - line_start[i] for i in range(3)]
    point_vector = [point[i] - line_start[i] for i in range(3)]

    t = sum(line_vector[i] * point_vector[i] for i in range(3)) / sum(line_vector[i] ** 2 for i in range(3))

    t = max(0, min(1, t))  # Clamp t to the range [0, 1]

    closest_point = [line_start[i] + t * line_vector[i] for i in range(3)]

    return closest_point

def check_collision(cube_center, cube_half_extents, sphere_center, sphere_radius):
        
    # Sphere-Cube edge collision
    for i in range(3):
        edge_start = [cube_center[j] + cube_half_extents[j] * ((-1) ** (i // 2)) if j != i else cube_center[j] - cube_half_extents[j] for j in range(3)]
        edge_end = [cube_center[j] + cube_half_extents[j] * ((-1) ** ((i // 2) + 1)) if j != i else cube_center[j] + cube_half_extents[j] for j in range(3)]
        
        closest_point = distance_point_to_line(sphere_center, edge_start, edge_end)
        
        if math.sqrt(sum((sphere_center[j] - closest_point[j]) ** 2 for j in range(3))) < sphere_radius:
            return True
        
    # Sphere Vertex Collision
    for i in range(2):
        for j in range(2):
            for k in range(2):
                vertex = [cube_center[0] + cube_half_extents[0] * ((-1) ** i),
                          cube_center[1] + cube_half_extents[1] * ((-1) ** j),
                          cube_center[2] + cube_half_extents[2] * ((-1) ** k)]
                
                if math.sqrt(sum((sphere_center[l] - vertex[l]) ** 2 for l in range(3))) < sphere_radius:
                    return True
                
    return False


if __name__ == "__main__":
    blue="0x0000ff"
    red="0xff0000"
    green="0x00ff00"
    trajectory0 = []
    trajectory1 = []
    color = green
    square_size = 1
    radius = 3
    distance_squared = 100
    sum_of_radii_squared = 1
    viz_out = threejs_group(js_dir="../js")
    
    cube0 = box('box0', square_size, square_size, square_size, [0, 2, .5], [1, 0, 0, 0])
    sphere0 = sphere("sphere0", radius, [5, 5, 0], [1, 0, 0, 0])
    
    # geom1.position
    # geom1.orientation
    # can be assigned also so geom1.position = new position
    # have to store the trajectory somewhere else
    
    
    for t in np.arange(0,10,.01, dtype=float):
        # state: [time, position, quarternion (orientation), color]
        cube_state = [t, [t, 2, .5], [1,0,0,0], color] 
        #----
        # geom0.position[0] = geom0.position[0] + [t, 0, 0] #something like this
        # geom0.position[0] += t
        #----------
        trajectory0.append(cube_state) #trajectory is an array of states
        # trajectory0.append(move_object(geom0, [t, t, 0], [1, 0, 0, 0], color)[0])
        # state0 = move_object(geom0, [t, t, 0], [1, 0, 0, 0], color)[1]
        
        
        sphere_state = [t, [5, 5, 0], [1, 0, 0, 0], blue]
        trajectory1.append(sphere_state)
        
        if check_collision(cube_state[1], [square_size/2, square_size/2, square_size/2], sphere_state[1], radius):
            color = red
        else:
            color = green
        # distance_squared = sum((state0[1][i] - state1[1][i]) ** 2 for i in range(3)) #use geom0.position here
        # sum_of_radii_squared = (square_size / 2 + radius) ** 2
        
        # if distance_squared < sum_of_radii_squared:
        #     color = red
        # else:
        #     color = green
    
    viz_out.add_animation(cube0, trajectory0)
    viz_out.add_animation(sphere0, trajectory1)
    
    viz_out.to_html("animation.html", "out/");
        