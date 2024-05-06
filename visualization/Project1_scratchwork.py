import numpy as np
from geometry import * 
from threejs_group import *
import random
import math
import quaternion


def generate_scene(num_spheres, r_min, r_max):
    blue="0x0000ff"
    #viz_out = threejs_group(js_dir="../js")
    obstacle_dict = {}
    for i in range(num_spheres):
        radius = random.choice(range(r_min, r_max))
        positionx, positiony, positionz = random.choice(range(0, 10)), random.choice((0, 10)), 0 # changed  from 100 to 10 to make smaller
        #positionx, positiony, positionz = random.choice(range(0, 100)), random.choice((0, 100)), random.choice((0, 100))
        sphere0 = sphere(f"sphere_{i}", radius, [positionx, positiony, positionz], [1, 0, 0, 0])
        
        for t in np.arange(0, sim_length,.01, dtype=float):
            sphere_state = [t, sphere0.position, sphere0.quaternion, blue]
        
            #trajectory1.append(sphere_state)
        
        obstacle_dict[f"sphere_{i}_state"] = sphere_state
        obstacle_dict[f"sphere_{i}_radius"] = radius
        
        #obstacle_states.append(sphere_state)
        #obstacle_radius.append(radius)
        viz_out.add_obstacle(sphere0, blue)
        #trajectory1 = []
        
    return obstacle_dict
    #viz_out.to_html("animation.html", "out/");
        
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

def distance_point_to_line(point, line_start, line_end):
    line_vector = [line_end[i] - line_start[i] for i in range(3)]
    point_vector = [point[i] - line_start[i] for i in range(3)]

    t = sum(line_vector[i] * point_vector[i] for i in range(3)) / sum(line_vector[i] ** 2 for i in range(3))

    t = max(0, min(1, t))  # Clamp t to the range [0, 1]

    closest_point = [line_start[i] + t * line_vector[i] for i in range(3)]

    return closest_point

def distance_between_points(p1, p2):
    return np.linalg.norm(p1 - p2)

def check_collision(cube_center, cube_half_extents, sphere_center, sphere_radius): #, face_color, edge_color, vertex_color):             #COLISION DEBUG
    
    #Check for sphere-cube face collision
    for i in range(3):                     # Seems to work well at detecting no collision
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
    
    link0_position = np.array([5, 5, .25])
    link1_position = np.array([link0_position[0], link0_position[1], .25 + link1_height/2])
    
    joint0_position = np.array([link0_position[0], link0_position[1], link0_position[2] - .25])
    joint1_position = np.array([link0_position[0], link0_position[1], link0_position[2] + .25])
    joint2_position = np.array([link1_position[0], link1_position[1], link1_position[2] + link1_height/2])
    
    
    link0 = box('link0', link0_width, link0_length, link0_height, link0_position, [1, 0, 0, 0])
    link1 = box('link1', link1_width, link1_length, link1_height, link1_position, [1, 0, 0, 0])
    #sphere0 = sphere("sphere0", radius, [5, 5, 0], [1, 0, 0, 0])
    
    #Colision Debug:--------------------------------------------------------------------------------------
    # sphere_face = sphere("sphere_face", radius, [0, 10, 0], [1, 0, 0, 0])             #COLISION DEBUG
    # sphere_edge = sphere("sphere_edge", radius, [5, 10, 0], [1, 0, 0, 0])             #COLISION DEBUG
    # sphere_vertex = sphere("sphere_vertex", radius, [10, 10, 0], [1, 0, 0, 0])             #COLISION DEBUG
    #-----------------------------------------------------------------------------------------------------
    
    #obstacle_dict = generate_scene(num_spheres, min_radius, max_radius)
    
    # geom1.position
    # geom1.orientation
    # can be assigned also so geom1.position = new position
    # have to store the trajectory somewhere else
    
    for t in np.arange(0, sim_length, .1):
        # link1_state = [t, [0, 0, .5], [1, 0, 0, 0], color] 
        # trajectory0.append(link1_state) #trajectory is an array of states
        # Increment angle (for rotation around z-axis)
        theta0 += 0.0  # You can adjust the angular velocity as needed
        theta1 += 0.1
        
        q0 = np.quaternion(np.cos(theta0/2), 0, 0, np.sin(theta0/2))
        q1 = np.quaternion(np.cos(theta1/2), 0, np.sin(theta1/2), 0)
        
        res = q0 * q1
        
        link1_position = res * link1_position * np.conjugate(res)
       
        #-------------------------------------------
        # link1_position_quaternion = np.quaternion(0, link1_position[0], link1_position[1], link1_position[2])
        
        # rotated_link1_position_quaternion = res * link1_position_quaternion * np.conjugate(res)

        # translation1 = q1 * link1_position_quaternion * np.conjugate(q1)
        
        # link1_position = np.array([translation1.x + rotated_link1_position_quaternion.x, 
        #                            translation1.y + rotated_link1_position_quaternion.y, 
        #                            translation1.z + rotated_link1_position_quaternion.z])
        #-------------------------------------------
        # Calculate new link1 state with updated rotation
        link0_state = [t, link0_position, [q0.w, q0.x, q0.y, q0.z], color0]
        trajectory0.append(link0_state)  # Add new state to trajectory
        
        # link2_state = [t, [link1.position[0], link1.position[1], link1.position[2] + link1_height/2 + link2_height/2],
        #                [link1_state[2][0], link1_state[2][1], link1_state[2][2], link1_state[2][3]], color2] 
        link1_state = [t, link1_position, [res.w, res.x, res.y, res.z], color1]
        trajectory1.append(link1_state) #trajectory is an array of states
        
        # sphere_state = [t, [5, 5, 0], [1, 0, 0, 0], blue]
        # trajectory1.append(sphere_state)
        
        # COLISION DEBUG:-----------------------------------------------------------------------------
        # sphere_face_state = [t, [0, 10, 0], [1, 0, 0, 0], face_color]             #COLISION DEBUG
        # trajectory_face.append(sphere_face_state)             #COLISION DEBUG
        # sphere_edge_state = [t, [5, 10, 0], [1, 0, 0, 0], edge_color]             #COLISION DEBUG
        # trajectory_edge.append(sphere_edge_state)             #COLISION DEBUG
        # sphere_vertex_state = [t, [10, 10, 0], [1, 0, 0, 0], vertex_color]             #COLISION DEBUG
        # trajectory_vertex.append(sphere_vertex_state)             #COLISION DEBUG
        
        # collision = []
        # for i in range(num_spheres):
        #     collision_bool, face_color, edge_color, vertex_color = check_collision(np.array(cube_state[1]), [square_size/2, square_size/2, square_size/2], np.array(obstacle_dict[f"sphere_{i}_state"][1]), np.array(obstacle_dict[f"sphere_{i}_radius"]), face_color, edge_color, vertex_color)
        #     if collision_bool:             #COLISION DEBUG
        #         collision.append(True)            #COLISION DEBUG
        #     else:             #COLISION DEBUG
        #         collision.append(False)             #COLISION DEBUG
            
        # if any(collision):
        #     color = red
        # else:
        #     color = green
        #----------------------------------------------------------------------------------------------
        
        #Working with 1 Sphere:------------------------------------
        # if check_collision(np.array(cube_state[1]), [square_size/2, square_size/2, square_size/2], np.array(sphere_state[1]), radius):
        #     color = red
        # else:
        #     color = green
        #---------------------------------------------------------
        
        #-----------------------------
        #Combined generate scene and collision
        # collision = []
        # for i in range(num_spheres):
        #     if check_collision(np.array(link1_state[1]), [link1_width/2, link1_length/2, link1_height/2], np.array(obstacle_dict[f"sphere_{i}_state"][1]), np.array(obstacle_dict[f"sphere_{i}_radius"])):
        #         collision.append(True)
        #     else:
        #         collision.append(False)
        
        # if any(collision):
        #     color = red
        # else:
        #     color = green
        #-------------------------------------------------
    
    viz_out.add_axis(joint0_position, [q0.w, q0.x, q0.y, q0.z], length = 1) 
    viz_out.add_axis(joint1_position, [q0.w, q0.x, q0.y, q0.z], length = 1)
    viz_out.add_axis(joint2_position, [res.w, res.x, res.y, res.z], length = 1)
    
    viz_out.add_animation(link0, trajectory0)
    viz_out.add_animation(link1, trajectory1)
    #viz_out.add_animation(sphere0, trajectory1)
    
    # viz_out.add_animation(sphere_face, trajectory_face)             #COLISION DEBUG
    # viz_out.add_animation(sphere_edge, trajectory_edge)             #COLISION DEBUG
    # viz_out.add_animation(sphere_vertex, trajectory_vertex)             #COLISION DEBUG
    
    
    viz_out.to_html("animation.html", "out/");
        