import numpy as np
from geometry import sphere, box
from threejs_group import threejs_group
import random
import math
import json
import os
import sys

# Constants
L = 1.5  # Wheelbase of the car
dt = 0.1  # Time step
total_time = 10  # Duration of simulation
# Low Noise
actuation_noise = [.1, .05] #[v, theta]
odometry_noise = [.05, .03] #[v, theta]
observation_noise = [.1, .1] #[d, angle]
# # High Noise
# actuation_noise = [.3, .2] #[v, theta]
# odometry_noise = [.15, .1] #[v, theta]
# observation_noise = [.5, .25] #[d, angle]

# Dynamical model
def car_dynamics(state, control):
    x, y, theta = state
    v, phi = control
    dx = v * np.cos(theta)
    dy = v * np.sin(theta)
    dtheta = v / L * np.tan(phi)
    return np.array([dx, dy, dtheta])

# Euler integration
def euler_integration(state, control, dt):
    state_dot = car_dynamics(state, control)
    new_state = state + dt * state_dot
    return new_state

# Simulation
def simulate_trajectory(initial_state, control, total_time, dt, actuation_noise, odometry_noise, observation_noise, obstacle_dict, observed_obstacle_trajectories, particles):
    timesteps = int(total_time / dt)
    actual_trajectory = np.zeros((timesteps+1, 3))
    actual_trajectory[0] = initial_state
    observed_trajectory = np.zeros((timesteps+1, 3))
    observed_trajectory[0] = initial_state
    
    for t in range(1, timesteps+1):
        acctuation_noise_v = random.gauss(0, actuation_noise[0])                             # ADDING NOISE
        acctuation_noise_theta = random.gauss(0, actuation_noise[1])                         # ADDING NOISE
        actual_trajectory[t] = euler_integration(actual_trajectory[t-1], [control[0] + acctuation_noise_v, control[1] + acctuation_noise_theta], dt)
        odometry_noise_v = random.gauss(0, odometry_noise[0])                             # ADDING NOISE
        odometry_noise_theta = random.gauss(0, odometry_noise[1])                         # ADDING NOISE
        observed_trajectory[t] = euler_integration(observed_trajectory[t-1], [control[0] + odometry_noise_v, control[1] + odometry_noise_theta], dt)
        particles = particle_filter(particles, control)
        
        for key in obstacle_dict.keys():
            observation_noise_distance = random.gauss(0, observation_noise[0])                             # ADDING NOISE
            observation_noise_angle = random.gauss(0, observation_noise[1])                         # ADDING NOISE
            landmark_actual_observation = observe_landmark(actual_trajectory[t], obstacle_dict[f'{key}']['position'])
            landmark_noisy_observation = [landmark_actual_observation[0] + observation_noise_distance, landmark_actual_observation[1] + observation_noise_angle, 0]
            landmark_noisy_location = calculate_new_position([observed_trajectory[t][0], observed_trajectory[t][1]], landmark_noisy_observation[0], landmark_noisy_observation[1])
            observed_obstacle_trajectories[key].append([t*dt, landmark_noisy_location, [1, 0, 0, 0], "#0000FF"])
            #print("Key: ", key, "time step: ", t, landmark_noisy_location)
    return actual_trajectory, control, observed_trajectory, obstacle_dict, observed_obstacle_trajectories, particles

def particle_filter(particles, control):
    # Sample index j(i) from discrete distribution given by w_t-1
    sorted_particles = dict(sorted(particles.items(), key=lambda x: x[1]['weight']))
    norm_factor = 0
    for i in range(len(particles.keys())):
        sample_index = random.uniform(0, 1)
        weight_prob = 0
        for key in sorted_particles.keys():
            weight_prob += sorted_particles[key]['weight']
            if sample_index < weight_prob:
                particles[i]['trajectory'].append(sorted_particles[key]['trajectory'][-1]) #Sample new point based on weight distribution
                particles[i]['weight'] = sorted_particles[key]['weight'] # Compute importance weight (Def wrong but IDK how to do this)
                norm_factor += sorted_particles[key]['weight'] # Update Normalization Factor
                break
        # if sample_index is larger than any weight it is set to the largest particle weight
        particles[i]['trajectory'].append(sorted_particles[key]['trajectory'][-1]) 
        particles[i]['weight'] = sorted_particles[key]['weight']
        norm_factor += sorted_particles[key]['weight']
    # Update particles      
    for i in range(len(particles.keys())):
        odometry_noise_v = random.gauss(0, odometry_noise[0])                             # ADDING NOISE
        odometry_noise_theta = random.gauss(0, odometry_noise[1])                         # ADDING NOISE
        particles[i]["trajectory"].append(euler_integration(particles[i]["trajectory"][-1], [control[0] + odometry_noise_v, control[1] + odometry_noise_theta], dt))
        # Normalize Weights
        particles[i]['weight'] = particles[i]['weight']/norm_factor
    
    return particles

def calculate_new_position(initial_position, distance, angle_radians):
    # Convert angle from degrees to radians
    #angle_radians = math.radians(angle_degrees)
    
    # Calculate the new position using trigonometry
    new_x = initial_position[0] + distance * math.cos(angle_radians)
    new_y = initial_position[1] + distance * math.sin(angle_radians)
    
    return [new_x, new_y, 0]

def scene_from_file(filename):
    """Reads the scene from a JSON file."""
    with open(filename, 'r') as file:
        return json.load(file)
    
def observe_landmark(robot_position, landmark):
    # print("Start")
    # print(landmark)
    # print("Finish")
    landmark_x = float(landmark[0])
    landmark_y = float(landmark[1])
    distance = np.sqrt(np.square(robot_position[0]-landmark_x) + np.square(robot_position[1]-landmark_y))
    angle = np.arctan((robot_position[1] - landmark_y)/(robot_position[0] - landmark_x))
    return [distance, angle]
    
def visualize_scene(scene_file, sim_length):
    scene = scene_from_file(scene_file)  # Load the scene from the file
    viz_out = threejs_group(js_dir="../js")
    actual_trajectory = []
    observed_trajectory = []
    actual_robot = box('actual_robot', 2, 1, 1, [0, 0, .5], [1, 0, 0, 0])
    observed_robot = box('observed_robot', 2, 1, 1, [0, 0, .5], [1, 0, 0, 0])
    #------------------------------------------------------------------
    # Parse control input
    v, phi = map(float, control_input)
    control = (v, phi)
    initial_state = np.array([0, 0, 0])  # Initial state
    
    
    # Initialize Particle Filter
    num_particles = 100
    # particles = [[initial_state, 1]]*num_particles
    particles = {}
    for i in range(num_particles):
        particles[i] = {"trajectory": [initial_state], "weight": 1/num_particles}
    # Prepare the animation data
    actual_position_data = []
    observed_position_data = []
    observed_landmark_position_data = []
    
    # Initialize obstacle dict and trajectories from the loaded scene
    obstacle_dict = {}
    actual_obstacle_trajectories = {}
    observed_obstacle_trajectories = {}
    for obj in scene:
        if obj['type'] == 'sphere':
            key = obj['name']
            radius = obj['radius']
            position = obj['position']
            obstacle_dict[key] = {'radius': radius, 'position': position}
            actual_obstacle_trajectories[key] = []
            observed_obstacle_trajectories[key] = []

    # Simulate trajectory
    actual_trajectory, control, observed_trajectory, obstacle_dict, observed_obstacle_trajectories, particles = simulate_trajectory(initial_state, control, total_time, dt, actuation_noise, odometry_noise, observation_noise, obstacle_dict, observed_obstacle_trajectories, particles)
    
    #print(observed_obstacle_trajectories["sphere_0"])
    #Actual Location---------------------------------------------------------
    for t, point in enumerate(actual_trajectory):
        # robot_state = [t, robot.position, [1, 0, 0, 0], "0x0000ff"]
        # trajectory.append(robot_state)
        position = [point[0], point[1], 0.5]  # z=0.5 for flat surface movement
        quaternion = [1, 0, 0, 0]  # Simplified quaternion for no tilt/roll
        color = "0xff0000"  # Red color; adjust as needed
        state = [t * dt, position, quaternion, color]
        actual_position_data.append(state)                                # Actual Location
        for key, value in obstacle_dict.items():
            actual_obstacle_trajectories[key].append([t, value['position'], [1, 0, 0, 0], "0xff0000"])
    #Observed Location------------------------------------------------------------------------
    for t, point in enumerate(observed_trajectory):
        # robot_state = [t, robot.position, [1, 0, 0, 0], "0x0000ff"]
        # trajectory.append(robot_state)
        #Actual Location---------------------------------------------------------
        position = [point[0], point[1], 0.5]  # z=0.5 for flat surface movement
        quaternion = [1, 0, 0, 0]  # Simplified quaternion for no tilt/roll
        color = "#0000FF"  # Blue color; adjust as needed
        state = [t * dt, position, quaternion, color]
        #print(state)
        observed_position_data.append(state)                                # Observed Location
    
    for key in particles.keys():
        particles_animation = []
        for t, point in enumerate(particles[key]['trajectory']):
            position = [point[0], point[1], 0.5]  # z=0.5 for flat surface movement
            quaternion = [1, 0, 0, 0]  # Simplified quaternion for no tilt/roll
            color = "#0000FF"  # Blue color; adjust as needed
            state = [t * dt, position, quaternion, color]
            #print(state)
            particles_animation.append(state)                                # Observed Location
        particle = sphere("point", .1, particles_animation[0][1], particles_animation[0][2])
        viz_out.add_animation(particle, particles_animation)
            
    # Add spheres with their trajectories to visualization
    for key, obs_trajectory in actual_obstacle_trajectories.items():
        radius = obstacle_dict[key]['radius']
        actual_sphere_obstacle = sphere(key, radius, obs_trajectory[0][1], obs_trajectory[0][2])
        viz_out.add_animation(actual_sphere_obstacle, obs_trajectory)
        # for i in obs_trajectory:
        #     print("Key: ", key, i)
    
    for key, observed_obs_trajectory in observed_obstacle_trajectories.items():
        radius = obstacle_dict[key]['radius']
        observation_sphere_obstacle = sphere(key, radius, observed_obs_trajectory[0][1], observed_obs_trajectory[0][2])
        viz_out.add_animation(observation_sphere_obstacle, observed_obs_trajectory)
        # for i in observed_obs_trajectory:
        #     print("Key: ", key, i)
        
        
    viz_out.add_animation(observed_robot, observed_position_data)
    viz_out.add_animation(actual_robot, actual_position_data)
    viz_out.to_html(f"{os.path.splitext(scene_file)[0]}_visualization.html", "out/")

if __name__ == "__main__":
    sim_length = 10
    scene_files = [
        "landmark_0.json",
        #"landmark_1.json",
        #"landmark_2.json",
        #"landmark_3.json",
        #"landmark_4.json"
    ]

    if len(sys.argv) != 3:
            print("Usage: python Assignment2_visualize_car.py <velocity> <steering angle>")
    else:
        control_input = sys.argv[1:3]
        for scene_file in scene_files:
            visualize_scene(scene_file, sim_length)
            print(f"Visualization generated for {scene_file}")