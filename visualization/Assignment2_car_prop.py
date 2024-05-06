import numpy as np
import random
import sys
from geometry import box
from threejs_group import threejs_group

# Constants
L = 1.5  # Wheelbase of the car
dt = 0.1  # Time step
total_time = 10  # Duration of simulation
noise = [0, .5] #[v, theta]

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
def simulate_trajectory(initial_state, control, total_time, dt, noise):
    timesteps = int(total_time / dt)
    trajectory = np.zeros((timesteps+1, 3))
    trajectory[0] = initial_state
    for t in range(1, timesteps+1):
        acctuation_noise_v = random.gauss(0, noise[0])                             # ADDING NOISE
        acctuation_noise_theta = random.gauss(0, noise[1])                         # ADDING NOISE
        observation_noise_v = random.gauss(0, noise[0])                             # ADDING NOISE
        observation_noise_theta = random.gauss(0, noise[1])                         # ADDING NOISE
        observation = [control[0] + acctuation_noise_v, control[1] + acctuation_noise_theta]
        trajectory[t] = euler_integration(trajectory[t-1], [control[0] + acctuation_noise_v, control[1] + acctuation_noise_theta], dt)
        
    return trajectory, observation

def visualize_trajectory_with_car(viz, trajectory, car_name="car"):
    # Prepare the animation data
    animation_data = []
    for t, point in enumerate(trajectory):
        position = [point[0], point[1], 0.5]  # z=0.5 for flat surface movement
        quaternion = [1, 0, 0, 0]  # Simplified quaternion for no tilt/roll
        color = "0xff0000"  # Red color; adjust as needed
        state = [t * dt, position, quaternion, color]
        animation_data.append(state)

    # Create the car object with initial position and orientation
    car = box(car_name, 2, 1, 1, animation_data[0][1], animation_data[0][2])

    # Add the car's animation to the visualization
    viz.add_animation(car, animation_data)

def main(control_input):
    # Parse control input
    v, phi = map(float, control_input)
    control = (v, phi)
    initial_state = np.array([0, 0, 0])  # Initial state

    # Simulate trajectory
    trajectory, observation = simulate_trajectory(initial_state, control, total_time, dt, noise)

    # Initialize visualization
    viz = threejs_group(canvas_width=800, canvas_height=600, js_dir="../js/")

    # Visualize the car's trajectory with animation
    visualize_trajectory_with_car(viz, trajectory, car_name="car")

    # Output the HTML file for the visualization
    viz.to_html("car_trajectory.html", "out/")  # Adjust the path as necessary

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python Assignment2_car_prop.py <velocity> <steering angle>")
    else:
        control_input = sys.argv[1:3]
        main(control_input)