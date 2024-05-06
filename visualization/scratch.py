# for i in range(5):
#     maps = open(f"maps_dir/landmark_{i}.txt")
#     map_list = str.split(maps.read(), "\n")
#     map_list.pop()
#     for i in map_list:
#         print(len(map_list))
# #         x = str.split(i)[0]
# #         y = str.split(i)[1]

import numpy as np

# # Initialize Particle Filter
initial_state = np.array([0, 0, 0])  # Initial state
# num_particles = 3
# particles = [[initial_state, 1]]*num_particles
# print(particles)

# Initialize Particle Filter
num_particles = 10
# particles = [[initial_state, 1]]*num_particles
particles = {}
for i in range(num_particles):
    particles[i] = {"trajectory": [initial_state], "weight": 1}
print(particles)