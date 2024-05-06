import os
import numpy as np

# Create the maps directory if it doesn't exist
if not os.path.exists('maps_dir'):
    os.makedirs('maps_dir')

# Function to generate random landmarks
def generate_landmarks(num_landmarks):
    landmarks = np.random.rand(num_landmarks, 2) * 25  # Assuming a 100x100 environment
    return landmarks

# Function to save landmarks to a text file
def save_landmarks(filename, landmarks):
    np.savetxt(filename, landmarks, fmt='%f', delimiter=' ')

# Generate and save maps
maps_info = [(5, 'landmark_0.txt'), (5, 'landmark_1.txt'), (8, 'landmark_2.txt'), (12, 'landmark_3.txt'), (12, 'landmark_4.txt')]

for num_landmarks, filename in maps_info:
    landmarks = generate_landmarks(num_landmarks)
    save_landmarks(os.path.join('maps_dir', filename), landmarks)
