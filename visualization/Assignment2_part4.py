import numpy as np

# Function to calculate the distance from point q to the nearest obstacle
def distance_to_nearest_obstacle(q, obstacles):
    min_distance = float('inf')
    for obstacle in obstacles:
        distance = np.linalg.norm(np.array(q) - np.array(obstacle))
        min_distance = min(min_distance, distance)
    return min_distance

# Gradient of the attraction potential function
def attraction_gradient(q, goal, alpha):
    return alpha * (np.array(q) - np.array(goal))

# Gradient of the repulsion potential function
def repulsion_gradient(q, obstacles, rho_0, eta):
    rho_q = distance_to_nearest_obstacle(q, obstacles)
    if rho_q <= rho_0:
        grad = np.zeros_like(q)
        for i in range(len(q)):
            diff = q[i] - obstacles[np.argmin([np.linalg.norm(np.array(q) - np.array(obstacle)) for obstacle in obstacles])][i]
            grad[i] = eta * (1 - rho_q / rho_0) * (diff / np.linalg.norm(np.array(q) - np.array(obstacles[np.argmin([np.linalg.norm(np.array(q) - np.array(obstacle)) for obstacle in obstacles])])))**2
        return grad
    else:
        return np.zeros_like(q)

# Combined gradient of the potential function
def potential_gradient(q, goal, obstacles, alpha, eta, rho_0):
    return attraction_gradient(q, goal, alpha) + repulsion_gradient(q, obstacles, rho_0, eta)

# Example obstacle region
obstacles = [(2, 3), (3, 5), (5, 7)]
goal = (8, 8)

# Example parameters
alpha = 1.0
eta = 1.0
rho_0 = 1.0

# Test the potential gradient function
q = (1, 1)
print("Gradient at q:", potential_gradient(q, goal, obstacles, alpha, eta, rho_0))
