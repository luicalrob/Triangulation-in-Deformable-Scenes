#!/usr/bin/env python3
import numpy as np

def generate_points(num_points, scale_movement, x_mean, x_std, y_mean, y_std, z_mean, z_std):
    # Generate original points based on mean and standard deviation for each coordinate
    original_points = np.zeros((num_points, 3))
    original_points[:, 0] = np.random.normal(x_mean, x_std, num_points)
    original_points[:, 1] = np.random.normal(y_mean, y_std, num_points)
    original_points[:, 2] = np.random.normal(z_mean, z_std, num_points)
    
    # Move randomly
    # moved_points = original_points + np.random.normal(scale=movement_scale, size=(num_points, 3))

    # Move along x-axis
    moved_points = original_points.copy()
    moved_points[:, 0] += np.random.normal(scale=movement_scale, size=num_points)  
    return original_points, moved_points

def save_points(filename, points):
    np.savetxt(filename, points, delimiter=' ')

# Generate and save original and moved points
num_points = 20  # Number of points in the dataset
movement_scale = 0.1  # Scale of movement for the moved points

# Original points position
x_mean = 0.05
x_std = 0.05
y_mean = 0.5
y_std = 0.1
z_mean = 0.0
z_std = 0.05

original_points, moved_points = generate_points(num_points, movement_scale, x_mean, x_std, y_mean, y_std, z_mean, z_std)

save_points('Data/original_points.csv', original_points)
save_points('Data/moved_points.csv', moved_points)
