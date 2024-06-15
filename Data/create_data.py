#!/usr/bin/env python3
import numpy as np

def generate_points(num_points, movement_scale=0.1):
    # Generate original set of 3D points
    original_points = np.random.rand(num_points, 3)  # Generate random 3D points

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
original_points, moved_points = generate_points(num_points, movement_scale)

save_points('Data/original_points.csv', original_points)
save_points('Data/moved_points.csv', moved_points)
