#!/usr/bin/env python3
import numpy as np

def generate_points(num_points, rigid_movement, gaussian_movement, x_mean, x_std, y_mean, y_std, z_mean, z_std, angle_x=0, angle_y=0, angle_z=0):
    # Generate original points based on mean and standard deviation for each coordinate
    original_points = np.zeros((num_points, 3))
    original_points[:, 0] = np.random.normal(0.0, x_std, num_points)
    original_points[:, 1] = np.random.normal(0.0, y_std, num_points)
    original_points[:, 2] = np.random.normal(0.0, z_std, num_points)
    
    # Rotate original points
    original_points = rotate_points(original_points, angle_x, angle_y, angle_z)

    original_points[:, 0] += x_mean
    original_points[:, 1] += y_mean
    original_points[:, 2] += z_mean
    
    # Move along yz-axis
    moved_points = original_points.copy()
    moved_points[:, 1] += rigid_movement
    moved_points[:, 1] += np.random.normal(scale=gaussian_movement, size=num_points)
    moved_points[:, 2] -= rigid_movement
    moved_points[:, 2] -= np.random.normal(scale=gaussian_movement, size=num_points)
    
    return original_points, moved_points

def rotate_points(points, angle_x, angle_y, angle_z):
    # Convert angles from degrees to radians
    angle_x = np.deg2rad(angle_x)
    angle_y = np.deg2rad(angle_y)
    angle_z = np.deg2rad(angle_z)
    
    # Rotation matrices
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(angle_x), -np.sin(angle_x)],
                   [0, np.sin(angle_x), np.cos(angle_x)]])
    
    Ry = np.array([[np.cos(angle_y), 0, np.sin(angle_y)],
                   [0, 1, 0],
                   [-np.sin(angle_y), 0, np.cos(angle_y)]])
    
    Rz = np.array([[np.cos(angle_z), -np.sin(angle_z), 0],
                   [np.sin(angle_z), np.cos(angle_z), 0],
                   [0, 0, 1]])
    
    # Combined rotation matrix
    R = Rz @ Ry @ Rx
    
    # Rotate points
    rotated_points = points @ R.T
    
    return rotated_points

def save_points(filename, points):
    np.savetxt(filename, points, delimiter=' ')

num_points = 200  # Number of points in the dataset

# Original points position
x_mean, x_std = 0.0, 0.2
y_mean, y_std = 0.0, 0.004
z_mean, z_std = 1.1, 0.06
# Rotation angles
angle_x, angle_y, angle_z = -45, -0, 45  # in degrees

# Movement
rigid_movement = 0.03  # Scale of movement for the moved points
gaussian_movement = 0.005  # Scale of movement for the moved points

original_points, moved_points = generate_points(num_points, rigid_movement, gaussian_movement, x_mean, x_std, y_mean, y_std, z_mean, z_std, angle_x, angle_y, angle_z)

save_points('Data/original_points.csv', original_points)
save_points('Data/moved_points.csv', moved_points)
