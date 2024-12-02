#!/usr/bin/env python3
import numpy as np

"""
Script to create data for the sintetic data base.

This script creates two sets of 3D points: the first with a specific 
shape and a second with a specified movement relative to the first.

Steps:
1. Select the Original points position. 
1. Select the Gaussian/Rigid movement. 
3. Choose between Planar, Gradual or Curvature movement.
4. Select a intensity of the Gradual or Curvature movement, if they have been choosen.
3. Run the script.
Example:
./Data/Scripts/create_data.py


Output:
- original_points.csv and moved_points.csv files are created

Author: Luis Calderón Robustillo
Date: 19/11/24
"""

def generate_points(num_points, rigid_movement, gaussian_movement, x_mean, x_std, y_mean, y_std, z_mean, z_std, 
                    angle_x=0, angle_y=0, angle_z=0, grad_direction='y', grad_intensity=1.0, gradual=False, curvature=False):
    original_points = np.zeros((num_points, 3))
    original_points[:, 0] = np.random.normal(0.0, x_std, num_points)
    original_points[:, 1] = np.random.normal(0.0, y_std, num_points)
    original_points[:, 2] = np.random.normal(0.0, z_std, num_points)
    
    moved_points = original_points.copy()
    
    axis_index = {'x': 0, 'y': 1, 'z': 2}[grad_direction]

    for i in range(num_points):
        grad_factor = grad_intensity * moved_points[i, 0]
        if curvature:
            moved_points[i, axis_index] += rigid_movement * (grad_factor*grad_factor)
        elif gradual:
            moved_points[i, axis_index] += rigid_movement * (1+grad_factor)
        else:
            moved_points[i, axis_index] += rigid_movement
        
        # moved_points[i, axis_index] += rigid_movement
        # moved_points[i, axis_index] += rigid_movement
    
        moved_points[i, 0] += np.random.normal(scale=gaussian_movement)
        moved_points[i, 1] += np.random.normal(scale=gaussian_movement)
        moved_points[i, 2] += np.random.normal(scale=gaussian_movement)

    original_points = rotate_points(original_points, angle_x, angle_y, angle_z)

    original_points[:, 0] += x_mean
    original_points[:, 1] += y_mean
    original_points[:, 2] += z_mean
    
    moved_points = rotate_points(moved_points, angle_x, angle_y, angle_z)

    moved_points[:, 0] += x_mean
    moved_points[:, 1] += y_mean
    moved_points[:, 2] += z_mean

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

num_points = 240  # Number of points in the dataset

# Original points position
x_mean, x_std = 0.0, 0.03
y_mean, y_std = 0.0, 0.001
z_mean, z_std = 0.2, 0.01
# Rotation angles
angle_x, angle_y, angle_z = -45, -0, 45  # in degrees

# Movement
rigid_movement = 0.01  # Scale of movement for the moved points
gaussian_movement = 0.00  # Scale of movement for the moved points
movement_type = "Planar" # "Planar" "Gradual or "Curvature"
gradual_and_curv_intensity = 25.0 # A significant value returns pronunciated gradual/curvature movements

Gradual = False
Curvature = False

if (movement_type == "Gradual"):
    Gradual = True
elif (movement_type == "Curvature"):
    Curvature = True

original_points, moved_points = generate_points(num_points, rigid_movement, gaussian_movement, x_mean, 
                                                x_std, y_mean, y_std, z_mean, z_std, angle_x, angle_y, angle_z,
                                                'y', gradual_and_curv_intensity, Gradual, Curvature)

save_points('./Data/original_points.csv', original_points)
save_points('./Data/moved_points.csv', moved_points)
