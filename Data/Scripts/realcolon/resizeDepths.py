import os
import numpy as np
import cv2
from PIL import Image

# Paths
background_path = "/home/luis/datasets/RealColon/58/mask_border_endo_ori.jpg"
input_folder = "/home/luis/datasets/RealColon/58/depth_npy"
output_folder = "/home/luis/datasets/RealColon/58/depth"

# Ensure output directory exists
os.makedirs(output_folder, exist_ok=True)

# Constants
SCALE_FACTOR = 30.0 / (pow(2, 16) - 1)
FINAL_HEIGHT = 1080
FINAL_WIDTH = 1440
DEPTH_HEIGHT = 992
DEPTH_WIDTH = 1344

# Load mask image and convert it to grayscale (to detect border areas)
mask = cv2.imread(background_path, cv2.IMREAD_GRAYSCALE)
if mask is None:
    raise FileNotFoundError(f"Mask image not found at {background_path}")

# Ensure the mask is the correct size
mask = cv2.resize(mask, (FINAL_WIDTH, FINAL_HEIGHT))

# Threshold the mask to identify border regions (set to 0 where the mask is nonzero)
border_mask = (mask == 0).astype(np.uint16)  # 1 where valid, 0 where border

# Get top-left coordinates to center depth image
top = (FINAL_HEIGHT - DEPTH_HEIGHT) // 2
left = (FINAL_WIDTH - DEPTH_WIDTH) // 2

# Process each .npy depth file
for filename in os.listdir(input_folder):
    if filename.endswith(".npy"):
        depth_path = os.path.join(input_folder, filename)
        output_path = os.path.join(output_folder, filename.replace(".npy", ".png"))

        # Load .npy depth image
        depth = np.load(depth_path)
        depth = np.squeeze(depth)  # Remove extra dimensions
        # print("Depth map resolution:", depth.shape) 

        if depth.shape != (DEPTH_HEIGHT, DEPTH_WIDTH):
            raise ValueError(f"Unexpected depth image shape {depth.shape}, expected {(DEPTH_HEIGHT, DEPTH_WIDTH)}")

        final_image = np.ones((FINAL_HEIGHT, FINAL_WIDTH), dtype=np.uint16)

        # Ensure depth is in uint16 format before inserting
        scale_factor = 30.0 / (pow(2, 16) - 1) 
        depth_uint16 = np.clip(depth / scale_factor, 0, 65535).astype(np.uint16)

        # Now assign the properly formatted depth image
        final_image[top:top + DEPTH_HEIGHT, left:left + DEPTH_WIDTH] = depth_uint16

        # Apply border mask (set border regions to 0)
        final_image *= border_mask

        # Save as 16-bit PNG
        Image.fromarray(final_image).save(output_path)

print("Processing complete! Depth images saved.")
