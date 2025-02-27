import os
import numpy as np
import cv2

# Paths
background_path = "/home/luis/datasets/RealColon/mask_border_endo_ori.jpg"
input_folder = "/home/luis/datasets/RealColon/27/depth_npy"
output_folder = "/home/luis/datasets/RealColon/27/depth_npy_output"

# Ensure output directory exists
os.makedirs(output_folder, exist_ok=True)

# Constants
FINAL_HEIGHT = 1080
FINAL_WIDTH = 1440
DEPTH_HEIGHT = 992
DEPTH_WIDTH = 1344

# Load mask image and convert it to grayscale (to detect border areas)
mask = cv2.imread(background_path, cv2.IMREAD_GRAYSCALE)
if mask is None:
    raise FileNotFoundError(f"Mask image not found at {background_path}")

# Ensure the mask is the correct size
print("Mask shape:", mask.shape)

# Threshold the mask to identify border regions (set to 0 where the mask is nonzero)
border_mask = (mask == 0).astype(np.uint16)  # 1 where valid, 0 where border

# Get top-left coordinates to center depth image
top = (FINAL_HEIGHT - DEPTH_HEIGHT) // 2
left = (FINAL_WIDTH - DEPTH_WIDTH) // 2

# Process each .npy depth file
filename = "0000.npy"
if filename.endswith(".npy"):
    depth_path = os.path.join(input_folder, filename)
    output_path = os.path.join(output_folder, filename)

    print(f"Processing: {depth_path}")

    # Load .npy depth image
    depth = np.load(depth_path)
    depth = np.squeeze(depth)  # Remove extra dimensions
    print("Depth map resolution:", depth.shape)
    
    if depth.shape != (DEPTH_HEIGHT, DEPTH_WIDTH):
        raise ValueError(f"Unexpected depth image shape {depth.shape}, expected {(DEPTH_HEIGHT, DEPTH_WIDTH)}")

    # Create final depth image with zeros (border areas remain zero)
    final_image = np.zeros((FINAL_HEIGHT, FINAL_WIDTH), dtype=depth.dtype)
    
    # Insert the original depth image centered
    final_image[top:top + DEPTH_HEIGHT, left:left + DEPTH_WIDTH] = depth

    # Apply border mask (ensuring borders remain zero)
    final_image *= border_mask

    # Save as .npy
    np.save(output_path, final_image)
    # cv2.imwrite("imagen.png", img)
    print("Final image shape:", final_image.shape)
    print(f"Saved: {output_path}")

print("Processing complete! Depth images saved as .npy.")
