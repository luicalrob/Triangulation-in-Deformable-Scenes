import numpy as np
import cv2
import os

# Set input and output folders
input_folder = "/home/luis/datasets/RealColon/58/depth_npy"
output_folder = "/home/luis/datasets/RealColon/58/depth"

# Ensure output folder exists
os.makedirs(output_folder, exist_ok=True)

# Process all .npy files in the input folder
for filename in os.listdir(input_folder):
    if filename.endswith(".npy"):
        input_path = os.path.join(input_folder, filename)
        output_path = os.path.join(output_folder, filename.replace(".npy", ".png"))

        print(f"Processing: {input_path}")

        # Load the depth file
        depth_array = np.load(input_path)

        depth_array = np.squeeze(depth_array)  # Remove extra dimensions
        depth_array = depth_array  # Transpose to (1344, 992)
        print("Depth map resolution:", depth_array.shape) 

        # Normalize and convert to 16-bit
        scale_factor = 30.0 / (pow(2, 16) - 1) 
        depth_uint16 = np.clip(depth_array / scale_factor, 0, 65535).astype(np.uint16)

        # Save as a 16-bit PNG
        cv2.imwrite(output_path, depth_uint16)
        print(f"Saved: {output_path}")

print("✅ Batch processing complete!")
