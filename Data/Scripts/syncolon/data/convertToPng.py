import OpenEXR
import Imath
import numpy as np
import imageio
import os

# Folder paths
exr_folder = '/home/luis/datasets/SynColon/625/level0/depth_exr/'
png_folder = '/home/luis/datasets/SynColon/625/level0/depth/'

# Create the output folder if it doesn't exist
if not os.path.exists(png_folder):
    os.makedirs(png_folder)

# List all EXR files in the directory that match the pattern (e.g., 0004.exr, 0005.exr, etc.)
exr_files = [f for f in os.listdir(exr_folder) if f.endswith('.exr')]

# Loop over the EXR files and convert each to PNG
for exr_file_name in exr_files:
#exr_file_name = "0104.exr"
    exr_path = os.path.join(exr_folder, exr_file_name)
    png_output_path = os.path.join(png_folder, exr_file_name.replace('.exr', '.png'))

    # Open EXR file using OpenEXR
    exr_file = OpenEXR.InputFile(exr_path)

    # Get image size and channels
    dw = exr_file.header()['dataWindow']
    channels = exr_file.header()['channels']

    width = dw.max.x - dw.min.x + 1
    height = dw.max.y - dw.min.y + 1

    print(f" - Image Size: {width} x {height}")
    print(f" - Channels Available: {list(channels.keys())}")

    # Read the depth data (assuming 'R' channel holds the depth)
    depth_data = exr_file.channel('R', Imath.PixelType(Imath.PixelType.HALF))

    # min_val, max_val = np.min(depth_data), np.max(depth_data)
    # print(f" - Raw Depth Min: {min_val}, Max: {max_val}")

    # Convert to numpy array
    depth = np.frombuffer(depth_data, dtype=np.float16)
    depth = depth.reshape((dw.max.y - dw.min.y + 1, dw.max.x - dw.min.x + 1))

    min_val, max_val = np.min(depth), np.max(depth)
    print(f" - Raw Depth Min: {min_val}, Max: {max_val}")

    # Scale the depth values to 16-bit range using the exact scale factor you provided
    scale_factor = 0.5 / (pow(2, 16) - 1) # 0.2
    scaled_depth = np.clip(depth / scale_factor, 0, 65535).astype(np.uint16)

    scaled_min, scaled_max = np.min(scaled_depth), np.max(scaled_depth)
    print(f" - Scaled Depth Min: {scaled_min}, Max: {scaled_max}")

    # Save as PNG using imageio
    imageio.imwrite(png_output_path, scaled_depth)

    print(f"PNG saved at {png_output_path}")
