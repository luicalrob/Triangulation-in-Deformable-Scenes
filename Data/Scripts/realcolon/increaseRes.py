import cv2
import os

# Paths for input and output folders
input_folder = "/home/luis/datasets/RealColon/01/depth_resize"
output_folder = "/home/luis/datasets/RealColon/01/depth"

# Ensure output folder exists
os.makedirs(output_folder, exist_ok=True)

# New resolution (adjust as needed)
scale_factor = 720/496  # Increase resolution by 2x
interpolation_method = cv2.INTER_NEAREST  # Recommended for depth images

# Process all images in the input folder
for filename in os.listdir(input_folder):
    input_path = os.path.join(input_folder, filename)
    output_path = os.path.join(output_folder, filename)

    # Read image as grayscale (unchanged to preserve depth values)
    depth_image = cv2.imread(input_path, cv2.IMREAD_UNCHANGED)

    if depth_image is None:
        print(f"Skipping {filename}: unable to read.")
        continue

    # Get new dimensions
    new_width = int(depth_image.shape[1] * scale_factor)
    new_height = int(depth_image.shape[0] * scale_factor)

    # Resize using the chosen interpolation method
    upscaled_depth = cv2.resize(depth_image, (new_width, new_height), interpolation=interpolation_method)

    # Save resized image
    cv2.imwrite(output_path, upscaled_depth)
    print(f"Upscaled and saved: {output_path}")

print("All images upscaled successfully!")
