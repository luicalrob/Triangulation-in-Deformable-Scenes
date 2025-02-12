import cv2
import os

# Paths for input and output folders
input_folder = "path/to/input/folder"
output_folder = "path/to/output/folder"

# Ensure output folder exists
os.makedirs(output_folder, exist_ok=True)

# New resolution
new_width = 256
new_height = 256

# Process all images in the input folder
for filename in os.listdir(input_folder):
    input_path = os.path.join(input_folder, filename)
    output_path = os.path.join(output_folder, filename)

    # Read image as grayscale (unchanged to preserve depth values)
    depth_image = cv2.imread(input_path, cv2.IMREAD_UNCHANGED)

    if depth_image is None:
        print(f"Skipping {filename}: unable to read.")
        continue

    # Resize using nearest neighbor interpolation
    resized_depth = cv2.resize(depth_image, (new_width, new_height), interpolation=cv2.INTER_NEAREST)

    # Save resized image
    cv2.imwrite(output_path, resized_depth)
    print(f"Resized and saved: {output_path}")

print("All images processed successfully!")
