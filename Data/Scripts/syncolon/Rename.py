import os

def rename_images(folder_path):
    for filename in os.listdir(folder_path):
        if filename.startswith("image_") and filename.endswith(".png"):
            new_name = filename.replace("image_", "", 1)
            old_path = os.path.join(folder_path, filename)
            new_path = os.path.join(folder_path, new_name)
            os.rename(old_path, new_path)
            print(f'Renombrado: {filename} -> {new_name}')

if __name__ == "__main__":
    folder_path = input("/home/luis/datasets/SynColon/juanjo/seq0/rgb").strip()
    if os.path.isdir(folder_path):
        rename_images(folder_path)
    else:
        print("La ruta especificada no es válida.")
