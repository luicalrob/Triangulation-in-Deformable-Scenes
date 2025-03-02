import os
from PIL import Image

def crop_and_center(input_folder, output_folder):
    # Crear la carpeta de salida si no existe
    os.makedirs(output_folder, exist_ok=True)

    for filename in os.listdir(input_folder):
        if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.gif')):
            img_path = os.path.join(input_folder, filename)
            img = Image.open(img_path).convert("RGB")

            # Primer recorte: (top=0, left=70, width=1350, height=1080)
            img_cropped = img.crop((70, 0, 70 + 1350, 0 + 1080))

            # Segundo recorte: Centrar en (992, 1344)
            target_width, target_height = 1344, 992
            img_width, img_height = img_cropped.size
            left = (img_width - target_width) // 2
            top = (img_height - target_height) // 2
            right = left + target_width
            bottom = top + target_height

            img_final = img_cropped.crop((left, top, right, bottom))

            # Guardar la imagen procesada
            output_path = os.path.join(output_folder, filename)
            img_final.save(output_path)

            print(f"Procesado: {filename}")

# Rutas de los folders
input_folder = "/home/luis/datasets/RealColon/58/rgb_original"
output_folder = "/home/luis/datasets/RealColon/58/rgb"

crop_and_center(input_folder, output_folder)
