import os

def rename_images(folder_path, remove_text):
    """
    Renombra todas las imágenes en una carpeta eliminando un texto específico del nombre.
    
    :param folder_path: Ruta a la carpeta con las imágenes.
    :param remove_text: Texto a eliminar del nombre de los archivos.
    """
    if not os.path.isdir(folder_path):
        print("La carpeta especificada no existe.")
        return
    
    for filename in os.listdir(folder_path):
        old_path = os.path.join(folder_path, filename)
        
        if os.path.isfile(old_path):
            new_filename = filename.replace(remove_text, "")
            new_path = os.path.join(folder_path, new_filename)
            
            if old_path != new_path:
                os.rename(old_path, new_path)
                print(f"Renombrado: {filename} -> {new_filename}")
            
if __name__ == "__main__":
    folder = "/home/luis/datasets/SynColon/juanjo/seq0/depth_exr"
    text_to_remove = "aov_image_"
    rename_images(folder, text_to_remove)