import OpenEXR
import Imath
import numpy as np

def leer_exr(archivo, canal="R"):
    exr = OpenEXR.InputFile(archivo)
    dw = exr.header()["dataWindow"]
    ancho = dw.max.x - dw.min.x + 1
    alto = dw.max.y - dw.min.y + 1
    
    tipo_float = Imath.PixelType(Imath.PixelType.HALF)  # 16-bit float
    datos = exr.channel(canal, tipo_float)
    arr = np.frombuffer(datos, dtype=np.float16).reshape(alto, ancho)
    
    return arr

# Ruta de la imagen EXR
imagen = "0080.exr"

# Leer valores de los canales disponibles
for canal in ["R", "G", "B", "A"]:  # Puedes agregar "Z" o "Depth" si existen
    try:
        valores = leer_exr(imagen, canal)
        print(f"\nValores del canal {canal}:")
        print(valores)
    except KeyError:
        print(f"El canal {canal} no está presente en la imagen.")
