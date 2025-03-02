from selenium import webdriver
from selenium.webdriver.firefox.service import Service
from selenium.webdriver.firefox.options import Options
import time
import os

# Configurar Firefox en modo sin cabeza (headless)
options = Options()
options.headless = True  # No abrirá una ventana gráfica

# Ruta al driver de Firefox (geckodriver)
service = Service("/usr/bin/geckodriver")
driver = webdriver.Firefox(service=service, options=options)

# URL base
base_url = "http://155.210.155.84:8000/lightdepth?id={}"  # URL para cargar la página
download_url = "http://155.210.155.84:8000/static/PRED/depth.npy"  # URL del archivo

dest_folder = os.path.expanduser("~/datasets/RealColon/58/depth")  # Ruta destino
os.makedirs(dest_folder, exist_ok=True)  # Crear carpeta si no existe

for i in range(300):  # Iterar desde 0 hasta 299
    print(f"Procesando ID {i}...")
    driver.get(base_url.format(i))  # Abrir la URL en Firefox
    time.sleep(2)  # Esperar a que la página cargue completamente
    
    output_file = os.path.join(dest_folder, f"{i:04d}.npy")
    command = f"wget -O {output_file} {download_url}"  # Comando de descarga
    os.system(command)  # Ejecutar comando en la terminal
    
    print(f"Descarga completada: {output_file}")

driver.quit()  # Cerrar Firefox cuando termine
print("Proceso completado")
