import sys
import os
import pandas as pd
import matplotlib.pyplot as plt
import glob
import re
import numpy as np
from collections import defaultdict
from scipy.interpolate import make_interp_spline
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import syncolon_values, syncolon_level_types, setSyncolonParameters

default_values = syncolon_values

# Configuración de los archivos y parámetros
folder_path = "./Data/Excels/Syncolon/Resumes"
triangulation_type = "FarPoints"  # O "TwoPoints"
data_type = "Final"  # O "Final"
file_range = range(66, 71)

data = {}

# Leer los archivos y procesar los datos
for i in file_range:
    file_pattern = f"{folder_path}/Data {i} {triangulation_type} {data_type}.csv"
    files = glob.glob(file_pattern)

    print(f"\n🔍 Buscando archivos con patrón: {file_pattern}")
    print(f"   📂 Archivos encontrados: {files}")

    # Cargar el archivo de experimentos correspondiente
    experiment_file = f"./Data/Excels/Syncolon/Resumes/Experiment {triangulation_type} {i}.csv"
    try:
        experiment_data = pd.read_csv(experiment_file, delimiter=",", dtype=str)
        experiment_data.columns = [col.strip() for col in experiment_data.columns]
        print(f"✅ Cargado archivo de experimentos: {experiment_file}")
    except FileNotFoundError:
        print(f"⚠️ Archivo de experimentos no encontrado: {experiment_file}")
        continue

    for file in files:
        # print(f"\n📖 Leyendo archivo: {file}")
        df = pd.read_csv(file, delimiter=",", dtype=str)
        df.columns = [col.strip() for col in df.columns]  # Limpieza de nombres de columnas

        # Convertir comas decimales a puntos y valores numéricos
        df["parallax"] = df["parallax"].str.replace(",", ".").astype(float)
        df["Av. error (mm)"] = df["Av. error (mm)"].str.replace(",", ".").astype(float)

        # Procesar cada fila
        for index, row in df.iterrows():
            match = re.match(r"(seq\d+)_(checks|no_checks)", row["Info"])
            if match:
                seq, check_type = match.groups()

                # Extraer la pareja de imágenes desde Higher Info
                pattern = fr"(\d+-\d+-\d+)_{triangulation_type}"
                higher_info_match = re.match(pattern, row["Higher Info"])
                if not higher_info_match:
                    continue

                pair = higher_info_match.group(1)  # Ejemplo: "105-2-115"
                # print(f"🔹 Extraída pareja de frames: {pair} de Higher Info: {row['Higher Info']}")

                # Buscar la columna de t C1C2 norm en el archivo de experimentos
                col_t = f"{pair}-{triangulation_type} t C1C2 norm (mm)"
                if col_t not in experiment_data.columns:
                    # print(f"⚠️ Columna {col_t} no encontrada en el archivo de experimentos")
                    continue

                # Filtrar la fila correcta
                t_values = experiment_data.loc[
                    (experiment_data.iloc[:, 0] == seq) & 
                    (experiment_data.iloc[:, 1] == check_type), 
                    col_t
                ]

                if not t_values.empty:
                    try:
                        t_value = float(t_values.values[0].replace(",", "."))
                        # print(f"   ✅ Encontrado t_value: {t_value} en columna {col_t} para {seq} {check_type}")
                    except ValueError:
                        # print(f"⚠️ No se pudo convertir a número: {t_values.values[0]}")
                        continue

                    adjusted_error = row["Av. error (mm)"] / t_value  # Error normalizado
                    # print(f"   🔸 Av. error (mm): {row['Av. error (mm)']} → Normalizado: {adjusted_error}")

                    # Guardar los datos en el diccionario
                    key = f"{seq}_{check_type}"
                    if key not in data:
                        data[key] = {"parallax": [], "error": []}
                    
                    data[key]["parallax"].append(row["parallax"])
                    data[key]["error"].append(adjusted_error)

# Graficar los datos con líneas más limpias y sin dispersión de puntos
plt.figure(figsize=(10, 6))
colors = ["blue", "red", "green", "purple", "orange", "brown", "black", "cyan"]

data_plotted = False
for i, (seq, values) in enumerate(data.items()):
    if values["parallax"]:
        # Ordenar valores
        parallax, error = zip(*sorted(zip(values["parallax"], values["error"])))

        # 🔹 Agrupar datos en intervalos clave de parallax
        num_intervals = 8  # Ajustable: Reduce el número de puntos clave en la curva
        min_p, max_p = min(parallax), max(parallax)
        interval_bounds = np.linspace(min_p, max_p, num_intervals)

        grouped_parallax = []
        avg_errors = []
        for j in range(len(interval_bounds) - 1):
            lower, upper = interval_bounds[j], interval_bounds[j + 1]
            indices = [k for k, p in enumerate(parallax) if lower <= p < upper]
            
            if indices:
                grouped_parallax.append(np.mean([parallax[k] for k in indices]))
                avg_errors.append(np.mean([error[k] for k in indices]))

        # 🔹 Suavizar la curva con interpolación polinómica si hay suficientes puntos
        if len(grouped_parallax) > 2:
            trend_smooth = np.linspace(min(grouped_parallax), max(grouped_parallax), 50)
            spline = make_interp_spline(grouped_parallax, avg_errors, k=2)
            error_smooth = spline(trend_smooth)
            plt.plot(trend_smooth, error_smooth, label=seq, color=colors[i % len(colors)], linewidth=2)
        else:
            # 🔸 Si hay pocos puntos, conectar con líneas rectas
            plt.plot(grouped_parallax, avg_errors, label=seq, color=colors[i % len(colors)], linewidth=2, linestyle="--")

        data_plotted = True

if not data_plotted:
    print("❌ No se encontraron datos para graficar.")
else:
    plt.xlabel("Parallax (degrees)")
    plt.ylabel("Normalized Error (Av. error / t C1C2 norm)")
    plt.title(f"Normalized Error vs Parallax ({triangulation_type} - {data_type})")
    plt.legend()
    plt.grid()
    plt.show()

