import sys
import os
import pandas as pd
import matplotlib.pyplot as plt
import glob
import re
import numpy as np
from collections import defaultdict
from scipy.interpolate import make_interp_spline

# Configuration
folder_path = "./Data/Excels/Syncolon/Resumes"
selected_triangulation = "FarPoints"  # Ahora fijo en FarPoints
midpoint_triangulation = "TwoPoints"
file_range = range(66, 71)

data = {"Initial": defaultdict(lambda: {"parallax": [], "error": []}),
        "Final": defaultdict(lambda: {"parallax": [], "error": []}),
        "MidPoint": defaultdict(lambda: {"parallax": [], "error": []})}

# Read and process files for Initial, Final, and MidPoint
for data_type in ["Initial", "Final"]:
    for i in file_range:
        file_pattern = f"{folder_path}/Data {i} {selected_triangulation} {data_type}.csv"
        files = glob.glob(file_pattern)
        
        experiment_file = f"{folder_path}/Experiment {selected_triangulation} {i}.csv"
        try:
            experiment_data = pd.read_csv(experiment_file, delimiter=",", dtype=str)
            experiment_data.columns = [col.strip() for col in experiment_data.columns]
        except FileNotFoundError:
            continue
        
        for file in files:
            df = pd.read_csv(file, delimiter=",", dtype=str)
            df.columns = [col.strip() for col in df.columns]
            df["parallax"] = df["parallax"].str.replace(",", ".").astype(float)
            df["Av. error (mm)"] = df["Av. error (mm)"].str.replace(",", ".").astype(float)
            
            for index, row in df.iterrows():
                match = re.match(r"(seq\d+)_(checks|no_checks)", row["Info"])
                if match:
                    seq, check_type = match.groups()
                    pattern = fr"(\d+-\d+-\d+)_{selected_triangulation}"
                    higher_info_match = re.match(pattern, row["Higher Info"])
                    if not higher_info_match:
                        continue
                    pair = higher_info_match.group(1)
                    
                    col_t = f"{pair}-{selected_triangulation} t C1C2 norm (mm)"
                    if col_t not in experiment_data.columns:
                        continue
                    
                    t_values = experiment_data.loc[
                        (experiment_data.iloc[:, 0] == seq) &
                        (experiment_data.iloc[:, 1] == check_type),
                        col_t
                    ]
                    if not t_values.empty:
                        try:
                            t_value = float(t_values.values[0].replace(",", "."))
                            adjusted_error = row["Av. error (mm)"] / t_value
                            key = f"{seq}_{check_type}"
                            data[data_type][key]["parallax"].append(row["parallax"])
                            data[data_type][key]["error"].append(adjusted_error)
                        except ValueError:
                            continue

# Procesar datos para MidPoint (Initial de TwoPoints)
for i in file_range:
    file_pattern = f"{folder_path}/Data {i} {midpoint_triangulation} Initial.csv"
    files = glob.glob(file_pattern)
    
    experiment_file = f"{folder_path}/Experiment {midpoint_triangulation} {i}.csv"
    try:
        experiment_data = pd.read_csv(experiment_file, delimiter=",", dtype=str)
        experiment_data.columns = [col.strip() for col in experiment_data.columns]
    except FileNotFoundError:
        continue
    
    for file in files:
        df = pd.read_csv(file, delimiter=",", dtype=str)
        df.columns = [col.strip() for col in df.columns]
        df["parallax"] = df["parallax"].str.replace(",", ".").astype(float)
        df["Av. error (mm)"] = df["Av. error (mm)"].str.replace(",", ".").astype(float)
        
        for index, row in df.iterrows():
            match = re.match(r"(seq\d+)_(checks|no_checks)", row["Info"])
            if match:
                seq, check_type = match.groups()
                pattern = fr"(\d+-\d+-\d+)_{midpoint_triangulation}"
                higher_info_match = re.match(pattern, row["Higher Info"])
                if not higher_info_match:
                    continue
                pair = higher_info_match.group(1)
                
                col_t = f"{pair}-{midpoint_triangulation} t C1C2 norm (mm)"
                if col_t not in experiment_data.columns:
                    continue
                
                t_values = experiment_data.loc[
                    (experiment_data.iloc[:, 0] == seq) &
                    (experiment_data.iloc[:, 1] == check_type),
                    col_t
                ]
                if not t_values.empty:
                    try:
                        t_value = float(t_values.values[0].replace(",", "."))
                        adjusted_error = row["Av. error (mm)"] / t_value
                        key = f"{seq}_{check_type}"
                        data["MidPoint"][key]["parallax"].append(row["parallax"])
                        data["MidPoint"][key]["error"].append(adjusted_error)
                    except ValueError:
                        continue

# Plot configuration
key_parallax_values = [1, 1.5, 2, 2.5, 3, 3.5]
min_e, max_e = 0.0, 30
min_p, max_p = 0.1, 5.7
num_intervals = 8
colors = ["cyan", "red", "green", "purple", "orange", "blue", "black"]
custom_titles = {
    "seq0_no_checks": "No deformation",
    "seq1_no_checks": "Deformation level 1",
    "seq2_no_checks": "Deformation level 2",
    "seq3_no_checks": "Deformation level 3",
    "seq4_no_checks": "Deformation level 4",
    "seq5_no_checks": "Deformation level 5",
}

for i, (seq, initial_values) in enumerate(data["Initial"].items()):
    if seq not in data["Final"] or seq not in data["MidPoint"]:
        continue
    
    final_values = data["Final"][seq]
    midpoint_values = data["MidPoint"][seq]
    plt.figure(figsize=(6, 5))
    color = colors[i % len(colors)]
    
    legend_labels = ["MidPoint", "Before optimization", "After optimization"]
    line_styles = [":", "--", "-"]
    values_list = [midpoint_values, initial_values, final_values]
    
    for label, values, linestyle in zip(legend_labels, values_list, line_styles):
        if values["parallax"]:
            parallax, error = zip(*sorted(zip(values["parallax"], values["error"])))
            interval_bounds = np.linspace(min_p, max_p, num_intervals)
            grouped_parallax, avg_errors = [], []
            
            for j in range(len(interval_bounds) - 1):
                lower, upper = interval_bounds[j], interval_bounds[j + 1]
                indices = [k for k, p in enumerate(parallax) if lower <= p < upper]
                if indices:
                    grouped_parallax.append(np.mean([parallax[k] for k in indices]))
                    avg_errors.append(np.mean([error[k] for k in indices]))
            
            plt.plot(grouped_parallax, avg_errors, label=label, color=color, linewidth=2, linestyle=linestyle)
    
    plt.title(custom_titles.get(seq, seq))
    plt.xlabel("Parallax (degrees)")
    plt.ylabel("Av. error / t")
    plt.xlim(min_p, max_p)
    plt.ylim(min_e, max_e)
    plt.grid()
    plt.legend()
    plt.legend(loc="upper right")
    plt.show()
