import sys
import os
import pandas as pd
import matplotlib.pyplot as plt
import glob
import re
import numpy as np
from collections import defaultdict

# Configuration
folder_path = "./Data/Excels/Syncolon/Resumes"
triangulation_type = "FarPoints"
file_range = range(66, 71)

data = {"Initial": defaultdict(lambda: {"parallax": [], "error": []}),
        "Final": defaultdict(lambda: {"parallax": [], "error": []})}

# Read and process files for both Initial and Final
for data_type in ["Initial", "Final"]:
    for i in file_range:
        file_pattern = f"{folder_path}/Data {i} {triangulation_type} {data_type}.csv"
        files = glob.glob(file_pattern)

        experiment_file = f"{folder_path}/Experiment {triangulation_type} {i}.csv"
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
                    pattern = fr"(\d+-\d+-\d+)_{triangulation_type}"
                    higher_info_match = re.match(pattern, row["Higher Info"])
                    if not higher_info_match:
                        continue
                    pair = higher_info_match.group(1)

                    col_t = f"{pair}-{triangulation_type} t C1C2 norm (mm)"
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

# Plot configuration
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
    if seq not in data["Final"]:
        continue  # Ensure both Initial and Final exist
    
    final_values = data["Final"][seq]
    plt.figure(figsize=(6, 5))
    color = colors[i % len(colors)]
    
    if final_values["parallax"]:
        plt.scatter(final_values["parallax"], final_values["error"], label="Our non-rigid triangulation", color=color, marker="s", edgecolors="black", s=20, zorder=3)
    if initial_values["parallax"]:
        plt.scatter(initial_values["parallax"], initial_values["error"], label="MidPoint", color="black", marker="x", s=20, zorder=3)
    

    font_size = 16
    plt.title(custom_titles.get(seq, seq), fontsize=font_size)
    plt.xlabel("Parallax (degrees)", fontsize=font_size)
    plt.ylabel("Av. error / t", fontsize=font_size)
    plt.grid()
    plt.legend(loc="upper right", fontsize=font_size)
    plt.show()