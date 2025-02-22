#!/usr/bin/env python3

import argparse
import sys
import os
import csv
import pandas as pd
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import syncolon_values

default_values = syncolon_values

"""
Script to aggregate resume all the experiments generated as CSV files.

This script processes data from a specific Model, using a chosen 
Triangulation method and Experiment. It compiles the data 
into a consolidated CSV file for easier analysis.

Is important to fist use automated_loop_to_csv.py

Output:
- A single CSV & Excel file containing all the experiments for the specified 
  Model, Triangulation method and Experiment.
Example:
./Data/Scripts/real/create_exp_resume.py --Pair 1024_00000_1229-1236 --Triangulation FarPoints TwoPoints --Experiment 1

Author: Luis Calderón Robustillo
Date: 19/11/24
"""

def validate_options(option, available_values, option_name):
    if option not in available_values:
        raise ValueError(
            f"{option_name} '{option}' is not valid. "
            f"Available options are: {available_values}"
        )

def get_csv_filename(triangulation, experiment):
    return f"{triangulation}_{experiment}.csv"

def process_files(pairs, triangulations, experiment):
    """
    Process the CSV files for the specified pairs, triangulations, and experiment.

    Args:
        pairs (list): List of pairs names to process.
        triangulations (list): List of triangulation methods to process.
        experiment (int): Experiment number to process.

    Returns:
        pd.DataFrame: Consolidated DataFrame of results.
    """
    # Initialize the results with a metadata structure.
    results = []
    metadata = {
        "Level": [],
        "Checks": [],
    }

    # Headers for model and triangulation columns
    pair_columns = {}
    for pair in pairs:
        for triangulation in triangulations:
            col_prefix = f"{pair}-{triangulation}"
            pair_columns[f"{col_prefix} Improvement (%)"] = []
            pair_columns[f"{col_prefix} Final Vs Mov (%)"] = []
            pair_columns[f"{col_prefix} Av. movement"] = []
            pair_columns[f"{col_prefix} Av. error"] = []
            pair_columns[f"{col_prefix} Av. upToScale error"] = []
            pair_columns[f"{col_prefix} RMSE"] = []
            pair_columns[f"{col_prefix} t C1C2 norm (mm)"] = []
            pair_columns[f"{col_prefix} parallax"] = []
            pair_columns[f"{col_prefix} nMatches"] = []
            pair_columns[f"{col_prefix} nMPs"] = []

    # Now process each model and triangulation combination
    firstIteration = True
    for pair in pairs:
        initial_frame = pair.split("-")[0]
        step = pair.split("-")[1]
        final_frame = pair.split("-")[2]
        DATA_DIR = f'./Data/Excels/Syncolon/{initial_frame}_{final_frame}'
        for triangulation in triangulations:
            file_name = f"{triangulation}_{experiment}.csv"
            file_path = os.path.join(DATA_DIR, file_name)

            if os.path.exists(file_path):
                df = pd.read_csv(file_path, skiprows=1)
                initial_rows = df[df["Section"] == "INITIAL"]
                final_rows = df[df["Section"] == "FINAL"]

                # Ensure we process the correct rows
                if not final_rows.empty:
                    for _, final_row in final_rows.iterrows():  # Loop through all "FINAL" rows
                        if (firstIteration):
                            # Extract values for each "FINAL" row
                            level = final_row["Level"]
                            checks = final_row["Checks"]

                            # Populate the metadata dictionary
                            metadata["Level"].append(level)
                            metadata["Checks"].append(checks)

                        avg_movement = final_row["Av. movement"]
                        avg_error = final_row["Av. error"]
                        avg_error = final_row["Av. up-to-scale error in 3D"]
                        RMSE = final_row["RMSE"]
                        improvement = final_row["Improv. (%)"]
                        final_vs_mov = final_row["Final Vs Mov (%)"]
                        parallax = final_row["pllx"]
                        nMatches = final_row["nMchs"]
                        nMPs = final_row["nMPs"]
                        t = final_row["t C1C2 norm (mm)"]

                        # Add Improvement (%) and Final Vs Mov (%) for the specific model & triangulation
                        pair_columns[f"{pair}-{triangulation} Improvement (%)"].append(improvement)
                        pair_columns[f"{pair}-{triangulation} Final Vs Mov (%)"].append(final_vs_mov)  
                        pair_columns[f"{pair}-{triangulation} Av. movement"].append(avg_movement)  
                        pair_columns[f"{pair}-{triangulation} Av. error"].append(avg_error)  
                        pair_columns[f"{pair}-{triangulation} Av. upToScale error"].append(avg_error)  
                        pair_columns[f"{pair}-{triangulation} RMSE"].append(RMSE)
                        pair_columns[f"{pair}-{triangulation} t C1C2 norm (mm)"].append(t)
                        pair_columns[f"{pair}-{triangulation} parallax"].append(parallax)
                        pair_columns[f"{pair}-{triangulation} nMatches"].append(nMatches)
                        pair_columns[f"{pair}-{triangulation} nMPs"].append(nMPs)
                    firstIteration = False

                else:
                    print(f"No FINAL row found in {file_name}.")
            else:
                print(f"File not found: {file_name}")
            

    # Merge all collected data into a single DataFrame
    final_data = {**metadata, **pair_columns}
    for key, value in final_data.items():
        print(f"Key: {key}, Length: {len(value)}")
    return pd.DataFrame(final_data)


parser = argparse.ArgumentParser(description="Process CSV data for experiments.")
parser.add_argument("--Pairs", nargs='+', required=True, help="List of pairs to include.")
parser.add_argument("--Triangulation", nargs='+', required=True, help="Triangulation methods to include.")
parser.add_argument("--Experiment", type=int, required=True, help="Experiment number (1-6).")
args = parser.parse_args()


output_file = f"./Data/Excels/Syncolon/Resumes/Experiment {args.Experiment}"

# Validate input
selected_pairs = args.Pairs
selected_triangulations = args.Triangulation
selected_experiment = args.Experiment

for pair in selected_pairs:
    validate_options(pair, default_values["Pair"], "Pair")
for triangulation in selected_triangulations:
    validate_options(triangulation, default_values["Triangulation"], "Triangulation")
validate_options(selected_experiment, default_values["Experiment"], "Experiment")

# Collect relevant CSV files
csv_files = []
for pair in selected_pairs:
    initial_frame = pair.split("-")[0]
    step = pair.split("-")[1]
    final_frame = pair.split("-")[2]

    DATA_DIR = f"./Data/Excels/Syncolon/{initial_frame}_{final_frame}"
    for triangulation in selected_triangulations:
        filename = get_csv_filename(triangulation, selected_experiment)
        filepath = os.path.join(DATA_DIR, filename)
        if os.path.exists(filepath):
            csv_files.append(filepath)
        else:
            print(f"Warning: File {filename} not found in {DATA_DIR}.")

if not csv_files:
    raise FileNotFoundError("No valid CSV files found for the given parameters.")

print(f"Collected CSV files: {csv_files}")

# Process the files and create a DataFrame
results_df = process_files(selected_pairs, selected_triangulations, selected_experiment)

# Save the results to a CSV file
if not results_df.empty:
    csv_file = output_file + '.csv'
    os.makedirs(os.path.dirname(csv_file), exist_ok=True)
    results_df.to_csv(csv_file, index=False)
    print(f"Results saved to {csv_file}")

    df = pd.read_csv(csv_file, decimal=',')
    excel_file = output_file + '.xlsx'
    df.to_excel(excel_file, index=False, engine='openpyxl') 

else:
    print("No results to save.")
