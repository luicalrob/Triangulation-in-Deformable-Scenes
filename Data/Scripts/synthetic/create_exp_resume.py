#!/usr/bin/env python3

import argparse
import sys
import os
import csv
import pandas as pd
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import synthetic_values

default_values = synthetic_values

"""
Script to aggregate resume all the experiments generated as CSV files.

This script processes data from a specific Model, using a chosen 
Triangulation method and Experiment. It compiles the data 
into a consolidated CSV file for easier analysis.

Is important to fist use automated_loop_to_csv.py

Output:
- A single CSV & Excel file containing all the experiments for the specified 
  Model, Triangulation method and Experiment.

Author: Luis Calderón Robustillo
Date: 19/11/24
"""

def validate_options(option, available_values, option_name):
    if option not in available_values:
        raise ValueError(
            f"{option_name} '{option}' is not valid. "
            f"Available options are: {available_values}"
        )

def get_csv_filename(model, triangulation, experiment):
    return f"{model}_{triangulation}_{experiment}.csv"

def process_files(models, triangulations, experiment):
    """
    Process the CSV files for the specified models, triangulations, and experiment.

    Args:
        models (list): List of model names to process.
        triangulations (list): List of triangulation methods to process.
        experiment (int): Experiment number to process.

    Returns:
        pd.DataFrame: Consolidated DataFrame of results.
    """
    # Initialize the results with a metadata structure.
    results = []
    metadata = {
        "Avg Movement": [],
        "Shape": [],
        "Gaussian Mov": [],
        "Rigid Mov": [],
    }

    # Headers for model and triangulation columns
    model_columns = {}
    for model in models:
        for triangulation in triangulations:
            col_prefix = f"{model}-{triangulation}"
            # model_columns[f"{col_prefix} Improvement (%)"] = []
            # model_columns[f"{col_prefix} Final Vs Mov (%)"] = []
            # model_columns[f"{col_prefix} Initial VS Mov (%)"] = []
            model_columns[f"{col_prefix} Initial (mm)"] = []
            model_columns[f"{col_prefix} Final (mm)"] = []

    # Now process each model and triangulation combination
    firstIteration = True
    for model in models:
        for triangulation in triangulations:
            file_name = f"{model}_{triangulation}_{experiment}.csv"
            if(model == "ARAP_not_scaled_depth"):
                folder = "Depth without scale"
            elif(model == "ARAP_depth_onlyTriang"):
                folder = "Depth methods"
            elif(model == "ARAP_depth_1mm" or model == "ARAP_depth_3mm" or model == "ARAP_depth_8mm"):
                folder = "Depth uncertainty"
            else:
                folder = "Compare models"

            DATA_DIR = f"./Data/Excels/Synthetic/{folder}/{experiment}"
            file_path = os.path.join(DATA_DIR, file_name)

            if os.path.exists(file_path):
                df = pd.read_csv(file_path, skiprows=1)
                initial_rows = df[df["Section"] == "INITIAL"]
                final_rows = df[df["Section"] == "FINAL"]

                # Ensure we process the correct rows
                if not final_rows.empty:
                    for _, initial_row in initial_rows.iterrows():  # Loop through all "INITIAL" rows
                        initial_error = initial_row["Av. error"]
                        mov = initial_row["Av. movement"]

                        initial_e_value = float(initial_error.replace(',', '.'))
                        mov_value = float(mov.replace(',', '.'))

                        initial_vs_mov_percentage = (initial_e_value / mov_value) * 100
                        # initial_error = "{:.2f}".format(initial_vs_mov_percentage).replace('.', ',')

                        # model_columns[f"{model}-{triangulation} Initial VS Mov (%)"].append(initial_vs_mov)  
                        model_columns[f"{model}-{triangulation} Initial (mm)"].append(initial_error)  

                    for _, final_row in final_rows.iterrows():  # Loop through all "FINAL" rows
                        if (firstIteration):
                            # Extract values for each "FINAL" row
                            avg_movement = final_row["Av. movement"]
                            shape = final_row["Shape"]
                            gaussian_mov = final_row["Gaussian"]
                            rigid_mov = final_row["Rigid"]

                            # Populate the metadata dictionary
                            metadata["Avg Movement"].append(avg_movement)
                            metadata["Shape"].append(shape)
                            metadata["Gaussian Mov"].append(gaussian_mov)
                            metadata["Rigid Mov"].append(rigid_mov)

                        improvement = final_row["Improv. (%)"]
                        final_vs_mov = final_row["Final Vs Mov (%)"]
                        final_error = final_row["Av. error"]

                        # Add Improvement (%) and Final Vs Mov (%) for the specific model & triangulation
                        # model_columns[f"{model}-{triangulation} Improvement (%)"].append(improvement)
                        # model_columns[f"{model}-{triangulation} Final Vs Mov (%)"].append(final_vs_mov)  
                        model_columns[f"{model}-{triangulation} Final (mm)"].append(final_error)  
                    firstIteration = False

                else:
                    print(f"No FINAL row found in {file_name}.")
            else:
                print(f"File not found: {file_name}")
            
    # Merge all collected data into a single DataFrame
    final_data = {**metadata, **model_columns}
    for key, value in final_data.items():
        print(f"Key: {key}, Length: {len(value)}")
    return pd.DataFrame(final_data)


parser = argparse.ArgumentParser(description="Process CSV data for experiments.")
parser.add_argument("--Models", nargs='+', required=True, help="List of models to include.")
parser.add_argument("--Triangulation", nargs='+', required=True, help="Triangulation methods to include.")
parser.add_argument("--Experiment", type=int, required=True, help="Experiment number (1-6).")
args = parser.parse_args()

# Validate input
selected_models = args.Models
selected_triangulations = args.Triangulation
selected_experiment = args.Experiment

for model in selected_models:
    validate_options(model, default_values["Model"], "Model")
for triangulation in selected_triangulations:
    validate_options(triangulation, default_values["Triangulation"], "Triangulation")
validate_options(selected_experiment, default_values["ExperimentType"], "Experiment")

# Collect relevant CSV files
csv_files = []
for model in selected_models:
    for triangulation in selected_triangulations:
        filename = get_csv_filename(model, triangulation, selected_experiment)

        if(model == "ARAP_not_scaled_depth"):
            folder = "Depth without scale"
        elif(model == "ARAP_depth_onlyTriang"):
            folder = "Depth methods"
        elif(model == "ARAP_depth_1mm" or model == "ARAP_depth_3mm" or model == "ARAP_depth_8mm"):
            folder = "Depth uncertainty"
        else:
            folder = "Compare models"
            
        DATA_DIR = f"./Data/Excels/Synthetic/{folder}/{selected_experiment}"
        filepath = os.path.join(DATA_DIR, filename)
        if os.path.exists(filepath):
            csv_files.append(filepath)
        else:
            print(f"Warning: File {filename} not found in {DATA_DIR}.")

if not csv_files:
    raise FileNotFoundError("No valid CSV files found for the given parameters.")

print(f"Collected CSV files: {csv_files}")

# Process the files and create a DataFrame
results_df = process_files(selected_models, selected_triangulations, selected_experiment)

output_file = f"./Data/Excels/Synthetic/Resumes/Errors {args.Experiment}"

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