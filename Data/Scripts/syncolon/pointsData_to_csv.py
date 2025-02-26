#!/usr/bin/env python3
import csv
import re
import sys
import os
import pandas as pd
import argparse
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import syncolon_values

default_values = syncolon_values

def parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Set parameters for the experiment.")
    parser.add_argument('--Pair', type=str, choices=default_values["Pair"], required=True, help="Pair of frames")
    parser.add_argument('--Triangulation', type=str, choices=default_values["Triangulation"], required=False, help="Triangulation type")
    parser.add_argument('--Level', type=str, choices=default_values["Level"], required=False, help="Deformation level")
    parser.add_argument('--Checks', type=str, choices=default_values["Checks"], required=False, help="Check essential matrix")
    parser.add_argument('--Experiment', type=int, choices=default_values["Experiment"], required=False, help="Experiment number")
    return parser.parse_args()

def convert_to_float(value):
    if '.' in value:
        value = value.replace('.', '')
    value = value.replace(',', '.')
    # Return the value converted to float
    return float(value)

def process_value(value, key, precision=2, scientific=False):
    """
    Process and format a value according to the given parameters.

    Args:
        value (str): The input value.
        key (str): The key to associate with the value.
        precision (int): Number of decimal places.
        scientific (bool): Use scientific notation if True.

    Returns:
        tuple: Key and formatted value.
    """
    value_float = float(convert_to_float(value))
    formatted_value = (f"{value_float:.{precision}e}" if scientific
                       else f"{round(value_float, precision):.{precision}f}")
    return key, formatted_value.replace('.', ',')

def convert_txt_to_csv(txt_path, csv_path, level, checks, precision=2, scientific=False):
    header_info = f"{level}_{checks}"

    # Read TXT file with proper decimal handling
    df = pd.read_csv(txt_path, sep=";", header=None, names=["Av. error (mm)", "parallax"], decimal=",", dtype=str)

    # Process numerical columns using process_value function
    df["Av. error (mm)"] = df["Av. error (mm)"].apply(lambda x: process_value(x, "Av. error (mm)", precision, scientific)[1])
    df["parallax"] = df["parallax"].apply(lambda x: process_value(x, "parallax", precision, scientific)[1])

    # Insert "Info" column in third position
    df.insert(2, "Info", header_info)
    df = df[["parallax", "Av. error (mm)", "Info"]]

    # Ensure correct CSV formatting: use `;` as separator and keep `,` for decimals
    df.to_csv(csv_path, mode='a' if os.path.exists(csv_path) else 'w', header=not os.path.exists(csv_path),
          index=False, sep=",", encoding="utf-8")

def save_to_excel(csv_path, excel_path):
    """Convert CSV data to Excel format."""
    df = pd.read_csv(csv_path, decimal=',')
    df.to_excel(excel_path, index=False, engine='openpyxl')

def main():
    args = parse_arguments()
    
    Pair = args.Pair
    Triangulation = args.Triangulation or "FarPoints"
    Level = args.Level or "seq0"
    Checks = args.Checks or "no_checks"
    Experiment = args.Experiment or 1

    initial_frame = Pair.split("-")[0]
    step = Pair.split("-")[1]
    final_frame = Pair.split("-")[2]

    base_path = f'./Data/Experiments/Syncolon/{initial_frame}_{final_frame}/{args.Triangulation}/{args.Level}/{args.Checks}/{args.Experiment}'
    excel_path = f'./Data/Excels/Syncolon/{initial_frame}_{final_frame}'

    files = {
        "Initial": (f"{base_path}/Initial.txt", f"{excel_path}/{args.Triangulation}_{args.Experiment}_Initial.csv"),
        "Final": (f"{base_path}/Final.txt", f"{excel_path}/{args.Triangulation}_{args.Experiment}_Final.csv")
    }

    for key, (txt_path, csv_path) in files.items():
        convert_txt_to_csv(txt_path, csv_path, Level, Checks)
        print(f"Coverted {txt_path} to {csv_path}")
        save_to_excel(csv_path, csv_path.replace('.csv', '.xlsx'))

    print("CSV and Excel files created successfully.")

if __name__ == "__main__":
    main()
