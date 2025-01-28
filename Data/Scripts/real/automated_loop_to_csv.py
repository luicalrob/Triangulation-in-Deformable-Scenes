#!/usr/bin/env python3

"""
Script to aggregate experiments data into a single CSV file.

This script processes data from a specific Model, using a chosen 
Triangulation method and Experiment. It compiles the data 
into a consolidated CSV file for easier analysis.

Steps:
1. Run the experiments that you want to study using "run_experiments.py" script.
2. Select the Model, Triangulation method and the Experiment in the command line. 
3. Select extra choices if needed (not mandatory).
4. Run the script.
Example:
./Data/Scripts/real/automated_loop_to_csv.py --Pair 320_00000_1975-1983 --Triangulation InRays --Experiment 1


Output:
- A single CSV & Excel file containing all the experiments for the specified 
  Model and Triangulation method.

Author: Luis Calderón Robustillo
Date: 19/11/24
"""
import sys
import os
import argparse
import subprocess
from itertools import product
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import real_values, level_types, setParameters

default_values = real_values

parser = argparse.ArgumentParser(description="Run experiments automatically.")
parser.add_argument('--Pair',nargs='+', type=str, choices=default_values["Pair"] ,required=False, help="Pair of frames")
parser.add_argument('--Triangulation', type=str, choices=default_values["Triangulation"], required=False, help="Triangulation type (FarPoints, InRays or TwoPoints)")
parser.add_argument('--Level',nargs='+', type=str, choices=default_values["Level"], help="Level of deformation (level0 to level3)", required=False)
parser.add_argument('--Checks', type=str, choices=default_values["Checks"], required=False, help="Check Essential matrix and rep. error or not")
parser.add_argument('--Experiment', type=int, choices=default_values["Experiment"], required=False, help="Experiment number (1 to 5)")
args = parser.parse_args()
        
levels = args.Level if args.Level else default_values["Level"]
checks = [args.Checks] if args.Checks else default_values["Checks"]

pairs = args.Pair if args.Pair else default_values["Pair"]
triangulations = [args.Triangulation] if args.Triangulation else default_values["Triangulation"]
experiments = [args.Experiment] if args.Experiment else default_values["Experiment"]

convert_script = './Data/Scripts/real/convert_to_csv.py'

for pair, triangulation, experiment in product(pairs, triangulations, experiments):
    print("Pair: " + pair)
    print("Triangulation method: " + triangulation)
    print("Experiment: " + str(experiment))
    for level in product(levels):
        if (level[0] in level_types) and not args.Checks:
            checks = level_types[level[0]] 
        
        for check_election in checks:
            command = [
                "python3",
                convert_script,
                "--Pair", pair,
                "--Triangulation", triangulation,
                "--Level", level[0],
                "--Checks", check_election,
                "--Experiment", str(experiment)
            ]

            try:
                result = subprocess.run(command, check=True, text=True, capture_output=True)
                print(result.stdout)  # Print the script's output
            except subprocess.CalledProcessError as e:
                print(f"Error running {convert_script}: {e.stderr}")