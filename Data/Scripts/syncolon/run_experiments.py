#!/usr/bin/env python3

"""
Script to run the experiments that we have selected.

This script create optimization data from a specific Model, using
data selected from the sintetic data base previously prepared.

Steps:
1. Create data if it doesn't exist using "create_data.py" script.
2. Select the Model. 
3. Select extra choices if needed (not mandatory).
4. Run the script.
Example:
./Data/Scripts/real/run_real_experiments.py --Pair 1024_00000_1229-1236


Output:
- Experiments.txt files saved in each folder inside "./Data/Experiments"

Author: Luis Calderón Robustillo
Date: 19/11/24
"""
import sys
import os
import shutil
import argparse
from itertools import product

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config import syncolon_values, syncolon_level_types, setSyncolonParameters

default_values = syncolon_values

parser = argparse.ArgumentParser(description="Run experiments automatically.")
parser.add_argument('--Pair', type=str, choices=default_values["Pair"] ,required=False, help="Pair of frames")
parser.add_argument('--Triangulation', type=str, choices=default_values["Triangulation"], required=False, help="Triangulation type (FarPoints, InRays or TwoPoints)")
parser.add_argument('--Level',nargs='+', type=str, choices=default_values["Level"], help="Level of deformation (level0 to level3)", required=False)
parser.add_argument('--Checks', type=str, choices=default_values["Checks"], required=False, help="Check Essential matrix and rep. error or not")
parser.add_argument('--Experiment', type=int, choices=default_values["Experiment"], required=False, help="Experiment number (1 to 5)")
args = parser.parse_args()

pairs = [args.Pair] if args.Pair else default_values["Pair"]
triangulations = [args.Triangulation] if args.Triangulation else default_values["Triangulation"]
levels = args.Level if args.Level else default_values["Level"]
checks = [args.Checks] if args.Checks else default_values["Checks"]
experiments = [args.Experiment] if args.Experiment else default_values["Experiment"]

for pair, triangulation, level, experiment in product(pairs, triangulations, levels, experiments):
    # Determine checks types based on level

    if (level in syncolon_level_types) and not args.Checks:
        checks = syncolon_level_types[level] 

    initial_frame = pair.split("-")[0]
    step = pair.split("-")[1]
    final_frame = pair.split("-")[2]

    int_initial_frame = int(initial_frame)
    final_initial_frame = int(final_frame)
    difference = final_initial_frame - int_initial_frame
    limit = final_initial_frame + 1 

    limit_frame = str(limit)
    for check_election in checks:
        test_yaml_path = './Data/Syncolon.yaml'
        destination_folder = './Data'
        result_folder = f'./Data/Experiments/Syncolon/{initial_frame}_{final_frame}/{triangulation}/{level}/{check_election}/{experiment}'

        parameters = setSyncolonParameters(pair, level)

        cintrinsics = {"fx": 472.64955100886374, "fy": 472.64955100886374, "cx": 479.5, "cy": 359.5}
        cresolution = {"cols": 960, "rows": 720}
        
        with open(test_yaml_path, 'r') as file:
            lines = file.readlines()
        with open(test_yaml_path, 'w') as file:
            for line in lines:
                if "Camera.fx:" in line:
                    file.write(f"Camera.fx: {cintrinsics['fx']}\n")
                elif "Camera.fy:" in line:
                    file.write(f"Camera.fy: {cintrinsics['fy']}\n")
                elif "Camera.cx:" in line:
                    file.write(f"Camera.cx: {cintrinsics['cx']}\n")
                elif "Camera.cy:" in line:
                    file.write(f"Camera.cy: {cintrinsics['cy']}\n")

                elif "Camera.cols:" in line:
                    file.write(f"Camera.cols: {cresolution['cols']}\n")
                elif "Camera.rows:" in line:
                    file.write(f"Camera.rows: {cresolution['rows']}\n")

                elif "Matching.initialization:" in line:
                    file.write(f"Matching.initialization: {parameters['Hdist']}\n")
                elif "Triangulation.minCos:" in line:
                    file.write(f"Triangulation.minCos: {parameters['minParallax']}\n")
                elif "Triangulation.depthLimit:" in line:
                    file.write(f"Triangulation.depthLimit: {parameters['maxDepth']}\n")
                elif "Matching.initialization.radius:" in line:
                    file.write(f"Matching.initialization.radius: {parameters['window']}\n")
                elif "Triangulation.checks:" in line:
                    if check_election == "no_checks":
                        file.write(f'Triangulation.checks: "false"\n')
                    elif check_election == "checks":
                        file.write(f'Triangulation.checks: "true"\n')


                elif "Execution.stop:" in line:
                    file.write(f'Execution.stop: "false"\n')
                elif "MapVisualizer.showScene:" in line:
                    file.write(f'MapVisualizer.showScene: "true"\n')
                elif "Triangulation.seed.location:" in line:
                    file.write(f'Triangulation.seed.location: "{triangulation}"\n')
                else:
                    file.write(line)

        source_folder = f'/home/luis/datasets/SynColon/juanjo/{level} '
        main_executable = './Execution/syncolon'
        if os.path.isfile(main_executable):
            os.system(main_executable + ' ' + source_folder + initial_frame + ' ' + step + ' ' + limit_frame)
        else:
            raise FileNotFoundError(f"The executable {main_executable} does not exist.")

        os.makedirs(result_folder, exist_ok=True)
        experiment_file = f'{destination_folder}/Experiment.txt'
        if os.path.isfile(experiment_file):
            shutil.move(experiment_file, f'{result_folder}/Experiment.txt')
        else:
            raise FileNotFoundError(f"The experiment result file {experiment_file} does not exist.")

        initial_data_file = f'{destination_folder}/Initial.txt'
        if os.path.isfile(initial_data_file):
            shutil.move(initial_data_file, f'{result_folder}/Initial.txt')

        final_data_file = f'{destination_folder}/Final.txt'
        if os.path.isfile(final_data_file):
            shutil.move(final_data_file, f'{result_folder}/Final.txt')