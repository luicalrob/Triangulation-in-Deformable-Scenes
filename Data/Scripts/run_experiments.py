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
./Data/Scripts/run_experiments.py --Model ARAP


Output:
- Experiments.txt files saved in each folder inside "./Data/Experiments"

Author: Luis Calderón Robustillo
Date: 19/11/24
"""

import os
import shutil
import argparse
from itertools import product

from config import default_values, shape_experiment_types, setExperiment

parser = argparse.ArgumentParser(description="Run experiments automatically.")
parser.add_argument('--Model', type=str, choices=default_values["Model"] ,required=True, help="Model name (ARAP_NoGlobal, ARAP_OneSet, ARAP, Elastic or HyperElastic)")
parser.add_argument('--Triangulation', type=str, choices=default_values["Triangulation"], required=False, help="Triangulation type (InRays or TwoPoints)")
parser.add_argument('--Depth', type=int, choices=default_values["Depth"], required=False, help="Depth value (20, 80, 150)")
parser.add_argument('--Shape', type=str, choices=default_values["Shape"], required=False, help="Shape type (Planar or Gradual)")
parser.add_argument('--ExperimentType',nargs='+', type=int, choices=default_values["ExperimentType"], help="Type of experiment (1 to 6)", required=False)
parser.add_argument('--Experiment', type=int, choices=default_values["Experiment"], required=False, help="Experiment number (1 to 5)")
args = parser.parse_args()

triangulations = [args.Triangulation] if args.Triangulation else default_values["Triangulation"]
depths = [args.Depth] if args.Depth else default_values["Depth"]
shapes = [args.Shape] if args.Shape else default_values["Shape"]
experiment_types = [args.ExperimentType] if args.ExperimentType else default_values["ExperimentType"]
experiments = [args.Experiment] if args.Experiment else default_values["Experiment"]

for triangulation, depth, shape, experiment in product(triangulations, depths, shapes, experiments):
    # Determine experiment types based on shape
    if shape in shape_experiment_types:
        if args.ExperimentType:
            experiment_types = args.ExperimentType if isinstance(args.ExperimentType, list) else [args.ExperimentType]
        else:
            experiment_types = shape_experiment_types[shape]
    else:
        experiment_types = default_values["ExperimentType"]

    for experiment_type in experiment_types:
        parameters = setExperiment(experiment_type)
        gaussianMov = parameters["gaussian"]
        rigidMov = parameters["rigid"]

        if gaussianMov == 2.5 or rigidMov == 2.5:
            totalMov = "2_5"
        else:
            totalMov = "10"

        if gaussianMov == 0:
            typeMov = "rigid"
        elif rigidMov == 0:
            typeMov = "gaussian"
        else:
            typeMov = "gaussian + rigid"

        source_folder = f'./Data/SinteticDataBase/{depth}cm Depth/{shape}/{totalMov} mm {typeMov}/{experiment}'
        destination_folder = './Data'
        test_yaml_path = './Data/Test.yaml'
        result_folder = f'./Data/Experiments/{args.Model}/{triangulation}/{depth}cm Depth/{shape}/{totalMov} mm {typeMov}/{experiment}'

        os.makedirs(destination_folder, exist_ok=True)
        shutil.copy(f'{source_folder}/moved_points.csv', destination_folder)
        shutil.copy(f'{source_folder}/original_points.csv', destination_folder)

        camera_poses = {
            150: {"x": 0.34, "y": 0.08, "z": 0.06},
            80: {"x": 0.24, "y": 0.01, "z": 0.06},
            20: {"x": 0.14, "y": 0.01, "z": 0.06},
        }

        if depth in camera_poses:
            pose = camera_poses[depth]
            with open(test_yaml_path, 'r') as file:
                lines = file.readlines()
            with open(test_yaml_path, 'w') as file:
                for line in lines:
                    if "Camera.SecondPose.x:" in line:
                        file.write(f"Camera.SecondPose.x: {pose['x']}\n")
                    elif "Camera.SecondPose.y:" in line:
                        file.write(f"Camera.SecondPose.y: {pose['y']}\n")
                    elif "Camera.SecondPose.z:" in line:
                        file.write(f"Camera.SecondPose.z: {pose['z']}\n")
                    elif "Execution.stop:" in line:
                        file.write(f'Execution.stop: "false"\n')
                    elif "Triangulation.seed.location:" in line:
                        file.write(f'Triangulation.seed.location: "{triangulation}"\n')
                    else:
                        file.write(line)

        main_executable = './Execution/main'
        if os.path.isfile(main_executable):
            os.system(main_executable)
        else:
            raise FileNotFoundError(f"The executable {main_executable} does not exist.")

        os.makedirs(result_folder, exist_ok=True)
        experiment_file = f'{destination_folder}/Experiment.txt'
        if os.path.isfile(experiment_file):
            shutil.move(experiment_file, f'{result_folder}/Experiment.txt')
        else:
            raise FileNotFoundError(f"The experiment result file {experiment_file} does not exist.")