#!/usr/bin/env python3
import argparse
import subprocess
from itertools import product

def setExperiment(experiment_type):
    """
    Args:
        experiment_type (int): The experiment type (1 a 6).
    
    Returns:
        dict: gaussianMov and rigidMov values
    """
    # Diccionario de configuración
    experiment_config = {
        1: {"gaussian": 2.5, "rigid": 0},
        2: {"gaussian": 0, "rigid": 2.5},
        3: {"gaussian": 2.5, "rigid": 2.5},
        4: {"gaussian": 10, "rigid": 0},
        5: {"gaussian": 0, "rigid": 10},
        6: {"gaussian": 10, "rigid": 10},
    }
    
    try:
        return experiment_config[experiment_type]
    except KeyError:
        raise ValueError("The type of experiment must be between 1 and 6.")


default_values = {
    "Triangulation": ["InRays", "TwoPoints"],
    "Depth": [20, 80, 150],
    "Shape": ["Planar", "Gradual"],
    "ExperimentType": range(1, 7),
    "Experiment": range(1, 6),
}

shape_experiment_types = {
    "Planar": [1, 2, 3, 4, 5, 6],
    "Gradual": [2, 3, 5, 6],
}

parser = argparse.ArgumentParser(description="Run experiments automatically.")
parser.add_argument('--Model', type=str, choices=["ARAP_NoGlobal", "ARAP_Modified", "ARAP", "Elastic", "HyperElasticOdgen", "ARAP_OneSet"] ,required=True, help="Model name (ARAP_NoGlobal, ARAP_OneSet, ARAP, Elastic or HyperElastic)")
parser.add_argument('--Triangulation', type=str, choices=default_values["Triangulation"], required=True, help="Triangulation type (InRays or TwoPoints)")
parser.add_argument('--Depth', type=int, choices=default_values["Depth"], required=False, help="Depth value (20, 80, 150)")
parser.add_argument('--Shape', type=str, choices=default_values["Shape"], required=False, help="Shape type (Planar or Gradual)")
parser.add_argument('--ExperimentType', type=int, choices=default_values["ExperimentType"], help="Type of experiment (1 to 6)", required=False)
parser.add_argument('--Experiment', type=int, choices=default_values["Experiment"], required=True, help="Experiment number (1 to 5)")
args = parser.parse_args()
        
depths = [args.Depth] if args.Depth else default_values["Depth"]
shapes = [args.Shape] if args.Shape else default_values["Shape"]
experiment_types = [args.ExperimentType] if args.ExperimentType else default_values["ExperimentType"]

for depth, shape in product(depths, shapes):
    # Determine experiment types based on shape
    if shape in shape_experiment_types:
        experiment_types = [args.ExperimentType] if args.ExperimentType else shape_experiment_types[shape]
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

        convert_script = './Data/Scripts/convert_to_csv.py'

        command = [
            "python3",
            convert_script,
            "--Model", args.Model,
            "--Triangulation", args.Triangulation,
            "--Depth", str(depth),
            "--Shape", shape,
            "--ExperimentType", str(experiment_type),
            "--Experiment", str(args.Experiment)
        ]

        try:
            result = subprocess.run(command, check=True, text=True, capture_output=True)
            print(result.stdout)  # Print the script's output
        except subprocess.CalledProcessError as e:
            print(f"Error running {convert_script}: {e.stderr}")