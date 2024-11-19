import os
import shutil
import argparse

# Define los parámetros de entrada
parser = argparse.ArgumentParser(description="Run experiments automatically.")
parser.add_argument('--Model', type=str, choices=["ARAP_NoGlobal", "ARAP", "Elastic", "HyperElastic"] ,required=True, help="Model name (ARAP_NoGlobal, ARAP, Elastic or HyperElastic)")
parser.add_argument('--Triangulation', type=str, choices=["InRays", "TwoPoints"], required=False, help="Triangulation type (InRays or TwoPoints)")
parser.add_argument('--Depth', type=int, choices=[20, 80, 150], required=False, help="Depth value (20, 80, 150)")
parser.add_argument('--Shape', type=str, choices=["Planar", "Gradual"], required=False, help="Shape type (Planar or Gradual)")
parser.add_argument('--ExperimentType', type=int, choices=range(1, 7), help="Type of experiment (1 to 6)", required=False)
parser.add_argument('--Experiment', type=int, choices=range(1, 6), required=False, help="Experiment number (1 to 5)")
args = parser.parse_args()

source_folder = f'./Data/SinteticData/{args.Depth}cm Depth/{args.Shape}/{args.totalMov} mm {args.typeMov}/{args.Experiment}'
destination_folder = './Data'
test_yaml_path = './Data/Test.yaml'
result_folder = f'./Data/Experiments/{args.Model}/{args.Triangulation}/{args.Depth}cm Depth/{args.Shape}/{args.totalMov} mm {args.typeMov}/{args.Experiment}'

os.makedirs(destination_folder, exist_ok=True)
shutil.copy(f'{source_folder}/moved_points.csv', destination_folder)
shutil.copy(f'{source_folder}/original_points.csv', destination_folder)

camera_poses = {
    150: {"x": 0.34, "y": 0.08, "z": 0.06},
    80: {"x": 0.24, "y": 0.01, "z": 0.06},
    20: {"x": 0.14, "y": 0.01, "z": 0.06}
}

if args.Depth in camera_poses:
    pose = camera_poses[args.Depth]
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
            else:
                file.write(line)

# Ejecutar el programa principal
main_executable = './Execution/main'
if os.path.isfile(main_executable):
    os.system(main_executable)
else:
    raise FileNotFoundError(f"The executable {main_executable} does not exist.")

# Mover el archivo Experiment.txt al directorio de resultados
os.makedirs(result_folder, exist_ok=True)
experiment_file = f'{destination_folder}/Experiment.txt'
if os.path.isfile(experiment_file):
    shutil.move(experiment_file, f'{result_folder}/Experiment.txt')
else:
    raise FileNotFoundError(f"The experiment result file {experiment_file} does not exist.")
