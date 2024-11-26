#!/usr/bin/env python3
import csv
import re
import os
import pandas as pd
import argparse

def process_value(value, key, precision=2, scientific=False):
    """
    Processes a value extracted from a line and formats it according to the parameters.
    
    Args:
        line (str): The input line in the format 'key: value'.
        key (str): The key of the column to store in the dictionary.
        decimal_separator (str): The decimal separator to use in the output (default ',').
        precision (int): The number of decimal places to round.
        scientific (bool): If True, formats the number in scientific notation.
    
    returns:
        tuple: A key and a processed value ready to store in the dictionary.
    """
    
    value_float = float(value.replace(',', '.'))
    if scientific:
        formatted_value = f"{value_float:.{precision}e}"
    else:
        formatted_value = f"{round(value_float, precision):.{precision}f}"
    formatted_value = formatted_value.replace('.', ',')
    return key, formatted_value

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


parser = argparse.ArgumentParser(description="Set parameters for the experiment")
parser.add_argument('--Model', type=str, choices=["ARAP_depth", "ARAP_NoGlobal", "ARAP", "Elastic", "HyperElasticOdgen", "ARAP_OneSet"] ,required=False, help="Model name (ARAP_NoGlobal, ARAP, ARAP_OneSet, Elastic or HyperElastic)")
parser.add_argument('--Triangulation', type=str, choices=["InRays", "TwoPoints", "FarPoints"], required=False, help="Triangulation type (InRays or TwoPoints)")
parser.add_argument('--Depth', type=int, choices=[20, 80, 150], required=False, help="Depth value (20, 80, 150)")
parser.add_argument('--Shape', type=str, choices=["Planar", "Gradual"], required=False, help="Shape type (Planar or Gradual)")
parser.add_argument('--ExperimentType', type=int, choices=range(1, 7), help="Type of experiment (1 to 6)", required=False)
parser.add_argument('--Experiment', type=int, choices=range(1, 6), required=False, help="Experiment number (1 to 5)")
args = parser.parse_args()

#######  Inputs  ########

# "ARAP_NoGlobal", "ARAP_OneSet", "ARAP", "Elastic, HyperElasticOdgen"
if (args.Model):
    Model = args.Model
else:
    Model = "ARAP"

# "InRays" or "TwoPoints" 
if (args.Triangulation):
    Triangulation = args.Triangulation
else:           
    Triangulation = "TwoPoints"  

# 20, 80, 150   
if (args.Depth):
    Depth = args.Depth
else:
    Depth = 150       

# "Planar" or "Gradual"    
if (args.Shape):
    Shape = args.Shape
else:           
    Shape = "Gradual" 

# 0, 2.5, 10             
gaussianMov = 2.5
# 0, 2.5, 10            
rigidMov = 0

# or type of experiment # 1, 2, 3, 4, 5 or 6
if (args.ExperimentType):
    ExperimentType = args.ExperimentType
else:
    ExperimentType = 1
# 1, 2, 3, 4, 5  (Same experiment but different data)         
Experiment = 1    

parameters = setExperiment(ExperimentType)
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


# Step 1: Read the text file
with open('./Data/Experiments/' + Model + '/' + Triangulation + '/' + str(Depth) + 'cm Depth/' + Shape + 
            '/' + totalMov + ' mm ' + typeMov + '/' + str(Experiment) +'/Experiment.txt', 'r') as txt_file:
    lines = txt_file.readlines()

# Step 2: Initialize variables
out_file_path = './Data/Excels/'+ Model + "_" + Triangulation + "_" + str(Experiment)

firstMeasure = False
if not os.path.exists(out_file_path + '.csv') or os.stat(out_file_path + '.csv').st_size == 0:
    firstMeasure = True
else:
    firstMeasure = False

measurements = []
current_measurement = {}
line_index = 0

section = ""
initial_av_error = None
final_av_error = None
av_movement = None


# Step 3: Extract data line by line
while line_index < len(lines):
    line = lines[line_index].strip()
    
    if "INITIAL MEASUREMENTS" in line or re.match(r"\d+ \/ \d+ MEASUREMENTS", line) or "FINAL MEASUREMENTS" in line:
        if current_measurement:
            measurements.append(current_measurement)
        
        section = re.sub(r'\s*MEASUREMENTS\s*', '', line)
        section = re.sub(r'\s*:\s*', '', section)
        
        current_measurement = {"Section": section}
    
    elif "C1 standard desv" in line:
        value = line.split(':')[1].strip()
        key, finalValue = process_value(value, "C1 std dev", precision=2)
        current_measurement[key] = finalValue
    elif "C2 standard desv" in line:
        value = line.split(':')[1].strip()
        key, finalValue = process_value(value, "C2 std dev", precision=2)
        current_measurement[key] = finalValue

    elif "Rel. error" in line:
        value = line.split(':')[1].strip()
        key, finalValue = process_value(value, "Rel. error", precision=2, scientific=True)
        current_measurement[key] = finalValue

    elif "Global rotation" in line:
        rotation_matrix = [
            lines[line_index].strip().split()[2:],
            lines[line_index + 1].strip().split(),
            lines[line_index + 2].strip().split()
        ]

        key, finalValue = process_value(rotation_matrix[0][0], "Global rotation X", precision=2)
        current_measurement[key] = finalValue
        key, finalValue = process_value(rotation_matrix[1][1], "Global rotation Y", precision=2)
        current_measurement[key] = finalValue
        key, finalValue = process_value(rotation_matrix[2][2], "Global rotation Z", precision=2)
        current_measurement[key] = finalValue

    elif "Global translation" in line:
        translation_values = [
            lines[line_index].strip().split()[2:],
            lines[line_index + 1].strip().split(),
            lines[line_index + 2].strip().split()
        ]
        
        flat_translation_values = [value for sublist in translation_values for value in sublist]

        current_measurement["Global translation X"] = str(round(float(flat_translation_values[0].replace(',', '.')) * 1000, 2)).replace('.', ',')
        current_measurement["Global translation Y"] = str(round(float(flat_translation_values[1].replace(',', '.')) * 1000, 2)).replace('.', ',')
        current_measurement["Global translation Z"] = str(round(float(flat_translation_values[2].replace(',', '.')) * 1000, 2)).replace('.', ',')

    elif "Av. movement" in line:
        value = line.split(':')[1].strip()
        key, finalValue = process_value(value, "Av. movement", precision=2)
        current_measurement[key] = finalValue

        if "INITIAL" in section:
            av_movement = float(value.replace(',', '.'))

    elif "Av. error" in line:
        value = line.split(':')[1].strip()
        key, finalValue = process_value(value, "Av. error", precision=2)
        current_measurement[key] = finalValue

        if "INITIAL" in section:
            # Track the "Initial" values
            initial_av_error = float(value.replace(',', '.'))
        
        elif "FINAL" in section:
            # Track the "Final" values
            final_av_error = float(value.replace(',', '.'))

    elif "RMSE" in line:
        value = line.split(':')[1].strip()
        key, finalValue = process_value(value, "RMSE", precision=2)
        current_measurement[key] = finalValue
    
    line_index += 1

if current_measurement:
    measurements.append(current_measurement)


# Calculate values
if initial_av_error is not None and final_av_error is not None:
    try:
        improv_percentage = ((initial_av_error - final_av_error) / 
                             initial_av_error) * 100
    except ValueError:
        improv_percentage = None
else:
    improv_percentage = None

if av_movement is not None and final_av_error is not None:
    try:
        final_vs_mov_percentage = (final_av_error / 
                                   av_movement) * 100
    except ValueError:
        final_vs_mov_percentage = None
else:
    final_vs_mov_percentage = None


if measurements:
    last_measurement = measurements[-1]
    
    if improv_percentage is not None:
        last_measurement["Improv. (%)"] = "{:.2f}".format(improv_percentage).replace('.', ',')
    else:
        last_measurement["Improv. (%)"] = ""
    
    if final_vs_mov_percentage is not None:
        last_measurement["Final Vs Mov (%)"] = "{:.2f}".format(final_vs_mov_percentage).replace('.', ',')
    else:
        last_measurement["Final Vs Mov (%)"] = ""

# Step 4: Define Headers
ZeroRow = [
    {Triangulation},"","","","","","","","",
    "","","",
    "","","",
    "","","",""
]

FirstRow = [
    "Datos", "Shape", "Gaussian", "Rigid","Section", "C1 std dev", "C2 std dev", "Rel. error",
    "Global rotation X", "Global rotation Y", "Global rotation Z",
    "Global translation X", "Global translation Y", "Global translation Z",
    "Av. movement", "Av. error", "RMSE", "Improv. (%)", "Final Vs Mov (%)"
]

SecondRow = [
    "", "", "Mov.(mm)"
]

ThirdRow = [
    "Depth (mm)", "Shape", "Gaussian", "Rigid","", "", "", "",
    "X", "Y", "Z",
    "X", "Y", "Z",
    "", "", "", "", ""
]

# Step 5: Write the data to a CSV file
file_mode = 'w' if firstMeasure else 'a'
with open(out_file_path + '.csv', file_mode, newline='') as csv_file:
  
    csv_writer = csv.writer(csv_file)
    
    if firstMeasure:
        csv_writer.writerow(ZeroRow)
        csv_writer.writerow(FirstRow)
        csv_writer.writerow(SecondRow)
        csv_writer.writerow(ThirdRow)

    static_data = {
        "Depth (mm)": str(Depth).replace('.', ','),
        "Shape": Shape,
        "Gaussian": str(gaussianMov).replace('.', ','),
        "Rigid": str(rigidMov).replace('.', ','),
    }
    
    combined_measurements = [
        {**static_data, **{key: measurement.get(key, "") for key in FirstRow[1:] if key not in static_data}}
        for measurement in measurements
    ]

    for row in combined_measurements:
        csv_writer.writerow([row["Depth (mm)"]] + [row.get(header, "") for header in FirstRow[1:]])


print("CSV file created successfully with the selected section.")


csv_file = out_file_path + '.csv'
df = pd.read_csv(csv_file, decimal=',')


# Save the Data as an Excel file
excel_file = out_file_path + '.xlsx'
df.to_excel(excel_file, index=False, engine='openpyxl') 

