#!/usr/bin/env python3
import csv
import re
import os
import pandas as pd

#######  Inputs  ########

# "ARAP_NoGlobal" "ARAP", "Elastic, HyperElastic"
Model = "ARAP"      
# "InRays" or "TwoPoints"            
Triangulation = "InRays"  
# 20, 80, 150   
Depth = 80       
# "Planar" or "Gradual"               
Shape = "Gradual"   
# 0, 2.5, 10             
gaussianMov = 10  
# 0, 2.5, 10            
rigidMov = 10     
# 1, 2, 3, 4, 5           
Experiment = 1           

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
out_file_path = './Data/Excels/'+ Model + "_" + Triangulation

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
        current_measurement["C1 std dev"] = line.split(':')[1].strip().replace('.', ',')

    elif "C2 standard desv" in line:
        current_measurement["C2 std dev"] = line.split(':')[1].strip().replace('.', ',')

    elif "Rel. error" in line:
        current_measurement["Rel. error"] = line.split(':')[1].strip().replace('.', ',')

    elif "Global rotation" in line:
        rotation_matrix = [
            lines[line_index].strip().split()[2:],
            lines[line_index + 1].strip().split(),
            lines[line_index + 2].strip().split()
        ]
        
        current_measurement["Global rotation X"] = rotation_matrix[0][0].replace('.', ',')
        current_measurement["Global rotation Y"] = rotation_matrix[1][1].replace('.', ',')
        current_measurement["Global rotation Z"] = rotation_matrix[2][2].replace('.', ',')
    elif "Global translation" in line:
        translation_values = [
            lines[line_index].strip().split()[2:],
            lines[line_index + 1].strip().split(),
            lines[line_index + 2].strip().split()
        ]
        
        flat_translation_values = [value for sublist in translation_values for value in sublist]

        current_measurement["Global translation X"] = str(float(flat_translation_values[0].replace(',', '.')) * 1000).replace('.', ',')
        current_measurement["Global translation Y"] = str(float(flat_translation_values[1].replace(',', '.')) * 1000).replace('.', ',')
        current_measurement["Global translation Z"] = str(float(flat_translation_values[2].replace(',', '.')) * 1000).replace('.', ',')

    elif "Av. movement" in line:
        current_measurement["Av. movement"] = line.split(':')[1].strip().replace('.', ',')

        if "INITIAL" in section:
            av_movement = current_measurement.get("Av. movement", None)

    elif "Av. error" in line:
        current_measurement["Av. error"] = line.split(':')[1].strip().replace('.', ',')

        if "INITIAL" in section:
            # Track the "Initial" values
            initial_av_error = current_measurement.get("Av. error", None)
        
        elif "FINAL" in section:
            # Track the "Final" values
            final_av_error = current_measurement.get("Av. error", None)

    elif "RMSE" in line:
        current_measurement["RMSE"] = line.split(':')[1].strip().replace('.', ',')
    
    line_index += 1

if current_measurement:
    measurements.append(current_measurement)


# Calculate values
if initial_av_error is not None and final_av_error is not None:
    try:
        improv_percentage = ((float(initial_av_error.replace(',', '.')) - float(final_av_error.replace(',', '.'))) / 
                             float(initial_av_error.replace(',', '.'))) * 100
    except ValueError:
        improv_percentage = None
else:
    improv_percentage = None

if av_movement is not None and final_av_error is not None:
    try:
        final_vs_mov_percentage = (float(final_av_error.replace(',', '.')) / 
                                   float(av_movement.replace(',', '.'))) * 100
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
    "InRays","","","","","","","","",
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
with open('./Data/Excels/'+ Model + "_" + Triangulation + '.csv', file_mode, newline='') as csv_file:
  
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
df = pd.read_csv(csv_file)

# Save the Data as an Excel file
excel_file = out_file_path + '.xlsx'
df.to_excel(excel_file, index=False, engine='openpyxl') 
