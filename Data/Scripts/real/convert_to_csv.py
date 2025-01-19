#!/usr/bin/env python3
import csv
import re
import sys
import os
import pandas as pd
import argparse
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import real_values

default_values = real_values

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

def parse_arguments():
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="Set parameters for the experiment.")
    parser.add_argument('--Pair', type=str, choices=default_values["Pair"], required=True, help="Pair of frames")
    parser.add_argument('--Triangulation', type=str, choices=default_values["Triangulation"], required=False, help="Triangulation type")
    parser.add_argument('--Level', type=str, choices=default_values["Level"], required=False, help="Deformation level")
    parser.add_argument('--Checks', type=str, choices=default_values["Checks"], required=False, help="Check essential matrix")
    parser.add_argument('--Experiment', type=int, choices=default_values["Experiment"], required=False, help="Experiment number")
    return parser.parse_args()

def read_experiment_file(file_path):
    """Read and return lines from the experiment text file."""
    with open(file_path, 'r') as txt_file:
        return txt_file.readlines()

def initialize_measurements(output_path):
    """Determine if this is the first measurement and initialize output file."""
    is_first_measurement = not os.path.exists(output_path) or os.stat(output_path).st_size == 0
    return is_first_measurement

def convert_to_float(value):
    if '.' in value:
        value = value.replace('.', '')
    value = value.replace(',', '.')
    # Return the value converted to float
    return float(value)

def extract_data(lines):
    """Extract data from lines of the experiment file."""
    measurements, current_measurement = [], {}
    section, initial_av_error, final_av_error, av_movement = "", None, None, None
    
    for line_index, line in enumerate(lines):
        line = line.strip()

        if re.search(r"INITIAL MEASUREMENTS|\d+ / \d+ MEASUREMENTS|FINAL MEASUREMENTS", line):
            if current_measurement:
                measurements.append(current_measurement)
            section = re.sub(r'\s*MEASUREMENTS\s*', '', line).replace(':', '')
            current_measurement = {"Section": section}

        elif any(keyword in line for keyword in ["Translation norm between cameras (mm)", "Parallax", "nMatches", "nMapPoints", "C1 standard desv", "C2 standard desv", "Rel. error", "Av. movement", "Av. error", "RMSE"]):
            value = line.split(':')[1].strip()
            title = line.split(':')[0]

            if title == "Av. movement":
                if "INITIAL" in section:
                    # Track the "Initial" values
                    av_movement = float(convert_to_float(value))
            elif title == "Av. error":
                if "INITIAL" in section:
                    # Track the "Initial" values
                    initial_av_error = float(convert_to_float(value))
                elif "FINAL" in section:
                    # Track the "Final" values
                    final_av_error = float(convert_to_float(value))

            key, final_value = process_value(value, title, precision=2)
            current_measurement[key] = final_value
        
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

            current_measurement["Global translation X"] = str(round(float(convert_to_float(flat_translation_values[0])) * 1000, 2)).replace('.', ',')
            current_measurement["Global translation Y"] = str(round(float(convert_to_float(flat_translation_values[1])) * 1000, 2)).replace('.', ',')
            current_measurement["Global translation Z"] = str(round(float(convert_to_float(flat_translation_values[2])) * 1000, 2)).replace('.', ',')

    if current_measurement:
        measurements.append(current_measurement)

    return measurements, initial_av_error, final_av_error, av_movement

def write_csv(measurements, output_path, is_first_measurement, metadata):
    """Write measurements to a CSV file."""
    headers = [
        "Level", "Checks", "Section", "C1 std dev", "C2 std dev", "Rel. error",
        "Global R X", "Global R Y", "Global R Z", "Global t X", "Global t Y", "Global t Z",
        "Av. movement", "Av. error", "RMSE", "Improv. (%)", "Final Vs Mov (%)",
        "t C1C2 norm (mm)", "pllx", "nMchs", "nMPs"
    ]

    with open(output_path, 'w' if is_first_measurement else 'a', newline='') as csv_file:
        writer = csv.writer(csv_file)
        if is_first_measurement:
            writer.writerow([metadata['Triangulation']])
            writer.writerow(headers)

        for measurement in measurements:
            row = [metadata['Level'], metadata['Checks']] + [measurement.get(header, "") for header in headers[2:]]
            writer.writerow(row)

def calculate_improvement_percentage(initial_av_error, final_av_error):
    if initial_av_error is not None and final_av_error is not None:
        try:
            return ((initial_av_error - final_av_error) / initial_av_error) * 100
        except (ValueError, ZeroDivisionError):
            return None
    return None


def calculate_final_vs_movement_percentage(av_movement, final_av_error):
    if av_movement is not None and final_av_error is not None:
        try:
            return (final_av_error / av_movement) * 100
        except (ValueError, ZeroDivisionError):
            return None
    return None


def add_values_to_last_measurement(measurements, improv_percentage, final_vs_mov_percentage):
    """
    Add improvement and final vs movement percentages to the last measurement.

    Args:
        measurements (list): List of measurement dictionaries.
        improv_percentage (float or None): Improvement percentage.
        final_vs_mov_percentage (float or None): Final vs Movement percentage.
    """
    if not measurements:
        return

    last_measurement = measurements[-1]

    if improv_percentage is not None:
        last_measurement["Improv. (%)"] = "{:.2f}".format(improv_percentage).replace('.', ',')
    else:
        last_measurement["Improv. (%)"] = ""

    if final_vs_mov_percentage is not None:
        last_measurement["Final Vs Mov (%)"] = "{:.2f}".format(final_vs_mov_percentage).replace('.', ',')
    else:
        last_measurement["Final Vs Mov (%)"] = ""

    for measurement in measurements:
        if "Translation norm between cameras (mm)" in measurement:
            last_measurement["t C1C2 norm (mm)"] = measurement["Translation norm between cameras (mm)"]
        if "nMapPoints" in measurement:
            last_measurement["nMPs"] = measurement["nMapPoints"]
        if "nMatches" in measurement:
            last_measurement["nMchs"] = measurement["nMatches"]
        if "Parallax" in measurement:
            last_measurement["pllx"] = measurement["Parallax"]

def save_to_excel(csv_path, excel_path):
    """Convert CSV data to Excel format."""
    df = pd.read_csv(csv_path, decimal=',')
    df.to_excel(excel_path, index=False, engine='openpyxl')

def main():
    args = parse_arguments()
    
    Pair = args.Pair
    Triangulation = args.Triangulation or "FarPoints"
    Level = args.Level or "level0"
    Checks = args.Checks or "no_checks"
    Experiment = args.Experiment or 1

    file_path = f'./Data/Experiments/Drunkard/{Pair}/{Triangulation}/{Level}/{Checks}/{Experiment}/Experiment.txt'
    output_path = f'./Data/Excels/Drunkard/{Pair}/{Triangulation}_{Experiment}.csv'

    lines = read_experiment_file(file_path)
    is_first_measurement = initialize_measurements(output_path)

    measurements, initial_av_error, final_av_error, av_movement = extract_data(lines)

    improv_percentage = calculate_improvement_percentage(initial_av_error, final_av_error)
    final_vs_mov_percentage = calculate_final_vs_movement_percentage(av_movement, final_av_error)

    add_values_to_last_measurement(measurements, improv_percentage, final_vs_mov_percentage)

    write_csv(measurements, output_path, is_first_measurement, {
        "Triangulation": Triangulation,
        "Level": Level,
        "Checks": Checks
    })

    save_to_excel(output_path, output_path.replace('.csv', '.xlsx'))
    print("CSV and Excel files created successfully.")

if __name__ == "__main__":
    main()
