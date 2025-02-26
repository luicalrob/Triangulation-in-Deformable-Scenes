import argparse
import sys
import os
import pandas as pd

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import syncolon_values

default_values = syncolon_values

def validate_options(option, available_values, option_name):
    """Validate that the option exists in the available values."""
    if option not in available_values:
        raise ValueError(
            f"{option_name} '{option}' is not valid. "
            f"Available options are: {available_values}"
        )

def get_csv_filename(triangulation, experiment, data):
    """Generate CSV file name."""
    return f"{triangulation}_{experiment}_{data}.csv"

# Argument parser
parser = argparse.ArgumentParser(description="Process CSV data for experiments.")
parser.add_argument("--Pairs", nargs='+', required=True, help="List of pairs to include.")
parser.add_argument('--Triangulation', type=str, choices=default_values["Triangulation"], required=True, help="Triangulation type")
parser.add_argument("--Experiment", type=int, required=True, help="Experiment number (1-6).")
parser.add_argument("--Data", required=True, help="Initial or Final.")
args = parser.parse_args()

# Output file paths
output_file = f"./Data/Excels/Syncolon/Resumes/Data {args.Experiment} {args.Triangulation} {args.Data}"
csv_files = []
data_frames = []

# Validate input
selected_pairs = args.Pairs
triangulation = args.Triangulation
selected_experiment = args.Experiment
selected_data = args.Data

print(triangulation)
for pair in selected_pairs:
    validate_options(pair, default_values["Pair"], "Pair")
    
validate_options(triangulation, default_values["Triangulation"], "Triangulation")
validate_options(selected_experiment, default_values["Experiment"], "Experiment")

# Collect relevant CSV files
for pair in selected_pairs:
    initial_frame, step, final_frame = pair.split("-")

    DATA_DIR = f"./Data/Excels/Syncolon/{initial_frame}_{final_frame}"
    filename = get_csv_filename(triangulation, selected_experiment, selected_data)
    filepath = os.path.join(DATA_DIR, filename)

    if os.path.exists(filepath):
        df = pd.read_csv(filepath, header=0, dtype=str)
        
        # Generate the "Higher Info" column
        higher_info = f"{pair}_{triangulation}"
        df["Higher Info"] = higher_info
        
        # Append DataFrame
        data_frames.append(df)
    else:
        print(f"Warning: File {filename} not found in {DATA_DIR}.")

# Ensure that at least one file was processed
if not data_frames:
    raise FileNotFoundError("No valid CSV files found for the given parameters.")

# Merge data
merged_df = pd.concat(data_frames, ignore_index=True)

# Remove duplicate "Higher Info" values (only keep the first occurrence)
# merged_df["Higher Info"] = merged_df["Higher Info"].mask(merged_df["Higher Info"].duplicated(), '')

# Save as CSV
csv_file = output_file + '.csv'
os.makedirs(os.path.dirname(csv_file), exist_ok=True)
merged_df.to_csv(csv_file, index=False)
print(f"Results saved to {csv_file}")

df = pd.read_csv(csv_file, decimal=',')
excel_file = output_file + '.xlsx'
df.to_excel(excel_file, index=False, engine='openpyxl') 
