"""
Visualize metric values vs iteration number averaged across different
meshes, differentiated by different experiment name (base_dirs).

Example directory structure:

icp
├──visualize
│    ├──plot_shape_experiments.py
│    ...
└──results
        ├──symmetric_nonlinear_nn_partial
        │   ├── dinosaur
        │   │   ├── mesh_joined.off
        │   │   ├── rmse_nn_metric.txt
        │   │   └── time_metric.txt
        │   ├── dragon
        │   │   ├── mesh_joined.off
        │   │   ├── rmse_nn_metric.txt
        │   │   └── time_metric.txt
        │   ├── jaguar
        │   ...
        └──point_linear_nn_partial
            ├── dinosaur
            │   ├── mesh_joined.off
            │   ├── rmse_nn_metric.txt
            │   └── time_metric.txt
            ├── dragon
            │   ├── mesh_joined.off
            │   ├── rmse_nn_metric.txt
            │   └── time_metric.txt
            ├── jaguar
            ...
"""

import os
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

# List of base directories
_BASE_DIRS = ["../results/linear_plane_nn_color", 
             "../results/linear_point_nn_color", 
             "../results/linear_symm_nn_color"]

_BASE_NAME_MAP = {
   "linear_point_nn_color": "Point-to-Point ICP",
   "linear_plane_nn_color": "Point-to-Plane ICP",
   "linear_symm_nn_color": "Symmetric ICP",
}

# Ignore these metrics and meshes
_IGNORE_METRICS = []
_IGNORE_MESHES = []

# Aliases
_MAP_NAMES = {
    "rmse_naive_metric": "RMSE Pointclouds (naive)",
    "rmse_nn_metric": "RMSE Pointclouds (nearest)",
    "rmse_nn_plane_metric": "RMSE Pointclouds (nearest, point to plane)",
    "eval_r": "Frobenius Norm For Rotation",
    "eval_t": "RMSE Translation",
    "time_match_metric": "Match Time",
    "time_opti_metric": "Opti Time",
}

# Initialize a dictionary to store data
data = {}

# Function to read data from a file and return it as a DataFrame
def read_metric_file(file_path):
    df = pd.read_csv(file_path, header=None, names=['iteration', 'metric'])
    return df

# Traverse the base directories and read the files
for base_dir in _BASE_DIRS:
    base_dir_name = os.path.basename(base_dir)
    data[base_dir_name] = {}
    for entity in os.listdir(base_dir):
        if entity in _IGNORE_MESHES:
            continue
        entity_dir = os.path.join(base_dir, entity)
        if os.path.isdir(entity_dir):
            if entity not in data[base_dir_name]:
                data[base_dir_name][entity] = {}
            for metric_file in os.listdir(entity_dir):
                if metric_file.endswith('.txt'):
                    metric_name = metric_file.split('.')[0]
                    file_path = os.path.join(entity_dir, metric_file)
                    if metric_name in _IGNORE_METRICS:
                        continue
                    if metric_name not in data[base_dir_name][entity]:
                        data[base_dir_name][entity][metric_name] = []
                    data[base_dir_name][entity][metric_name].append(read_metric_file(file_path))

# Convert nested dictionaries to a multi-index DataFrame
multi_index_data = {}
for base_dir_name, entities in data.items():
    for entity, metrics in entities.items():
        for metric_name, dfs in metrics.items():
            combined_df = pd.concat(dfs).groupby('iteration').mean()
            multi_index_data[(base_dir_name, entity, metric_name)] = combined_df['metric']

multi_index_df = pd.DataFrame(multi_index_data)


# Calculate averaged metrics and standard deviation of the same type across all subdirectories for each base directory
averaged_metrics_per_base_dir = {}
std_metrics_per_base_dir = {}
metric_types = set(metric_name for entity_metrics in data.values() for metric_name in entity_metrics[next(iter(entity_metrics))].keys())

for base_dir_name in data.keys():
    averaged_metrics_per_base_dir[base_dir_name] = {}
    std_metrics_per_base_dir[base_dir_name] = {}
    for metric_type in metric_types:
        metric_columns = [col for col in multi_index_df.columns if col[0] == base_dir_name and col[2] == metric_type]
        if metric_columns:
            averaged_metrics_per_base_dir[base_dir_name][metric_type] = multi_index_df[metric_columns].mean(axis=1)
            std_metrics_per_base_dir[base_dir_name][metric_type] = multi_index_df[metric_columns].std(axis=1)


# Plot averaged metrics with standard deviation for each base directory
sns.set_theme(style="whitegrid")
for metric_type in metric_types:
    plt.rc('axes', titlesize=20)     # fontsize of the axes title
    plt.rc('axes', labelsize=20)     # fontsize of the x and y labels
    plt.rc('xtick', labelsize=14)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=14)    # fontsize of the tick labels
    plt.rc('legend', fontsize=20)    # legend fontsize

    plt.figure(figsize=(12, 8))
    for base_dir_name, metrics in averaged_metrics_per_base_dir.items():
        if metric_type in metrics:
            mean_values = metrics[metric_type]
            std_values = std_metrics_per_base_dir[base_dir_name][metric_type]
            plt.plot(mean_values.index, mean_values, label=f'{_BASE_NAME_MAP[base_dir_name]} Mean')
            plt.fill_between(mean_values.index, mean_values - std_values, mean_values + std_values, alpha=0.2)
    plt.title(f'{_MAP_NAMES[metric_type]} averaged across all meshes')
    plt.xlabel('Iteration')
    plt.ylabel(_MAP_NAMES[metric_type])
    plt.legend()
    plt.show()
