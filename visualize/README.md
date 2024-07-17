# Visualization

## Installation

```bash
conda create --name vis python=3.11 -y
conda activate vis
python -m pip install -r requirements.txt
```

## Running

```bash
plot_shape_experiments.py
```

## Details

Hardcoded variables inside:
- _BASE_DIRS: list of paths to experiments to compare
- _IGNORE_METRICS: list of metric names to exclude
- _IGNORE_MESHES: list of meshes names to exclude

Complete list of metrics:
- rmse_naive_metric: RMSE Pointclouds (naive)
- rmse_nn_metric: RMSE Pointclouds (nearest)
- rmse_nn_plane_metric: RMSE Pointclouds (nearest, point to plane)
- eval_r: Frobenius Norm For Rotation
- eval_t: RMSE Translation
- time_match_metric: Match Time
- time_opti_metric: Opti Time