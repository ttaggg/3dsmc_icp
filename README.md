# [3D Capture] ICP

## Installation

If you use Windows or Linux, please use `CMakeLists_win_linux.txt` and change the name to `CMakeLists.txt`. And if you use Mac, please use `CMakeLists_macos.txt` and change the name to `CMakeLists.txt`.

To set up the project, please install dependency libraries: 
> Ceres, freeimage, FLANN, Eigen, Open3D, yaml-cpp

Configure and compile program:
```sh
cd path/to/icp
mkdir build
cd build
cmake ..
make
```

### Run the program
Simply use the command line with `3dsmc_icp` program:

```sh
icp/build$ .3dsmc_icp config_file.yaml
```

### Example config file setting

```sh
# Task
runShapeICP: true
runSequenceICP: false
# ICP type
useLinearICP: true
# ICP objective(s)
useSymmetric: true
# Correspondence method (NN / SHOOT)
correspondenceMethod: NN
useColors: true
# Other settings
matchingMaxDistance: 0.1
nbOfIterations: 10

# Error metrics
evaluateRMSENearest: true
evaluateTransforms: true
evaluateTime: true                      

experimentName: plane_linear_nn_partial  # Experiment name for folder name
dataDir: ../../Data/greyc_partial/       # Output directory
```

### Directory layout

    icp
    ├── build                   # Compiled files
    ├── configs                 # config files
        ├──sequence_symmetric_nonlinear_nn.yaml
        └──shapes_plane_linear_nn.yaml
    ├── external/data_utils     # External header files
    ├── include                 # Header files
    ├── src                     # Source files
    ├── CMakeLists.txt
    └── README.md
    ├──Data
        ├──greyc_partial
        ├──greyc_partial_debug
        └──rgbd_dataset_freiburg1_xyz
    └──Libs