# Hybrid Controller

This repository contains the top-level architecture for hybrid control 
systems, currently focusing on the WaterTank model. It uses the `Breach` falsification
toolbox and a control gitlab repository named: `stl-control-imitation`.

## Repository Structure

The workspace is organized to strictly separate master configurations, external libraries, and specific system models.

```text
hybrid-controller/
├── data/                         # Simulation and experiment data output
├── logs/                         # Execution logs
├── stl-control-imitation/        # (Git Submodule) Core control library
├── WaterTank_hybrid/             # WaterTank specific models and scripts
│   ├── tuning_controllers/       # Process for designing individual controllers
│   ├── WaterTank_model/          # Contains imitation learning files
│   ├── init_watertank.m          # WaterTank initialization
│   ├── run_watertank.m           # Main execution script for WaterTank
│   └── test_watertank.m          # Tests if expert runs
├── init_workspace.m              # Master workspace initialization script
├── local_config_template.m       # Template for overriding Breach paths
└── setup.sh                      # Bash script to download dependencies
```

## Set Up Workspace

1. Clone the repo with a git submodule:

```
git clone --recursive <https://github.com/cindy-x-li/hybrid-controller.git>
cd hybrid-controller
```

2. Install Breach

This project requires Breach to evaluate Signal Temporal Logic (STL). Breach 
will be installed in a folder called `external` if the `step.sh` is used. This 
method only clones the Breach Github project. It does not use the built binary 
files for intel. Thus, depending on your machine, you may have to make additional modifications to 
successfully run Breach.  
```
chmod +x setup.sh
./setup.sh
```

If you already have Breach installed locally, then you can 

a. Copy `local_config_template.m` and rename it to `local_config.m`.

b. Open `local_config.m` and update the custom_breach_path variable with the absolute path to your existing Breach folder.


Notes: For the arm64 chip, Matlab was configured to use Xcode complier to 
build and to use the classic linker by modifying compile_stl_mex.m 
and CompileRobusthom.m


3. Initialize the workspace:

```
init_workspace
```

### MATLAB 
Download:
[License through UGA](https://fr.mathworks.com/academia/tah-portal/universite-grenoble-alpes-31749054.html)

Download following toolboxes:
1. Control System  
2. Deep Learning 
3. Optimization 
4. Parallel Computing