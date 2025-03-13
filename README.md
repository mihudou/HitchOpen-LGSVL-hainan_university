# HitchOpen-CIIC

This repository contains the autonomous driving system developed for the HitchOpen project. The system is divided into five main components: perception, localization, planning, control, and simulation.

## Getting Started

### System Requirements
- **Operating System**: Ubuntu 22.04 in native OS, virtual machine, or Windows Subsystem for Linux (WSL). Ubuntu 22.04 is what the race car runs on and it is strongly recommended you develop in native Ubuntu 22.04 Linux to avoid issues.
- **ROS Distribution**: [ROS2 Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)
- Notes: ARM64 architecture is untested.

### Hardware Recommandation
- **CPU**: 12th Gen Intel® Core™ i9-12900HK × 20 or above
- **GPU**: NVIDIA GeForce 3080 or above (for simulation)
- **Memory**: 64.0 GiB or above (for simulation)

### Dependencies
- **common.iron.repos**: Provides basic message types and modules for the stack. Most of them are modeified version from [The Autoware Foundation](https://github.com/autowarefoundation). [Autoware.universe](https://github.com/autowarefoundation/autoware.universe) provides many useful and insightful algorithms for autonomous driving stacks. 
- **svl.iron.repos**: Provides bridge to simulation

### Installation
We will assume you have ROS2 Iron installed and sourced in all these terminals.

1. Clone `HitchOpen-CIIC` and install `python3-vcstool`:
   ```bash
   git clone git@github.com:intelligentracing/HitchOpen-CIIC.git
   sudo apt update
   sudo apt install python3-vcstool
   ```

2. Import dependencies:
   ```bash
   cd HitchOpen-CIIC
   make vcs-import VCS_FILE=common.iron.repos
   make vcs-import VCS_FILE=svl.iron.repos # if working on SVL simulator
   ```

3. Install dependencies. In general if you run a VCS step post this then you need to rerun the `make rosdep-install` command:
   ```bash
   make rosdep-install-eol
   pip3 install environs
   ```

4. Install casadi:
   ```bash
   source /opt/ros/iron/setup.bash
   sudo apt install -y gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends
   sudo apt install -y -no-install-recommends coinor-libipopt-dev libslicot-dev
   cd ~/ && git clone https://github.com/casadi/casadi.git -b master
   cd casadi && mkdir build && cd build
   cmake -DWITH_IPOPT=ON -DWITH_SLICOT=ON -DWITH_QPOASES=ON -DWITH_OSQP=ON ... && make
   sudo make install
   echo export LD_LIBRARY_PATH="$(LD_LIBRARY_PATH):$(LD_LIBRARY_PATH):/usr/local/lib" >> ~/.bashrc
   source ~/.bashrc
   ```

5. (in progress)Build up the stack:
   ```bash
   make svl 
   ```

## Perception

The perception system is responsible for understanding the environment around the vehicle through various sensors and processing algorithms.

#### Dependencies
- `Autoware Universe`: https://github.com/autowarefoundation/autoware.universe


## Localization

The localization system determines the precise position and orientation of the vehicle in the environment.

#### Dependencies
- `Robot Localization`: https://github.com/cra-ros-pkg/robot_localization

#### Key Algorithms:
- `odometry_state_estimator`: Dynamically calibrates wheels speed from wheel reports and GPS, and outputs smoothed linear and angular velocity as odometry. 
- `vehicle_kinematic_state`: Merges odometry topics from different sources with local and global EKF, and outputs vehicle kinematic state.


## Planning

The planning system is responsible for generating safe and optimal global trajectories for autonomous navigation. 

#### Key Algorithms:
- `Race Path Planner*`: Generates optimal racing lines considering track boundaries and geometry and vehicle dynamics constraints.


## Control

The control system is responsible for executing the planned trajectory by generating appropriate steering and velocity commands. 

#### Key Algorithms:
- `Local Path Planner`: Generates a smooth, feasible trajectory considering vehicle dynamics and constraints.
- `Pure Pursuit Controller`: Provides robust path tracking by computing a look-ahead point on the path ahead of the vehicle, calculating steering commands to guide the vehicle toward this point and dynamically adjusting look-ahead distance based on vehicle speed.


## Simulation

We maintain a private fork of SVL simulator which adds richer vehicle dynamics to the vehicle. Combined with accurate sensor configuration, SVL was the ultimate solution for integration testing. 

#### Installation Steps
1. Download the release from [here](https://intelligentracingcom-my.sharepoint.com/:u:/g/personal/tianlun_zhang_intelligentracing_com/EVBmsMrNlrVCob8QErwZ7jEBM3x6QQZZTEedvdhRlpJm-w?e=J1WU67) and unzip it.
2. Download the `wise.zip` file from [here](https://intelligentracingcom-my.sharepoint.com/:u:/g/personal/tianlun_zhang_intelligentracing_com/ES8QdS9l-pBKgFhAtyaUQuQBil8Q7Qc9UqUZAgeWjPncOg?e=2RKUGa) and unzip it in the same folder as the simulator build

#### Expected Folder Structure
After extraction, you should have a folder structure like this:
```
OSSDC-SIM-ART-Linux # (if you downloaded the Linux version)
├── data
├── wise
├── OSSDC-SIM
├── start_sim_local.sh
├── start_sim_cloud.sh
├── config.yaml
└── ...
```

#### Running the Simulator

##### Terminal Types
| Terminal | Type | Description |
|----------|------|-------------|
| Pilot Terminal | WSL terminal / Linux terminal | A terminal open in the `HitchOpen-CIIC` folder and sourced. This can be done through `./source_all.sh` |
| OSSDC Sim Terminal | Windows cmd / WSL terminal / Linux Terminal | A terminal open in the simulator folder (`OSSDC-ART-Linux`) |

##### Manual Setup
Please source all the terminals properly.

1. Start the wise server (in OSSDC Sim Terminal):
   ```bash
   cd wise
   python3 -m http.server 9090
   ```
   Note: To change the port from 9090, edit `OSSDC-SIM-ART-Windows/start_sim_local.bat` or `OSSDC-SIM-ART-Linux/start_sim_local.sh`

2. Start the simulator (in OSSDC Sim Terminal):
   ```bash
   cd OSSDC-SIM-ART-Linux
   ./start_sim_local.sh
   ```

3. Start the ROS2 bridge (in Pilot Terminal):
   ```bash
   lgsvl_bridge --port 9091
   ```

4. Start the Python API (in Pilot Terminal):
   ```bash
   python3 src/launch/svl_launch/scripts/launch_ossdc.py --env svl.env
   ```
   Note: Map, sensors, and other configurations can be modified in `race.env`.

5. Simulate a dummy joystick, without which the car will not move. it won’t listen to any of your keyboard inputs, it is just there to make the stack think there is a joystick connected.
   ```bash
   python3 scripts/dummy_joy_command.py
   ```

6. (in progress)Start racing stack (in ART Pilot Terminal):
   ```bash
   ros2 launch svl_launch svl_all.launch.py
   ```

7. (in progress)Set parameters to control the vehicle(in Pilot Terminal):
   ```bash
   bash ./scripts/autonomous_params.sh
   ```

