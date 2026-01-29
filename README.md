# winterhack_m
This project is a complete ROS2-based development and simulation framework for the LanderPi mobile manipulator, tailored for WinterHack competition development and verification of mobile manipulator-related tasks.

## 1. Directory Structure
```
winterhack_m/
├── .typerc                      # Typo syntax check configuration file
├── 0_kill_all.sh                # One-click termination of all ROS2 nodes/simulation processes
├── 1_start_gazebo.sh            # Launch Gazebo simulation environment
├── 2_start_winterhack.sh        # Launch WinterHack core function modules
├── 3_start_challenge2.sh        # Launch Challenge 2 task process
├── 4_start_challenge1.sh        # Launch Challenge 1 task process
├── diagnose_and_fix.sh          # Environment diagnosis and automatic repair script
├── src/                         # ROS2 package source code directory
│   ├── costmap_converter        # Cost map conversion (adapted to path planner)
│   ├── holonomic_sim            # Holonomic mobile robot simulation and control
│   ├── landerpi_description     # Robot URDF model/joint/sensor definition
│   ├── robot_gazebo             # Gazebo simulation scene/model loading configuration
│   ├── teb_local_planner        # TEB local path planner
│   ├── trac_ik                  # TRAC-IK inverse kinematics solution library
│   ├── winterhack               # Competition core logic (task scheduling/data processing/control)
│   └── winterhack_interfaces    # Custom ROS2 interfaces (messages/services/actions)
└── README.md                    # Project documentation
```

## 2. Environmental Dependencies
### 2.1 Basic Environment
- Operating System: Ubuntu 22.04 LTS (recommended)
- ROS2 Version: Humble Hawksbill (compatible with all packages in the workspace)

### 2.2 Dependency Installation
```bash
# Install ROS2 basic dependencies
sudo apt update && sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-teb-local-planner \
  ros-humble-trac-ik \
  ros-humble-urdf-tutorial

# Install build tools
sudo apt install -y python3-colcon-common-extensions python3-pip
```

## 3. Quick Start
### 3.1 Clone the Repository
```bash
git clone <your-repository-url> winterhack_m
cd winterhack_m
```

### 3.2 Build the Workspace
```bash
# Install package dependencies
rosdep install --from-paths src --ignore-src -r -y

# Compile the workspace (--symlink-install avoids repeated compilation)
colcon build --symlink-install

# Load workspace environment (execute in every new terminal)
source install/setup.bash
```

### 3.3 Run Simulation/Tasks
#### 3.3.1 Basic Simulation Launch
```bash
# Step 1: Launch Gazebo simulation environment (Terminal 1)
./1_start_gazebo.sh

# Step 2: Launch WinterHack core functions (Terminal 2, source environment first)
source install/setup.bash
./2_start_winterhack.sh
```

#### 3.3.2 Launch Specified Challenge Tasks
```bash
# Launch Challenge 1 (Terminal 3)
source install/setup.bash
./4_start_challenge1.sh

# Launch Challenge 2 (Terminal 3)
source install/setup.bash
./3_start_challenge2.sh
```

#### 3.3.3 Terminate All Processes
```bash
./0_kill_all.sh
```

### 3.4 Environment Diagnosis and Repair
If you encounter compilation failures, missing dependencies, simulation exceptions, etc., execute one-click diagnosis and repair:
```bash
./diagnose_and_fix.sh
```

## 4. Core Package Description
| Package Name              | Core Functions                                                                 |
|---------------------------|--------------------------------------------------------------------------------|
| costmap_converter         | Cost map format conversion, raster map to polygon, adapt to TEB planner input  |
| holonomic_sim             | Holonomic robot kinematics solution, speed command parsing, simulation status feedback |
| landerpi_description      | Robot URDF model definition, joint limit configuration, laser/camera sensor mounting |
| robot_gazebo              | Gazebo world scene configuration, robot model loading, simulation plugin (e.g., odometry/contact sensor) configuration |
| teb_local_planner         | Time Elastic Band-based local path planning, supporting obstacle avoidance and motion optimization for non-holonomic constrained robots |
| trac_ik                   | High-precision inverse kinematics solution library, providing solution support for robot joint control |
| winterhack                | Competition core logic: state machine management, sensor data parsing, control command issuance, task result reporting |
| winterhack_interfaces     | Custom ROS2 interfaces: including custom messages/services/actions for task commands, status feedback, sensor data, etc. |

## 5. Common Problem Solving
### 5.1 Script Execution Prompts "Insufficient Permissions"
```bash
chmod +x *.sh  # Add execution permission to all scripts
```

### 5.2 Compilation Error "Cannot Find xxx Package/Header File"
```bash
# Complete all dependencies
rosdep install --from-paths src --ignore-src -r -y
# Recompile
colcon build --symlink-install --packages-select <error-package-name>
```

### 5.3 Gazebo Launches with No Robot/Blank Scene
- Check the launch files under `robot_gazebo/launch/` to ensure the URDF/world file paths are correct;
- Execute `source install/setup.bash` and restart Gazebo;
- Check the Gazebo model path: `echo $GAZEBO_MODEL_PATH` to ensure it includes the robot model directory.

### 5.4 ROS2 Node Communication Abnormal (No Data on Topics/Service Call Failure)
- All terminals must execute `source install/setup.bash`;
- Check the `ROS_DOMAIN_ID` environment variable to ensure all nodes are in the same domain;
- Use `ros2 node list`/`ros2 topic list` to confirm nodes/topics are started correctly.

## 6. Development Specifications
1. **Package Management**: Prioritize extending existing packages for new functions; if a new package needs to be created, follow ROS2 naming conventions (lowercase + underscore);
2. **Interface Specifications**: All custom messages/services/actions are uniformly placed in the `winterhack_interfaces` package, and recompilation is required after modification;
3. **Script Specifications**: New launch scripts follow the existing naming rules (numeric prefix + function description) and add comments to explain the purpose;
4. **Compilation Optimization**: Use `colcon build --symlink-install` during development, and use Release compilation for release:
   ```bash
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```
5. **Code Submission**: Ensure no compilation warnings and core functions run normally before submission.

## 7. License
This repository is only used for WinterHack competition/project development, and all rights are reserved by the project team. Unauthorized modification and distribution are prohibited.

## 8. Contact Information
For technical issues, contact the project maintainer: <your-email/contact-information>