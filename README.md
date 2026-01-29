# WinterHack
The WinterHack Challenge is an annual robotics-centric learning competition organized by Queen Mary University of London (QMUL). For its 2026 edition, under the theme "Operation Rescue: Search, Locate and Retrieve", participating teams are tasked with engineering a fully autonomous mobile robot (AMR) capable of exploring uncharted maze environments, identifying objects based on color cues, retrieving target items via a robotic arm, and navigating back to its initial deployment point without human intervention.

This repository features the official ROS2 development workspace designed exclusively for WinterHack. It serves as the foundational technical backbone supporting three key pillars: virtual simulation testing, hardware development for physical robots, and the on-site finals of the challenge. Unlike the comprehensive, general-use development repository (accessible at https://github.com/jonaloo19/ros2_humble_landerpi), this version has been extensively refactored and streamlined to align precisely with WinterHack’s unique requirements and use cases.


## Install flow
Clone this repo (contains the install scripts and the ros2_ws workspace):
```bash
git clone https://github.com/Tacoco666/winterhack_m.git
cd winterhack
```
Install ROS2 Humble (skips if already present):
```bash
chmod +x install_ros2_humble.sh
./install_ros2_humble.sh
```
Install dependencies, copy the workspace to `~/ros2_ws`, and build:
```bash
chmod +x install_gazebo_landerpi.sh
./install_gazebo_landerpi.sh
```
This script installs the ROS2 + Gazebo tooling needed for simulation and navigation, copies the provided workspace into `~/ros2_ws`, builds it, and updates `~/.bashrc` with the workspace and Gazebo resource paths (plus a CPU-render fallback you can disable for GPU rendering).

After running, open a new terminal (or `source ~/.bashrc`) to load the environment.

## Workspace highlights (ros2_ws/src)
- `winterhack`: main package with mission logic and nodes (detect/locate/pick/drop), launch files, configs, maps, URDF, and RViz setup (includes Challenge 1/2 mission logic).
- `winterhack_interfaces`: action definitions for Pick, Drop, and Locate (extended for Challenge 2 priority ordering).
- `robot_gazebo`: Gazebo worlds and launch files (including `winterhack_maze`, `challenge1_world`, `challenge2_world`), models, and sim configs.
- `landerpi_description`: robot description package (URDF + meshes) used by Gazebo/RViz.
- `holonomic_sim`: holonomic base simulation plugin and test world used by the sim stack.
- `costmap_converter`, `teb_local_planner`, `trac_ik`: upstream nav/IK dependencies needed by the navigation and manipulation stack.

## Manual Workspace (ros2_ws) build
Use the provided colcon build command when you need to rebuild the workspace manually (for example, after developing a custom ROS application for challenges). Run:
```bash
cd ~/ros2_ws
colcon build --symlink-install
```

## Running WinterHack Core Demos
### Launch Gazebo Base Worlds
#### Robocup Home (Base Manipulation Test)
Launches the `robocup_home` world to exercise the WinterHack base/arm manipulation workflow (locate, pick, drop).
```bash
ros2 launch robot_gazebo worlds.launch.py world_name:=robocup_home
```


#### WinterHack Maze (Full AMR Stack Test)
Launches the `winterhack_maze` world to exercise the full WinterHack AMR stack: navigation, base/arm manipulation, and the mission runner driving the search → locate → pick → retrieve → drop workflow.
```bash
ros2 launch robot_gazebo worlds.launch.py world_name:=winterhack_maze
```


### Launch WinterHack Core Stack
This launch brings up SLAM Toolbox, Nav2, MoveIt2, and the WinterHack ROS nodes/action servers for colour detection, locate, pick, and drop (compatible with all challenge modes).
```bash
ros2 launch winterhack winterhack.launch.py
```
**Note:**
- `robot_mode:=sim` (default) uses `use_sim_time` (default true, `/clock`) and keeps the Gazebo control path for MoveIt; navigation outputs to `/controller/cmd_vel`.
- `robot_mode:=real` forces `use_sim_time:=false`, disables the Gazebo control path, enables real controllers, and publishes `/cmd_vel`.

### Mission Runner (Core Navigation + Manipulation)
Runs the end-to-end mission loop by coordinating navigation and manipulation: it searches for colour objects, locates the object, performs pick, retrieves, and drops (compatible with challenge-specific configs).
```bash
ros2 run winterhack mission_runner
```
**Note:** Requires WinterHack bringup running (via `winterhack.launch.py`).

### Action Server Commands
Exercise core action servers (launched via `winterhack.launch.py`) to test pick/drop/locate behaviour:
```bash
# Test Pick action
ros2 action send_goal /pick winterhack_interfaces/action/Pick "{}"

# Test Drop action
ros2 action send_goal /drop winterhack_interfaces/action/Drop "{}"

# Test Locate action (with color priority)
ros2 action send_goal /locate winterhack_interfaces/action/Locate "{}"
ros2 param get /locate color_priority
ros2 param set /locate color_priority "['RED']"
```

## Running WinterHack Challenge Missions
### Challenge 1: Known-coordinate Rescue (Efficiency-Driven)
#### Objective
Rescue all three target blocks (RED, BLUE, YELLOW) from known coordinates provided in advance, and deposit them into the home drop zone. No search or exploration is required.
#### Engineering Emphasis
- Global and local path-planning optimisation
- Efficient navigation and localisation
- Reliable pick-and-place manipulation
- Efficient task sequencing

#### Step 1: Launch Challenge 1 Gazebo World
```bash
ros2 launch robot_gazebo worlds.launch.py world_name:=challenge1_world
```
#### Step 2: Launch WinterHack (Challenge 1 Config)
```bash
ros2 launch winterhack winterhack.launch.py challenge_mode:=challenge1
```
#### Step 3: Run Challenge 1 Mission Runner
```bash
ros2 run winterhack mission_runner --ros-args -p challenge_mode:=challenge1
```
#### Demo Video (Challenge 1)

[demo1.mp4](media/demo1.mp4)
</p>
*Demo Notes: The video demonstrates the robot completing efficient rescue of RED/BLUE/YELLOW blocks from pre-defined coordinates, with optimised path planning and task sequencing.*

### Challenge 2: Priority-Order Rescue (Decision-Driven)
#### Objective
Rescue all three target blocks in the strict order RED → BLUE → YELLOW, regardless of their spatial distribution within the maze, and deposit them into the home drop zone.
#### Engineering Emphasis
- Efficient search and perception
- Task sequencing and symbolic decision-making
- Robust state management
- Reliable autonomy under ordering constraints

#### Step 1: Launch Challenge 2 Gazebo World
```bash
ros2 launch robot_gazebo worlds.launch.py world_name:=challenge2_world
```
#### Step 2: Launch WinterHack (Challenge 2 Config)
```bash
ros2 launch winterhack winterhack.launch.py challenge_mode:=challenge2
```
#### Step 3: Run Challenge 2 Mission Runner
```bash
ros2 run winterhack mission_runner --ros-args -p challenge_mode:=challenge2
```
#### Demo Video (Challenge 2)

[demo2.mp4](media/demo2.mp4)
</p>
*Demo Notes: The video shows the robot autonomously searching for randomly distributed blocks, adhering to the RED→BLUE→YELLOW priority order, with robust state management and decision-making.*

## Kill ROS Processes
Stops existing ROS/Gazebo processes before relaunching to avoid conflicts like multiple action servers or duplicate topic publishers.
```bash
# One-time setup to make the script executable
chmod +x killros.sh

# Terminate running ROS/Gazebo processes
./killros.sh
```

## Summary
### Core Workflow
1. Start with a base simulation world (`robocup_home`/`winterhack_maze`) to validate core manipulation/navigation.
2. Bring up the WinterHack stack via `winterhack.launch.py` (specify `robot_mode` for sim/real).
3. Run the `mission_runner` to test end-to-end core behaviour.

### Challenge-Specific Workflow
1. Launch the challenge-specific Gazebo world (`challenge1_world`/`challenge2_world`).
2. Launch the WinterHack stack with `challenge_mode` set to the target challenge.
3. Run the `mission_runner` with the corresponding `challenge_mode` parameter to execute the challenge mission.

Use action/parameter commands to test individual Pick/Drop/Locate behaviours, and run the `killros.sh` script to reset ROS/Gazebo before re-running demos/challenges.