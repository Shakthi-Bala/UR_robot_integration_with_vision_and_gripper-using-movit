# UR16e Manipulation Framework (ROS) 🤖🦾

This project provides a **ROS-based manipulation framework for the Universal Robots UR16e**, integrating:
- Official **UR ROS Driver**
- **MoveIt motion planning**
- A **central client + action server** for manipulation tasks
- RViz visualization
- Support for calibration updates (TCP, gripper, CoG)

---

## 📁 Repository Structure

```bash
.
├── final_pkg/          # Action server and central client for manipulation
├── launch/             # Custom launch files
├── scripts/            # Planning and task execution scripts
└── README.md
```

## 🔗 Required External Repositories
This project depends on the official Universal Robots ROS packages:

- UR ROS Driver
  https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

- UR Industrial Packages (MoveIt, descriptions, configs)
  https://github.com/ros-industrial/universal_robot

Ensure both repositories are cloned into your catkin workspace and built successfully.

## 🧰 Software Requirements

- Ubuntu 18.04 / 20.04
- ROS Melodic / Noetic
- Universal Robots UR16e
- MoveIt
- RViz

## 🛠️ Build Instructions
Clone this repository into your catkin workspace:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone <THIS_REPOSITORY_URL>
```
Also clone the required UR repositories:
```bash
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
git clone https://github.com/ros-industrial/universal_robot.git
```
Build the workspace:
```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```
## 🚀 Running the System (Real Robot)
To launch the UR16e robot with full manipulation support, open four terminals:

### Terminal 1: Bring up the UR Robot Driver
```bash
roslaunch ur_robot_driver ur16e_bringup.launch \
robot_ip:=192.168.1.102 \
kinematics_config:=/home/host/my_robot_calibration.yaml
```
This launches the UR controller and allows control via the teach pendant or ROS.

### Terminal 2: Start MoveIt Move Group
```bash
roslaunch ur16e_moveit_config move_group.launch
```

### Terminal 3: Launch RViz for Visualization
```bash
roslaunch ur16e_moveit_config moveit_rviz.launch
```
### Terminal 4: Start Manipulation Action Server + Central Client
```bash
roslaunch final_pkg sim_complete.launch
```
This launch file contains:

- Central client
- Action server for manipulation tasks
- Custom task execution logic

## 🔧 Robot Calibration (Important)
If the tool, gripper, TCP, or center of gravity is changed, recalibration is required.
Run:
```bash
roslaunch ur_calibration calibration_correction.launch \
robot_ip:=<robot_ip> \
target_filename:="${HOME}/my_robot_calibration.yaml"
```
Use the generated calibration file in the bringup launch.

## 📐 Tool Pose Retrieval
To obtain the tool’s current position, use the script:
```bash
scripts/Task_server.py
```
This script provides access to the tool pose using MoveIt and the TF tree.

## 📜 License
This project is intended for academic and research use.
Refer to the upstream UR repositories for their respective licenses.

## 👤 Author
Shakthi Bala
---

### ✅ Why this README is solid
- Clear **step-by-step launch sequence**
- Proper separation of driver, MoveIt, RViz, and custom logic
- Easy for **lab mates / reviewers / recruiters** to follow
- Matches your actual folder layout

If you want, I can:
- Add a **system architecture diagram**
- Create a **simulation-only version**
- Convert this into a **research project README**
- Clean and document `Task_server.py`

Just tell me 👍

