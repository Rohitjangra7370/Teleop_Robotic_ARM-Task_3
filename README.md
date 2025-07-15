# Teleop Robotic Arm - Task 3

## Overview

This repository contains a ROS2 package for simulating and teleoperating a 6DOF robotic arm equipped with a Robotiq 85 gripper. The project integrates Gazebo simulation, ROS2 control systems, and custom teleoperation scripts for keyboard-based control. It's designed for tasks involving manipulator control, trajectory planning, and Cartesian velocity commands in a simulated environment[1].

Key features include:
- URDF model of the robotic arm with joint limits and inertial properties.
- Launch files for Gazebo simulation and robot state publishing.
- Python scripts for simple teleoperation, keyboard input, and Cartesian-to-joint velocity conversion.
- Mesh files for the gripper visualization.

## Prerequisites

To run this project, ensure you have:
- ROS2 (Humble or later) installed.
- Gazebo (Garden or later) for simulation.
- Required ROS2 packages: `ros2_control`, `gz_ros2_control`, `robot_state_publisher`, `xacro`, `geometry_msgs`, `trajectory_msgs`, `sensor_msgs`.

Install dependencies via:
```
rosdep install --from-paths src --ignore-src -r -y
```

## Installation

1. Clone the repository:
   ```
   git clone https://github.com/rohitjangra7370/teleop_robotic_arm-task_3.git
   cd teleop_robotic_arm-task_3
   ```

2. Build the package:
   ```
   colcon build --packages-select manipulator
   source install/setup.bash
   ```

## Usage

### Launching the Simulation

- To start Gazebo with the robotic arm:
  ```
  ros2 launch manipulator gazebo.launch.py
  ```
  This spawns the arm in an empty world and sets up the ROS-Gazebo bridge[1].

- For a complete teleoperation setup (including RViz):
  ```
  ros2 launch manipulator teleop_complete.launch.py
  ```
  This launches Gazebo, spawns the robot, loads controllers, and starts RViz for visualization.

- Test launch (with optional Gazebo):
  ```
  ros2 launch manipulator test.launch.py use_gazebo:=true
  ```

### Teleoperation Scripts

Run these scripts after launching the simulation to control the arm.

- **Keyboard Teleoperation**:
  ```
  ros2 run manipulator keyboard_teleop.py
  ```
  Use keys like W/S for linear X, A/D for Y, Q/E for Z, and I/K/J/L/U/O for rotations. Press SPACE to stop, ESC to quit[1].

- **Cartesian Controller**:
  ```
  ros2 run manipulator cartesian_controller.py
  ```
  Subscribes to `/arm_cartesian_velocity` (Twist messages) and publishes joint trajectories to `/arm_controller/joint_trajectory`. It converts Cartesian velocities to joint commands with basic mapping and limits[1].

- **Simple Teleoperation**:
  ```
  ros2 run manipulator simple_teleop.py
  ```
  Command-line interface for incremental joint movements (e.g., '1' to increase base joint by 0.2 rad). Enter '0' to reset or 'q' to quit[1].

### Controllers

The arm uses a `JointTrajectoryController` defined in `config/controllers.yaml`. Joints include base rotation, shoulder, upper arm, forearm, and wrists. Controllers are loaded automatically in launch files[1].

## Project Structure

- **urdf/**: Contains `robot_core.urdf.xacro` with the arm's kinematic model and Robotiq gripper integration.
- **launch/**: Python launch files for simulation and teleoperation.
- **config/**: YAML files for ROS2 controllers.
- **scripts/**: Python nodes for control and teleop.
- **meshes/**: STL files for gripper components (coarse and fine variants).

## Troubleshooting

- If Gazebo fails to spawn the robot, verify the URDF path and ROS2 sourcing.
- Joint limits are enforced in scripts to prevent invalid positions.
- For real hardware integration, modify the `ros2_control` plugin in the URDF.

## Contributing

Feel free to open issues or pull requests for improvements, such as advanced IK solvers or better velocity mapping. This project is a starting point for robotic arm teleoperation tasks[1].

working:https://drive.google.com/drive/folders/1AE7zTh7yydI7f0yBz0ihsfpwbSRC41ob?usp=sharing
