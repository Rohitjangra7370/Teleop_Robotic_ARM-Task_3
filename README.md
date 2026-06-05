# Teleoperation — Robotic Arm

ROS2 package for teleoperation of a robotic arm using MoveIt2. Control joint positions interactively via keyboard or GUI, with real-time visualization in RViz.

## Overview

Implements joint-space and task-space teleoperation for a multi-DOF robotic arm. Uses MoveIt2 for motion planning and collision checking, ensuring safe movement throughout the workspace.

## Stack

- **ROS2 Humble**
- **MoveIt2** — motion planning and collision checking
- **Gazebo** — simulation
- **RViz + MoveIt2 plugin** — interactive marker control
- **Python / C++**

## Usage

```bash
colcon build
source install/setup.bash

# Launch arm simulation + MoveIt2
ros2 launch teleop_arm arm_launch.py
```

Use the RViz interactive marker or keyboard interface to command joint positions. MoveIt2 plans a collision-free trajectory before executing.

## Robotronics Club — IIT Mandi

Part of the Robotronics Club recruitment task series (Task 3).

## Author

Rohit Jangra · [github.com/Rohitjangra7370](https://github.com/Rohitjangra7370)
