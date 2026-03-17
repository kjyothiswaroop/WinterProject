# SemOpenVoc-Nav

**Semantic Open-Vocabulary Navigation for Humanoid Robots**

## Overview

Humans navigate indoor spaces not by following coordinates, but through a semantic understanding of their surroundings — walking toward "the fridge" or "the whiteboard" and communicating these intentions through natural language. SemOpenVoc-Nav brings this capability to a humanoid robot.

This project implements a semantic navigation pipeline on the **Unitree G1** that combines SLAM-based 3D mapping, open-vocabulary object detection (SAM + CLIP), and voice interaction (Whisper) — enabling the robot to navigate to objects described in natural language, such as *"check the fridge and then throw the trash away."*

**Author:** Jyothiswaroop Kasina

## Setup

### Prerequisites

Ubuntu 24.04, ROS 2 Kilted, Unitree G1 robot with D435 camera and Livox MID360 lidar.

Clone and follow the instructions from [unitree_ros2](https://github.com/kjyothiswaroop/unitree_ros2). Make sure to remove `source /opt/ros/kilted/setup.bash` from your `~/.bashrc` after setup.

### Install

```bash
mkdir -p ~/ws/winter/src && cd ~/ws/winter/src
git clone https://github.com/kjyothiswaroop/SemOpenVoc-Nav.git
cd SemOpenVoc-Nav
vcs import ../ < deps.repos
```

This pulls in `livox_ros_driver2` alongside `SemOpenVoc-Nav` in the workspace. Then build:

```bash
cd ~/ws/winter
colcon build
```
