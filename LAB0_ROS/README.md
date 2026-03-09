# LAB0 ROS2 Workspace

This folder contains the ROS2 workspace for Lab 0.

## Structure

- `ros2_ws/src/`: source packages (commit this)
- `ros2_ws/build/`: build artifacts (ignored)
- `ros2_ws/install/`: install artifacts (ignored)
- `ros2_ws/log/`: runtime/build logs (ignored)

## Quick Start

From `LAB0_ROS/ros2_ws`:

```bash
colcon build
source install/setup.bash
```

## Package Notes

Current package path:

- `ros2_ws/src/pkg_Anas`

## What to Commit

- Source code under `ros2_ws/src/`
- Package metadata files (`package.xml`, `setup.py`, `setup.cfg`)
- Launch/config/resource files and documentation

## What Not to Commit

- `ros2_ws/build/`
- `ros2_ws/install/`
- `ros2_ws/log/`
- Python cache files (`__pycache__`, `*.pyc`)
