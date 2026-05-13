# ros2_ws

ROS 2 workspace utilities: Python scripts for UR5 kinematics (FK/IK via URDF), demos, and Gazebo-related helpers, plus URDF models.

## Layout

- `scripts/` — kinematics, demos, and arm abstractions
- `urdf/` — robot description (e.g. `ur5.urdf`)

## Requirements

Python dependencies used by the scripts (e.g. `numpy`, `ikpy`) should be installed in your environment. For ROS 2 packages, add them under `src/` and build with `colcon` as usual; this repo ignores `build/`, `install/`, and `log/` when present.
