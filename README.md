# Diffdrive Navigation & Description Packages

This repository contains ROS 2 packages for a differential drive robot with navigation and simulation support, including:
- **diffdrive_description**: URDF/Xacro robot model with RPLIDAR support for simulation and real robots.
- **diffdrive_navigation**: Navigation2 configuration and launch files for autonomous navigation with dynamic obstacle avoidance.

---

## Features
- **Nav2-ready**: Local and global costmaps, dynamic obstacle avoidance
- **Launch files**: For both robot state publishing and navigation
- **Example map and config** for quick start

---

## Quick Start

### 1. Build the Workspace
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. Launch Robot Model in RViz
```bash
ros2 launch diffdrive_description display.launch.py
```

### 3. Launch Navigation Stack
```bash
ros2 launch diffdrive_navigation navigation.launch.py
```

---

## Robot Model (URDF/Xacro)
- Modular Xacro files: `common_properties.xacro`, `mobile_base.xacro`, `my_robot.urdf.xacro`
- **RPLIDAR**: Simulated with Gazebo plugin, publishes LaserScan on `/scan`
- **Frame conventions**: `base_link`, `laser`, `odom`, `map`

---

## Navigation2 Configuration
- **Costmaps**: Local and global, with obstacle and inflation layers
- **LaserScan**: `/scan` topic used for dynamic obstacle avoidance
- **Controller**: MPPI (Model Predictive Path Integral) by default
- **Map**: Example map and config provided in `diffdrive_navigation/map/`

---

## Real Robot Integration
- Ensure your RPLIDAR node publishes `sensor_msgs/LaserScan` on `/scan` frame `laser`
- Adjust transforms if your hardware setup differs

---



---

## License
Apache 2.0

---

## Maintainer
Edwin George
