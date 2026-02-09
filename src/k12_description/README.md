# k12_description

This package contains the robot description, simulation, and mapping tools for the K12 rover in ROS2 Humble with Gazebo Classic.

## Features
- **URDF/Xacro**: Robot model with VLP-16 LiDAR, IMU, and RGB-D camera
- **Gazebo Simulation**: Launch files for spawning the robot in an empty world
- **3D Point Cloud Mapping**: Custom mapping node with keyboard controls
- **RTAB-Map Visual SLAM**: Launch file for RGB-D SLAM

## Directory Structure
- `urdf/`         — Robot model files (URDF, Xacro, Gazebo plugins)
- `launch/`       — Launch files for simulation, mapping, and SLAM
- `scripts/`      — Custom Python nodes (e.g., mapping_node.py)
- `maps/`         — Saved point cloud maps and RTAB-Map databases

## Usage

### 1. Launch Simulation
```
ros2 launch k12_description gazebo.launch.py
```

### 2. 3D Point Cloud Mapping
**Recommended:** Run mapping node directly for keyboard controls:
```
ros2 run k12_description mapping_node
```
- Controls: s=save, l=load, e=end, r=resume, c=clear, q=quit

Or, use the launch file (keyboard input may not work in all terminals):
```
ros2 launch k12_description mapping.launch.py
```

### 3. Teleop
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 4. RTAB-Map Visual SLAM
```
ros2 launch k12_description rtabmap.launch.py
```

## Camera Topics
- `/camera/depth_camera/image_raw`         (RGB)
- `/camera/depth_camera/depth/image_raw`   (Depth)
- `/camera/depth_camera/camera_info`
- `/camera/depth_camera/points`            (PointCloud2)

## Notes
- The mapping node saves PCD files in `~/3d_rover/maps/`
- RTAB-Map database is saved as `rtabmap.db` in the same folder
- For best results, run mapping_node directly (not via launch) for keyboard input

## Authors
- K12 Rover Project

## License
[Specify your license here]
