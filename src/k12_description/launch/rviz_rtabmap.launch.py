#!/usr/bin/env python3
"""
RViz Launch File for RTAB-Map Visualization

Launches RViz2 with a configuration tailored for RTAB-Map SLAM visualization,
showing the occupancy grid map, 3D point cloud, RGB/depth camera feeds,
robot model, TF tree, odometry, and map path.

Usage:
  Terminal 1: ros2 launch k12_description gazebo.launch.py
  Terminal 2: ros2 launch k12_description rtabmap.launch.py
  Terminal 3: ros2 launch k12_description rviz_rtabmap.launch.py
  Terminal 4: ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('k12_description')
    rviz_config = os.path.join(pkg_share, 'config', 'rtabmap.rviz')

    # Filter out snap library paths from LD_LIBRARY_PATH to avoid glibc conflicts
    ld_path = os.environ.get('LD_LIBRARY_PATH', '')
    filtered_path = ':'.join(p for p in ld_path.split(':') if '/snap/' not in p)

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        SetEnvironmentVariable('LD_LIBRARY_PATH', filtered_path),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
