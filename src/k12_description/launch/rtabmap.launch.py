#!/usr/bin/env python3
"""
RTAB-Map SLAM Launch File for K12 Rover

This launches RTAB-Map with RGB-D camera for visual SLAM.

Usage:
  Terminal 1: ros2 launch k12_description gazebo.launch.py
  Terminal 2: ros2 launch k12_description rtabmap.launch.py
  Terminal 3: ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # RTAB-Map parameters - optimized for Gazebo simulation
    rtabmap_parameters = {
        'use_sim_time': use_sim_time,
        'subscribe_depth': True,
        'subscribe_rgb': True,
        'subscribe_scan_cloud': False,  # Disable lidar, use RGB-D only
        'approx_sync': True,
        'sync_queue_size': 30,
        'topic_queue_size': 30,
        'qos': 2,  # Best effort QoS for simulation
        'wait_for_transform': 0.5,  # Increased wait time for TF
        
        # Frame IDs
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        
        # Visual odometry - disabled, use wheel odom
        'visual_odometry': False,
        'odom_tf_linear_variance': 0.001,
        'odom_tf_angular_variance': 0.001,
        
        # Disable IMU (causes timing issues in simulation)
        'subscribe_odom_info': False,
        
        # RTAB-Map SLAM parameters
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.1',  # Less frequent updates
        'RGBD/LinearUpdate': '0.1',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'Reg/Strategy': '0',  # 0=Visual
        'Reg/Force3DoF': 'true',
        'Grid/FromDepth': 'true',
        'Grid/MaxGroundHeight': '0.1',
        'Grid/MaxObstacleHeight': '2.0',
        'Grid/RayTracing': 'true',
        'Grid/3D': 'true',
        'GridGlobal/MinSize': '20.0',
        
        # Database
        'database_path': os.path.expanduser('~/3d_rover/maps/rtabmap.db'),
        'Rtabmap/DetectionRate': '1',
    }

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # RTAB-Map SLAM Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_parameters],
            remappings=[
                ('rgb/image', '/camera/depth_camera/image_raw'),
                ('rgb/camera_info', '/camera/depth_camera/camera_info'),
                ('depth/image', '/camera/depth_camera/depth/image_raw'),
                ('odom', '/odom'),
            ],
            arguments=['--delete_db_on_start'],
        ),
        
        # RTAB-Map Visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan_cloud': False,
                'approx_sync': True,
                'sync_queue_size': 30,
                'topic_queue_size': 30,
                'qos': 2,
                'frame_id': 'base_link',
                'wait_for_transform': 0.5,
            }],
            remappings=[
                ('rgb/image', '/camera/depth_camera/image_raw'),
                ('rgb/camera_info', '/camera/depth_camera/camera_info'),
                ('depth/image', '/camera/depth_camera/depth/image_raw'),
                ('odom', '/odom'),
            ],
        ),
    ])
