import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # RTAB-Map parameters for 3D LiDAR mapping
    rtabmap_parameters = {
        'use_sim_time': use_sim_time,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan_cloud': True,
        'approx_sync': True,
        'frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'queue_size': 200,
        
        # RTAB-Map database
        'database_path': '~/.ros/rtabmap.db',
        
        # 3D LiDAR settings
        'RGBD/NeighborLinkRefining': 'True',
        'RGBD/ProximityBySpace': 'True',
        'RGBD/AngularUpdate': '0.01',
        'RGBD/LinearUpdate': '0.01',
        'RGBD/OptimizeFromGraphEnd': 'False',
        'Reg/Strategy': '1',  # 1=ICP
        'Reg/Force3DoF': 'True',  # 2D constraint for ground robot
        'Grid/FromDepth': 'False',
        'Grid/RayTracing': 'True',
        'Grid/3D': 'True',
        'Grid/MaxGroundHeight': '0.1',
        'Grid/MaxObstacleHeight': '2.0',
        'Grid/CellSize': '0.1',
        
        # ICP settings
        'Icp/VoxelSize': '0.1',
        'Icp/MaxCorrespondenceDistance': '1.0',
        'Icp/PointToPlane': 'True',
        'Icp/Iterations': '30',
        
        # Loop closure
        'RGBD/LoopClosureReextractFeatures': 'False',
        'Mem/IncrementalMemory': 'True',
        'Mem/InitWMWithAllNodes': 'True',
    }

    return LaunchDescription([
        # Fix GTK path for snap conflicts
        SetEnvironmentVariable('GTK_PATH', ''),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Static transform: map -> odom (RTAB-Map will update this)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_init',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        # Odom to base_link from odometry topic
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_init',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        
        # RTAB-Map SLAM node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[rtabmap_parameters],
            remappings=[
                ('scan_cloud', '/velodyne_points'),
                ('odom', '/odom'),
            ],
            arguments=['-d'],  # Delete database on start
        ),
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
