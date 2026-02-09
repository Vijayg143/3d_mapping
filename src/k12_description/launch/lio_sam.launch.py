import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    lio_sam_share = get_package_share_directory('lio_sam')
    
    # Paths
    lio_sam_params = os.path.join(lio_sam_share, 'config', 'params.yaml')
    rviz_config = os.path.join(lio_sam_share, 'config', 'rviz2.rviz')
    
    # Use simulation time (set to true when using with Gazebo)
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        # Unset GTK_PATH to avoid snap library conflict with RViz
        SetEnvironmentVariable('GTK_PATH', ''),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # LIO-SAM nodes
        Node(
            package='lio_sam',
            executable='lio_sam_imuPreintegration',
            name='lio_sam_imuPreintegration',
            parameters=[lio_sam_params, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_imageProjection',
            name='lio_sam_imageProjection',
            parameters=[lio_sam_params, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_featureExtraction',
            name='lio_sam_featureExtraction',
            parameters=[lio_sam_params, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lio_sam',
            executable='lio_sam_mapOptimization',
            name='lio_sam_mapOptimization',
            parameters=[lio_sam_params, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])
