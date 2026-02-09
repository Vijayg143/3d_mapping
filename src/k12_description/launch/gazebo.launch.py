from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
import os
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation time if true'
    )
    
    # Get package directory for the URDF
    share_dir = get_package_share_directory('k12_description')
    world_file = os.path.join(share_dir, 'worlds', 'my_world.world')
    
    
    xacro_file = os.path.join(share_dir, 'urdf', 'k12.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
        	'robot_description': robot_description,
        	'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Note: joint_state_publisher removed - Gazebo publishes joint states

    # Static Transform: map -> odom
    static_tf_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # NOTE: odom -> base_link is published by Gazebo diff_drive plugin
    # DO NOT add a static transform here - it will conflict and cause flickering!

    # Include Gazebo server and client
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file,     #include custom world
            'pause': 'false'
        }.items()                    # Auto-start simulation
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    # Spawn the robot in Gazebo (z=0.25 to prevent ground clipping and physics jitter)
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'k12', '-topic', 'robot_description', '-z', '0.25'],
        output='screen'
    )

    # Return LaunchDescription
    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        static_tf_map_to_odom,  # map->odom static transform
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
    ])

