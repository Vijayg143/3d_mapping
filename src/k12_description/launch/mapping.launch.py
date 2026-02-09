#!/usr/bin/env python3
"""
3D Point Cloud Mapping with Keyboard Controls

Controls (in this terminal):
  s + Enter  ->  SAVE map to PCD file
  l + Enter  ->  LOAD last saved map
  e + Enter  ->  END/PAUSE mapping (stop recording)
  r + Enter  ->  RESUME mapping (start recording)
  c + Enter  ->  CLEAR current map
  q + Enter  ->  QUIT

Usage:
  Terminal 1: ros2 launch k12_description gazebo.launch.py
  Terminal 2: ros2 launch k12_description mapping.launch.py
  Terminal 3: ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os
import sys
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('k12_description')
    
    # Get path to the mapping node script - go from share/k12_description to lib/k12_description
    pkg_prefix = os.path.dirname(os.path.dirname(pkg_share))  # go up twice from share/k12_description
    mapping_script = os.path.join(pkg_prefix, 'lib', 'k12_description', 'mapping_node')
    
    return LaunchDescription([
        ExecuteProcess(
            cmd=[sys.executable, mapping_script],
            name='mapping_node',
            output='screen',
            prefix='stdbuf -o L',  # Line-buffered output
            additional_env={'PYTHONUNBUFFERED': '1'},
            emulate_tty=True,  # Enable TTY for stdin passthrough
            stdin=None,  # Allow stdin from terminal
        ),
    ])
