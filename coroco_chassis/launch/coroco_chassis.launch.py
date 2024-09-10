#
# Created on Wed Aug 21 2024
#
# Author: Joonchen Liau
# Email: liaojuncheng123@foxmail.com
#
# Copyright (c) 2024 Tiante Intelligent Technology
#

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # dev_type is one of the '0 -> usbttlcan' '1 -> canable' '2 -> origin'
    dev_type_arg = DeclareLaunchArgument(
        'dev_type',
        default_value='0',
        description='Device type: 0 for usbttlcan, 1 for canable, 2 for origin'
    )
    dev_path_arg = DeclareLaunchArgument(
        'dev_path',
        default_value='/dev/ttyUSB0',
        description='Path to the device (e.g., /dev/ttyUSB0)'
    )
    ctrl_mode_arg = DeclareLaunchArgument(
        'ctrl_mode',
        default_value='1',
        description='Control mode: 0 for remote, 1 for CAN'
    )
    
    # Declare launch arguments with their default values
    pub_tf = LaunchConfiguration('pub_tf', default='true')
    base_frame = LaunchConfiguration('base_frame', default='map')
    odom_frame = LaunchConfiguration('odom_frame', default='odom')
    # dev_type is one of the '0 -> usbttlcan' '1 -> canable' '2 -> origin'
    dev_type = LaunchConfiguration('dev_type', default=0)
    dev_path = LaunchConfiguration('dev_path', default='/dev/ttyUSB0')
    ctrl_mode = LaunchConfiguration('ctrl_mode', default=1)

    return LaunchDescription([
        dev_type_arg,
        dev_path_arg,
        ctrl_mode_arg,
        # Define the Node with parameters
        Node(
            package='coroco_chassis',
            executable='coroco_chassis_node',
            name='coroco_chassis_node',
            namespace='coroco',
            output='screen',
            parameters=[{
                'pub_tf': pub_tf,
                'base_frame': base_frame,
                'odom_frame': odom_frame,
                'dev_type': dev_type,
                'dev_path': dev_path,
                'ctrl_mode': ctrl_mode
            }]
        )
    ])