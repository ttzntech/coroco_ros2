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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments with their default values
    pub_tf = LaunchConfiguration('pub_tf', default='true')
    base_frame = LaunchConfiguration('base_frame', default='map')
    odom_frame = LaunchConfiguration('odom_frame', default='odom')
    dev_path = LaunchConfiguration('dev_path', default='/dev/ttyUSB0')
    # dev_type is one of the 'usbttlcan' 'canable' 'origin'
    dev_type = LaunchConfiguration('dev_type', default='usbttlcan')

    return LaunchDescription([
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
                'dev_path': dev_path,
                'dev_type': dev_type,
            }]
        ),
    ])