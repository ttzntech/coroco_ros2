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
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Include the display_coroco launch file
    display_coroco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('coroco_description'),
                'launch',
                'display_coroco.launch.py'
            ])
        ])
    )

    # Include the cody_chassis launch file
    coroco_chassis_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('coroco_chassis'),
                'launch',
                'coroco_chassis.launch.py'
            ])
        ])
    )

    return LaunchDescription([
        coroco_chassis_launch,
        display_coroco_launch
    ])