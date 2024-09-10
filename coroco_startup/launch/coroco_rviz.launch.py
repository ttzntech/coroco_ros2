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
    # Declare launch arguments
    rviz_config = LaunchConfiguration('rviz_config', 
                                      default=PathJoinSubstitution([
                                          FindPackageShare('coroco_startup'),
                                          'rviz',
                                          'default.rviz'
                                      ]))

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

    # Define the RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
        output='screen'
    )

    return LaunchDescription([
        coroco_chassis_launch,
        rviz_node
    ])