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
    pub_tf = LaunchConfiguration('pub_tf', default='true')
    base_frame = LaunchConfiguration('base_frame', default='map')
    odom_frame = LaunchConfiguration('odom_frame', default='odom')
    dev_path = LaunchConfiguration('dev_path', default='/dev/ttyUSB0')
    # dev_type is one of the '1 -> usbttlcan' '2 -> canable' '3 -> origin'
    dev_type = LaunchConfiguration('dev_type', default=0)
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
        ]),
        launch_arguments={
            'pub_tf': pub_tf,
            'base_frame': base_frame,
            'odom_frame': odom_frame,
            'dev_path': dev_path,
            'dev_type': dev_type
        }.items()
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