#
# Created on Wed Oct 23 2024
#
# Author: Joonchen Liau
# Email: liaojuncheng123@foxmail.com
#
# Copyright (c) 2024 Tiante Intelligent Technology
#

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time =  DeclareLaunchArgument(
        'use_sim_time', 
        default_value='false',
        description='Use simulation clock'
    )

    rviz_config = LaunchConfiguration('rviz_config', 
                                    default=PathJoinSubstitution([
                                        FindPackageShare('coroco_description'),
                                        'rviz',
                                        'coroco_display.rviz'
                                        ]))

    model_name = 'coroco.xacro'
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ",
        PathJoinSubstitution(
            [FindPackageShare("coroco_description"), "urdf", model_name]
        )
    ])

    robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_description':robot_description_content
            }]
    )

    joint_state_publisher = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            output='screen'
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
        use_sim_time,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])