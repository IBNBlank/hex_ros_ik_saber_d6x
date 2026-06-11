#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2024 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2024-12-16
################################################################

import xacro
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    # arg
    sim_time_flag = DeclareLaunchArgument(
        'sim_time_flag',
        default_value='false',
    )
    
    # visual
    urdf_file_path = FindPackageShare('hex_toolkit_saber_d6x').find(
        'hex_toolkit_saber_d6x') + '/urdf/saber_d6x.urdf'
    rviz_file_path = FindPackageShare('hex_toolkit_saber_d6x').find(
        'hex_toolkit_saber_d6x') + '/config/ros2/display.rviz'
    visual_group = GroupAction([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time':
                LaunchConfiguration('sim_time_flag'),
                'robot_description':
                xacro.process_file(urdf_file_path).toxml(),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('sim_time_flag'),
            }],
            arguments=['-d', rviz_file_path],
        ),
    ])

    # work trace
    urdf_file_path = FindPackageShare('hex_ros_ik_saber_d6x').find(
        'hex_ros_ik_saber_d6x') + '/urdf/saber_d6x.urdf'
    work_track_path = FindPackageShare('hex_ros_ik_saber_d6x').find(
        'hex_ros_ik_saber_d6x') + '/config/ros2/work_trace.yaml'
    work_trace_group = GroupAction([
        Node(
            package='hex_ros_ik_saber_d6x',
            executable='work_trace',
            name='work_trace',
            output='screen',
            emulate_tty=True,
            parameters=[
                {
                    'use_sim_time': LaunchConfiguration('sim_time_flag'),
                    'prog_debug': True,
                    'model_path': urdf_file_path,
                },
                work_track_path,
            ],
            remappings=[
                # subscribe
                ('/target_pose', '/target_pose'),
                # publish
                ('/joint_state', '/joint_state'),
                ('/debug_pose', '/debug_pose'),
                ('/ik_success', '/ik_success'),
            ]),
    ])

    return LaunchDescription([
        # arg
        sim_time_flag,
        # visual
        visual_group,
        # work trace
        work_trace_group,
    ])
