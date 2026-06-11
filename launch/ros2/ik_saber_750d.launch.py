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
    urdf_file_path = FindPackageShare('hex_ros_ik_saber_750d').find(
        'hex_ros_ik_saber_750d') + '/urdf/saber_750d.urdf'
    rviz_file_path = FindPackageShare('hex_ros_ik_saber_750d').find(
        'hex_ros_ik_saber_750d') + '/config/ros2/display.rviz'
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

    # ik
    urdf_file_path = FindPackageShare('hex_ros_ik_saber_750d').find(
        'hex_ros_ik_saber_750d') + '/urdf/saber_750d.urdf'
    params_path = FindPackageShare('hex_ros_ik_saber_750d').find(
        'hex_ros_ik_saber_750d') + '/config/ros2/params.yaml'
    ik_node = Node(
        package='hex_ros_ik_saber_750d',
        executable='ik_saber_750d',
        name='ik_saber_750d',
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': LaunchConfiguration('sim_time_flag'),
                'prog_debug': True,
                'model_path': urdf_file_path,
            },
            params_path,
        ],
        remappings=[
            # subscribe
            ('/target_pose', '/target_pose'),
            # publish
            ('/joint_states', '/joint_states'),
            ('/debug_pose', '/debug_pose'),
            ('/ik_success', '/ik_success'),
        ])

    return LaunchDescription([
        # arg
        sim_time_flag,
        # visual
        visual_group,
        # ik
        ik_node,
    ])
