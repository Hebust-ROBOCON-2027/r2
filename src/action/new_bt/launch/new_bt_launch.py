#!/usr/bin/env python3
# Copyright 2026 RoboCon Team
# Licensed under the Apache License, Version 2.0

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('new_bt')
    default_xml = os.path.join(pkg_dir, 'behavior_trees', 'nav_to_fixed_point.xml')

    bt_xml = LaunchConfiguration('bt_xml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'bt_xml',
            default_value=default_xml,
            description='要执行的行为树 XML 文件绝对路径或相对路径',
        ),
        Node(
            package='new_bt',
            executable='new_bt_runner',
            name='new_bt_runner',
            output='screen',
            arguments=[bt_xml],
            parameters=[{'use_sim_time': False}],
        ),
    ])
