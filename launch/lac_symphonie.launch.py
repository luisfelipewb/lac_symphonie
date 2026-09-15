# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""Launch the Lac Symphonie world with the vehicles of a spawn config and the geo TF node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('lac_symphonie')

    vrx_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('vrx_gz'), 'launch', 'competition.launch.py')),
        launch_arguments={
            'world': 'lac_symphonie',
            'config_file': LaunchConfiguration('config_file'),
            'headless': LaunchConfiguration('headless'),
            'paused': LaunchConfiguration('paused'),
            'extra_gz_args': LaunchConfiguration('extra_gz_args'),
        }.items())

    geo_tf = Node(
        package='lac_symphonie',
        executable='geo_tf_publisher',
        output='screen',
        parameters=[{
            'world_sdf': os.path.join(pkg, 'worlds', 'lac_symphonie.sdf'),
            'points_file': LaunchConfiguration('points_file'),
            'use_sim_time': True,
        }])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=os.path.join(pkg, 'config', 'lac_symphonie_spawn.yaml'),
                              description='vrx_gz spawn config (list of models with positions)'),
        DeclareLaunchArgument('points_file', default_value=os.path.join(pkg, 'config', 'geo_points.yaml'),
                              description='YAML of lat/lon points published as TF frames'),
        DeclareLaunchArgument('headless', default_value='False',
                              description='True to run gz sim without the GUI'),
        DeclareLaunchArgument('paused', default_value='False',
                              description='True to start the simulation paused'),
        DeclareLaunchArgument('extra_gz_args', default_value='',
                              description='Additional arguments passed to gz sim'),
        vrx_launch,
        geo_tf,
    ])
