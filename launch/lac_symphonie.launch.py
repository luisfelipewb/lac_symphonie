# Copyright 2025 Luis Batista
#
# Licensed under the BSD License; see the package.xml for details.

"""Launch the Lac Symphonie world with the vehicles of a spawn config and the geo TF node."""

import os
import re
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.conditions import evaluate_condition_expression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# USVWind element <- launch arg
WIND_ARGS = {
    'wind_mean_velocity': 'wind_speed',
    'wind_direction': 'wind_direction',
    'var_wind_gain_constants': 'wind_gust',
}

# Block written by scripts/gen_trees.py
TREES_BLOCK = re.compile(r'(<!-- BEGIN generated trees -->).*?(<!-- END generated trees -->)', re.S)


def world_with_args(context, world_sdf):
    """Write a copy of the world with the wind and trees args and put it first on GZ_SIM_RESOURCE_PATH."""
    with open(world_sdf) as f:
        sdf = f.read()
    for tag, arg in WIND_ARGS.items():
        value = LaunchConfiguration(arg).perform(context)
        float(value)
        sdf, n = re.subn(rf'<{tag}>[^<]*</{tag}>', f'<{tag}>{value}</{tag}>', sdf)
        if n != 1:
            raise RuntimeError(f'expected one <{tag}> in {world_sdf}, found {n}')
    if not evaluate_condition_expression(context, [LaunchConfiguration('trees')]):
        sdf, n = TREES_BLOCK.subn(r'\1\n    \2', sdf)
        if n != 1:
            raise RuntimeError(f'expected one generated trees block in {world_sdf}, found {n}')
    out_dir = tempfile.mkdtemp(prefix='lac_symphonie_')
    with open(os.path.join(out_dir, os.path.basename(world_sdf)), 'w') as f:
        f.write(sdf)
    return [AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', out_dir, prepend=True)]


def generate_launch_description():
    pkg = get_package_share_directory('lac_symphonie')
    world_sdf = os.path.join(pkg, 'worlds', 'lac_symphonie.sdf')

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
            'world_sdf': world_sdf,
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
        DeclareLaunchArgument('wind_speed', default_value='0.0',
                              description='Mean wind speed, m/s'),
        DeclareLaunchArgument('wind_direction', default_value='240',
                              description='Where the wind goes, degrees counter-clockwise from east'),
        DeclareLaunchArgument('wind_gust', default_value='0.0',
                              description='Gust gain, about the standard deviation of the speed, m/s'),
        DeclareLaunchArgument('trees', default_value='True',
                              description='False to drop the generated trees for faster runs'),
        OpaqueFunction(function=world_with_args, args=[world_sdf]),
        vrx_launch,
        geo_tf,
    ])
