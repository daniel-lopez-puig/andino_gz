#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node, PushRosNamespace
import xacro

from nav2_common.launch import ParseMultiRobotPose


def generate_launch_description():
    pkg_andino_gz = get_package_share_directory('andino_gz')
    pkg_andino_gz_demos = get_package_share_directory('andino_gz_demos')

    rviz_arg = DeclareLaunchArgument('rviz',
                                     default_value='true',
                                     description='Start RViz.')
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='depot.sdf',
        description='Name of the world to load.')

    world_name = LaunchConfiguration('world_name')
    world_path = PathJoinSubstitution([pkg_andino_gz, 'worlds', world_name])

    robots = """andino1={x: 0., y: 0., z: 0.1, yaw: 0.};
               andino2={x: 0., y: 1., z: 0.1, yaw: 0.};
              """

    group = GroupAction([
        ExecuteProcess(
            cmd=['ros2', 'launch', 'andino_gz', 'andino_gz.launch.py', 'ros_bridge:=true', 'rviz:=true', TextSubstitution(text='world_name:=') + TextSubstitution(world_path) ],
            output='screen',

        ),
    ])

    ld = LaunchDescription()
    ld.add_action(rviz_arg)
    ld.add_action(world_name_arg)
    ld.add_action(group)
    return ld
