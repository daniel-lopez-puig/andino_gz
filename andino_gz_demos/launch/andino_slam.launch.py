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

    rviz_arg = DeclareLaunchArgument('rviz', default_value='true', description='Start RViz.')
    world_name_arg = DeclareLaunchArgument('world_name', default_value='depot.sdf', description='Name of the world to load.')
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("andino_slam"),
                                   'config', 'slam_toolbox_online_async.yaml'),
        description='Full path to the ROS 2 parameters file to use for the slam_toolbox node')

    world_name = LaunchConfiguration('world_name')
    world_path = PathJoinSubstitution([pkg_andino_gz, 'worlds', world_name])

    group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_andino_gz, 'launch', 'andino_gz.launch.py')
            ),
            launch_arguments={
                'robots': 'andino={x: 0., y: 0., z: 0.1, yaw: 0.};',
                'world_name': world_path,
                'ros_bridge': 'true',
                'rviz': 'false',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_andino_gz_demos, 'launch', 'include', 'slam_toolbox_online_async.launch.py')
            ),
            launch_arguments={
                'slam_params_file_arg': LaunchConfiguration('slam_params_file')
            }.items(),
        ),
    ])
      # RViz
    rviz_node = Node(
          package='rviz2',
          executable='rviz2',
          arguments=['-d', os.path.join(pkg_andino_gz_demos, 'rviz', 'andino_slam.rviz')],
          parameters=[{'use_sim_time': True}],
          remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
          ],
          condition=IfCondition(LaunchConfiguration('rviz')),
    )

    ld = LaunchDescription()
    ld.add_action(rviz_arg)
    ld.add_action(world_name_arg)
    ld.add_action(slam_params_file_arg)
    ld.add_action(rviz_node)
    ld.add_action(group)
    return ld
