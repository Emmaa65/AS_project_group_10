#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_path_planner = LaunchConfiguration('enable_path_planner')

    declared_args = [
        DeclareLaunchArgument('enable_path_planner', default_value='true', description='Enable path planner')
    ]

    planner_node = Node(
        package='navigation_pkg',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[
            {'planning_time': 1.0},
            {'min_x': -500.0}, {'max_x': 500.0},
            {'min_y': -100.0}, {'max_y': 100.0},
        ],
    )

    return LaunchDescription(declared_args + [planner_node])
