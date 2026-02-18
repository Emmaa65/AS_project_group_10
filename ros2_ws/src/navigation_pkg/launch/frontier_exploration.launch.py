#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    octomap_topic = LaunchConfiguration('octomap_topic', default='/octomap_binary')
    odom_topic = LaunchConfiguration('odom_topic', default='current_state')
    frontier_goal_topic = LaunchConfiguration('frontier_goal_topic', default='/exploration/frontier_goal')
    node_name = LaunchConfiguration('node_name', default='frontier_exploration_node')

    declared_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time'),
        DeclareLaunchArgument('octomap_topic', default_value='/octomap_binary', description='OctoMap binary topic'),
        DeclareLaunchArgument('odom_topic', default_value='current_state', description='Odometry / state topic'),
        DeclareLaunchArgument('frontier_goal_topic', default_value='/exploration/frontier_goal', description='Published frontier goal topic'),
        DeclareLaunchArgument('node_name', default_value='frontier_exploration_node', description='Node name')
    ]

    node = Node(
        package='navigation_pkg',
        executable='frontier_exploration',
        name=node_name,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'octomap_topic': octomap_topic,
                     'odom_topic': odom_topic,
                     'frontier_goal_topic': frontier_goal_topic}],
        remappings=[
            ('/octomap_binary', octomap_topic),
            ('current_state', odom_topic),
            ('/exploration/frontier_goal', frontier_goal_topic),
        ]
    )

    return LaunchDescription(declared_args + [node])
