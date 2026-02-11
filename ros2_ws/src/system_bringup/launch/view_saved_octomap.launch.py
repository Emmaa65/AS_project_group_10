#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    octomap_path = LaunchConfiguration("octomap_path")
    frame_id = LaunchConfiguration("frame_id")
    enable_static_tf = LaunchConfiguration("enable_static_tf")
    enable_rviz = LaunchConfiguration("enable_rviz")

    declared_args = [
        DeclareLaunchArgument(
            "octomap_path",
            default_value="/home/madita/octomap.bt",
            description="Path to saved OctoMap (.bt or .ot)"
        ),
        DeclareLaunchArgument(
            "frame_id",
            default_value="map",
            description="Frame id to publish the map in"
        ),
        DeclareLaunchArgument(
            "enable_static_tf",
            default_value="true",
            description="Publish a static world->map TF"
        ),
        DeclareLaunchArgument(
            "enable_rviz",
            default_value="true",
            description="Launch RViz"
        ),
    ]

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_map_tf",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "map"],
        condition=IfCondition(enable_static_tf),
    )

    static_octomap_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        parameters=[
            {"octomap_path": octomap_path},
            {"frame_id": frame_id},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("system_bringup"),
            "config",
            "view_saved_octomap.rviz"
        ])],
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription(declared_args + [static_tf_node, static_octomap_node, rviz_node])
