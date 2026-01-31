#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch configurations
    load_params = LaunchConfiguration("load_params")
    corrupt_state_estimate = LaunchConfiguration("corrupt_state_estimate")
    enable_perception = LaunchConfiguration("enable_perception")
    enable_rviz = LaunchConfiguration("enable_rviz")
    
    right_image_topic = LaunchConfiguration("right_image_topic")
    right_info_topic = LaunchConfiguration("right_info_topic")
    left_image_topic = LaunchConfiguration("left_image_topic")
    left_info_topic = LaunchConfiguration("left_info_topic")
    depth_image_topic = LaunchConfiguration("depth_image_topic")
    depth_info_topic = LaunchConfiguration("depth_info_topic")

    # Declare launch arguments
    declared_args = [
        DeclareLaunchArgument(
            "load_params", 
            default_value="true",
            description="Load parameters from config files"
        ),
        DeclareLaunchArgument(
            "corrupt_state_estimate", 
            default_value="true",
            description="Enable state estimate corruption for realistic simulation"
        ),
        DeclareLaunchArgument(
            "enable_perception", 
            default_value="true",
            description="Enable perception pipeline (pointcloud and occupancy grid)"
        ),
        DeclareLaunchArgument(
            "enable_rviz", 
            default_value="true",
            description="Launch RViz for visualization"
        ),
        DeclareLaunchArgument(
            "right_image_topic", 
            default_value="/realsense/rgb/right_image_raw"
        ),
        DeclareLaunchArgument(
            "right_info_topic", 
            default_value="/realsense/rgb/right_image_info"
        ),
        DeclareLaunchArgument(
            "left_image_topic", 
            default_value="/realsense/rgb/left_image_raw"
        ),
        DeclareLaunchArgument(
            "left_info_topic", 
            default_value="/realsense/rgb/left_image_info"
        ),
        DeclareLaunchArgument(
            "depth_image_topic", 
            default_value="/realsense/depth/image"
        ),
        DeclareLaunchArgument(
            "depth_info_topic", 
            default_value="/realsense/depth/camera_info"
        ),
    ]

    # Include simulation launch file
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("simulation"), 
                "launch", 
                "simulation.launch.py"
            ])
        ),
        launch_arguments={
            "load_params": load_params,
            "corrupt_state_estimate": corrupt_state_estimate,
            "right_image_topic": right_image_topic,
            "right_info_topic": right_info_topic,
            "left_image_topic": left_image_topic,
            "left_info_topic": left_info_topic,
            "depth_image_topic": depth_image_topic,
            "depth_info_topic": depth_info_topic,
        }.items(),
    )

    # Include perception launch file (conditionally)
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("perception_pkg"), 
                "launch", 
                "perception_with_packages.launch.py"
            ])
        ),
        condition=IfCondition(enable_perception),
    )

    # RViz node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", PathJoinSubstitution([
            FindPackageShare("system_bringup"),
            "config",
            "default.rviz"
        ])],
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription(
        declared_args
        + [
            simulation_launch,
            perception_launch,
            rviz_node,
        ]
    )
