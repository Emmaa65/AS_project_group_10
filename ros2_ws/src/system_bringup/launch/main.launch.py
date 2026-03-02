#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch configurations
    load_params = LaunchConfiguration("load_params")
    corrupt_state_estimate = LaunchConfiguration("corrupt_state_estimate")
    enable_perception = LaunchConfiguration("enable_perception")
    enable_rviz = LaunchConfiguration("enable_rviz")
    enable_octomap = LaunchConfiguration("enable_octomap")
    enable_controller = LaunchConfiguration("enable_controller")
    enable_waypoints = LaunchConfiguration("enable_waypoints")
    trajectory_finish_topic = LaunchConfiguration("trajectory_finish_topic")

    save_octomap_on_shutdown = LaunchConfiguration("save_octomap_on_shutdown")
    octomap_save_path = LaunchConfiguration("octomap_save_path")
    octomap_autosave_interval_sec = LaunchConfiguration("octomap_autosave_interval_sec")
    enable_object = LaunchConfiguration("enable_object")
    
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
            "enable_octomap",
            default_value="true",
            description="Launch OctoMap server"
        ),
        DeclareLaunchArgument(
            "enable_controller",
            default_value="true",
            description="Launch controller node"
        ),
        DeclareLaunchArgument(
            "enable_waypoints",
            default_value="true",
            description="Launch waypoint mission nodes"
        ),
        DeclareLaunchArgument(
            "trajectory_finish_topic",
            default_value="/trajectory_finished",
            description="Bool topic that signals end of static trajectory"
        ),
        DeclareLaunchArgument(
            "save_octomap_on_shutdown",
            default_value="true",
            description="Save OctoMap to file when shutting down"
        ),
        DeclareLaunchArgument(
            "octomap_save_path",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("HOME"),
                "octomap.bt",
            ]),
            description="Path to save OctoMap file (.bt or .ot)"
        ),
        DeclareLaunchArgument(
            "octomap_autosave_interval_sec",
            default_value="10.0",
            description="Periodic OctoMap autosave interval in seconds (<=0 disables)"
        ),
        DeclareLaunchArgument(
            "enable_object",
            default_value="true",
            description="Launch object detection"
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
                FindPackageShare("occupancy_grid"), 
                "launch", 
                "perception_with_packages.launch.py"
            ])
        ),
        condition=IfCondition(enable_perception),
    )

    depth_to_pointcloud_node = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node",
        name="depth_to_pointcloud",
        output="screen",
        remappings=[
            ("image_rect", "/realsense/depth/image"),
            ("camera_info", "/realsense/depth/camera_info"),
            ("points", "/camera/pointcloud"),
        ],
        parameters=[
            {"queue_size": 10}
        ],
        condition=IfCondition(enable_perception),
    )

    octomap_server_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        parameters=[
            {"frame_id": "world"},
            {"resolution": 1.0},
            {"sensor_model.max_range": 50.0},
            {"save_on_shutdown": save_octomap_on_shutdown},
            {"save_map_path": octomap_save_path},
            {"autosave_interval_sec": octomap_autosave_interval_sec},
        ],
        remappings=[
            ("cloud_in", "/camera/pointcloud"),
        ],
        condition=IfCondition(enable_octomap),
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("controller_pkg"),
                "launch",
                "controller.launch.py"
            ])
        ),
        condition=IfCondition(enable_controller),
    )

    waypoint_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("basic_waypoint_pkg"),
                "launch",
                "waypoint_mission.launch.py"
            ])
        ),
        condition=IfCondition(enable_waypoints),
    )


    object_detection_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([
            FindPackageShare("object_detection"),
            "launch",
            "object_detection.launch.py"
        ])
    ),
    condition=IfCondition(enable_object),
    )

    wait_for_trajectory_finish = ExecuteProcess(
        cmd=[
            "/usr/bin/python3",
            PathJoinSubstitution([
                FindPackageShare("system_bringup"),
                "scripts",
                "wait_for_trajectory_finish.py",
            ]),
            "--topic",
            trajectory_finish_topic,
        ],
        output="screen",
    )

    start_cave_stack_on_trajectory_finish = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_trajectory_finish,
            on_exit=[
                depth_to_pointcloud_node,
                perception_launch,
                octomap_server_node,
                object_detection_launch,
            ],
        )
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
            "default_object.rviz"
        ])],
        condition=IfCondition(enable_rviz),
    )

    return LaunchDescription(
        declared_args
        + [
            simulation_launch,
            controller_launch,
            waypoint_launch,
            wait_for_trajectory_finish,
            start_cave_stack_on_trajectory_finish,
            rviz_node,
        ]
    )
