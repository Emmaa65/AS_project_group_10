#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
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
    enable_frontier = LaunchConfiguration("enable_frontier")
    enable_pointcloud_filter = LaunchConfiguration("enable_pointcloud_filter")

    ompl_params_file = PathJoinSubstitution([
        FindPackageShare("ompl_planner_pkg"),
        "config",
        "exploration_params.yaml",
    ])

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
            "enable_frontier",
            default_value="true",
            description="Launch frontier exploration node from navigation_pkg"
        ),
        DeclareLaunchArgument(
            "enable_pointcloud_filter",
            default_value="false",
            description="Enable point cloud outlier filter (true=filtered, false=raw cloud)"
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
            ("points", "/camera/pointcloud_raw"),  # Changed to _raw
        ],
        parameters=[
            {"queue_size": 10}
        ],
        condition=IfCondition(enable_perception),
    )

    # Statistical outlier removal filter to remove isolated points (e.g., through wall gaps)
    pointcloud_filter_node = Node(
        package="perception_pkg",
        executable="pointcloud_outlier_filter",
        name="pointcloud_outlier_filter",
        output="screen",
        remappings=[
            ("cloud_in", "/camera/pointcloud_raw"),
            ("cloud_out", "/camera/pointcloud"),
        ],
        parameters=[
            {"voxel_leaf_size": 0.05},    # 5cm voxels (balanced: speed + detail)
            {"mean_k": 10},               # Reduced: 20 neighbors → 10 (faster)
            {"stddev_mul_thresh": 1.0},   # Relaxed: 1.5 → 1.0 (faster filtering)
        ],
        condition=IfCondition(enable_pointcloud_filter),
    )

    # OctoMap server subscribes to appropriate cloud topic based on filter setting
    # If filter enabled: subscribe to /camera/pointcloud (filtered output)
    # If filter disabled: subscribe to /camera/pointcloud_raw (raw depth)
    octomap_server_node = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        parameters=[
            {"frame_id": "world"},
            {"resolution": 2.0},
            {"sensor_model.max_range": 75.0},
            {"save_on_shutdown": save_octomap_on_shutdown},
            {"save_map_path": octomap_save_path},
            {"autosave_interval_sec": octomap_autosave_interval_sec},
        ],
        remappings=[
            ("cloud_in", "/camera/pointcloud"),
        ],
        condition=IfCondition(enable_pointcloud_filter),
    )
    
    # Alternative OctoMap node for when filter is disabled (subscribes to raw cloud)
    octomap_server_node_raw = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        parameters=[
            {"frame_id": "world"},
            {"resolution": 2.0},
            {"sensor_model.max_range": 75.0},
            {"save_on_shutdown": save_octomap_on_shutdown},
            {"save_map_path": octomap_save_path},
            {"autosave_interval_sec": octomap_autosave_interval_sec},
        ],
        remappings=[
            ("cloud_in", "/camera/pointcloud_raw"),
        ],
        condition=UnlessCondition(enable_pointcloud_filter),
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

    # RViz node (defined here before event handler references it)
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

    start_cave_stack_on_trajectory_finish = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_trajectory_finish,
            on_exit=[
                depth_to_pointcloud_node,
                perception_launch,
                octomap_server_node,
                octomap_server_node_raw,
                object_detection_launch,
                rviz_node,
                Node(
                    package="navigation_pkg",
                    executable="frontier_exploration",
                    name="frontier_exploration",
                    output="screen",
                    condition=IfCondition(enable_frontier),
                ),
                # Exploration manager and RRT path planner
                Node(
                    package="ompl_planner_pkg",
                    executable="exploration_manager_node",
                    name="exploration_manager",
                    parameters=[ompl_params_file],
                    output="screen",
                    condition=IfCondition(enable_frontier),
                ),
                Node(
                    package="ompl_planner_pkg",
                    executable="rrt_path_planner_node",
                    name="rrt_path_planner",
                    parameters=[ompl_params_file],
                    output="screen",
                    condition=IfCondition(enable_frontier),
                ),
            ],
        )
    )

    return LaunchDescription(
        declared_args
        + [
            simulation_launch,
            controller_launch,
            waypoint_launch,
            wait_for_trajectory_finish,
            start_cave_stack_on_trajectory_finish,
            pointcloud_filter_node,
        ]
    )
