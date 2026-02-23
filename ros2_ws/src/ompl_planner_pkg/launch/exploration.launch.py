from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch file for autonomous 3D cave exploration system.
    
    This launches two main nodes:
    1. exploration_manager: Orchestrates the state machine (waypoint → exploration)
    2. rrt_path_planner: Plans paths to frontier targets using RRT*
    
    NOTE: Frontier detection is handled by navigation_pkg/frontier_exploration_node
    which publishes frontier_points_3d from the OctoMap.
    
    These work together with existing nodes:
    - basic_waypoint_pkg/basic_waypoint_node: Initial waypoint navigation
    - navigation_pkg/frontier_exploration: 3D frontier detection from OctoMap
    - trajectory_executor: Converts PolynomialTrajectory to sampled states
    - controller: Executes the desired trajectory
    """
    
    # Path to exploration config
    exploration_config = PathJoinSubstitution([
        FindPackageShare("ompl_planner_pkg"),
        "config",
        "exploration_params.yaml"
    ])
    
    # ========================================================================
    # Exploration Manager Node (State Machine Orchestrator)
    # ========================================================================
    exploration_manager_node = Node(
        package="ompl_planner_pkg",
        executable="exploration_manager_node",
        name="exploration_manager",
        output="screen",
        parameters=[exploration_config],
        remappings=[
            # Subscribe to drone odometry (same as basic_waypoint_node)
            ("current_state_est", "current_state_est"),
            # Subscribe to frontiers from frontier_detector
            ("frontier_points_3d", "frontier_points_3d"),
            # Publish selected target frontier
            ("target_frontier", "target_frontier"),
            # Debug markers for RViz
            ("exploration_markers", "exploration_markers"),
            # Note: exploration_manager does NOT publish trajectory directly
            # That responsibility goes to rrt_path_planner
        ]
    )
    
    # ========================================================================
    # RRT Path Planner Node
    # ========================================================================
    rrt_path_planner_node = Node(
        package="ompl_planner_pkg",
        executable="rrt_path_planner_node",
        name="rrt_path_planner",
        output="screen",
        parameters=[exploration_config],
        remappings=[
            # Subscribe to target frontier from exploration_manager
            ("target_frontier", "target_frontier"),
            # Subscribe to drone odometry
            ("current_state_est", "current_state_est"),
            # Subscribe to occupancy for collision checking
            ("octomap_point_cloud_centers", "octomap_point_cloud_centers"),
            # Publish trajectory to executor
            ("trajectory", "trajectory"),
            # Debug markers for RViz
            ("planned_path_markers", "planned_path_markers"),
        ]
    )
    
    return LaunchDescription([
        frontier_detector_node,
        exploration_manager_node,
        rrt_path_planner_node,
    ])
