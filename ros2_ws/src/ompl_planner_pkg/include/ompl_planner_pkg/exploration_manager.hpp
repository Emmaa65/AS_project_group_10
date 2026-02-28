/**
 * Exploration Manager Node
 * 
 * Coordinates the transition from waypoint-based navigation to autonomous
 * frontier-based exploration inside the cave.
 * 
 * This is the central orchestration node that:
 * 1. Monitors whether the drone has reached the cave entrance
 * 2. Switches from waypoint mode to exploration mode
 * 3. Selects the best frontier to explore
 * 4. Commands the RRT planner to generate paths to frontiers
 * 5. Publishes trajectory waypoints for mav_trajectory_generation
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <Eigen/Dense>
#include <memory>
#include <vector>

#include "ompl_planner_pkg/exploration_types.hpp"

class ExplorationManager : public rclcpp::Node {
public:
  ExplorationManager();
  ~ExplorationManager() = default;

private:
  // ========================================================================
  // ROS2 Subscribers/Publishers
  // ========================================================================
  
  // Odometry - current position and velocity of drone
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  
  // Frontier goal from frontier detector (replaces frontier_points_3d)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_frontier_goal_;
  
  // Planning result feedback
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_planning_result_;
  
  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_next_frontier_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_; // Publish current state
  
  // Control loop timer
  rclcpp::TimerBase::SharedPtr control_loop_timer_;
  
  // ========================================================================
  // State and Data
  // ========================================================================
  
  ExplorationState state_ = ExplorationState::INITIALIZATION;
  Eigen::Vector3d current_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_velocity_ = Eigen::Vector3d::Zero();
  
  // Cave entrance position (from waypoint mission)
  Eigen::Vector3d cave_entrance_ = Eigen::Vector3d(-330.0, 10.0, 20.0);
  double entrance_reach_tolerance_ = 1.5; // meters
  
  // Timing for state transitions
  rclcpp::Time state_transition_time_;
  
  // Frontier tracking (now from frontier_exploration)
  FrontierPoint selected_frontier_;
  Eigen::Vector3d last_sent_frontier_ = Eigen::Vector3d::Zero();
  int planning_failures_ = 0;
  const int MAX_PLANNING_FAILURES = 3;  // Reject frontier after 3 failures
  
  // Exploration parameters
  double frontier_update_rate_ = 2.0; // Hz
  double min_frontier_distance_ = 0.5; // Don't go to frontier closer than this
  double max_frontier_distance_ = 50.0; // Don't go to frontier further than this
  
  // Timing
  rclcpp::Time last_frontier_selection_;
  
  // ========================================================================
  // Callback Functions
  // ========================================================================
  
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void frontierGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void planningResultCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void controlLoop();
  
  // ========================================================================
  // State Machine Methods
  // ========================================================================
  
  void handleInitialization();
  void handleWaypointNavigation();
  void handleWaitingAtEntrance();
  void handleAutonomousExploration();
  void handleExplorationComplete();
  
  // ========================================================================
  // Frontier Selection and Planning
  // ========================================================================
  
  /**
   * Publish the selected frontier as target
   */
  void publishTargetFrontier(const FrontierPoint& frontier);
  
  // ========================================================================
  // Helper Methods
  // ========================================================================
  
  void publishDebugMarkers();
  bool hasReachedCaveEntrance();
  void transitionToState(ExplorationState new_state);
  
  // Logging
  void logState();
};
