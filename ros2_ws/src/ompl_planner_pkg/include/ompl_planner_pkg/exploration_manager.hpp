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
#include <string>
#include <limits>
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
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_accepted_frontier_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_debug_markers_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_; // Publish current state
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_frontier_request_; // Request next frontier on demand
  
  // Control loop timer
  rclcpp::TimerBase::SharedPtr control_loop_timer_;
  
  // ========================================================================
  // State and Data
  // ========================================================================
  
  ExplorationState state_ = ExplorationState::INITIALIZATION;
  Eigen::Vector3d current_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_velocity_ = Eigen::Vector3d::Zero();
  
  // Cave entrance position (from waypoint mission)
  Eigen::Vector3d cave_entrance_ = Eigen::Vector3d(-175.0, 10.0, 12.0);
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
  double cave_interior_margin_x_ = 1.0; // Require frontier to be this far inside cave (x < cave_entrance_x - margin)
  double frontier_blacklist_radius_ = 2.0; // Reject new frontiers near recently failed ones
  double frontier_blacklist_timeout_s_ = 60.0; // Expire blacklist entries after this time
  double frontier_stall_timeout_s_ = 20.0; // Reject active frontier if no progress for this long
  double frontier_progress_epsilon_m_ = 0.3; // Minimum progress to reset stall timer
  double frontier_exhaustion_timeout_s_ = 8.0; // Consider exploration complete if no valid frontier for this long
  int frontier_exhaustion_min_requests_ = 3; // Minimum request attempts before completion
  std::vector<std::pair<Eigen::Vector3d, rclcpp::Time>> rejected_frontiers_; // [position, timestamp]
  double active_frontier_min_distance_ = std::numeric_limits<double>::infinity();
  rclcpp::Time active_frontier_last_progress_time_;
  rclcpp::Time no_frontier_since_;
  int no_frontier_request_count_ = 0;
  
  // Timing
  rclcpp::Time last_frontier_selection_;
  rclcpp::Time last_frontier_request_time_;
  bool frontier_request_pending_ = false;
  
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
  void requestNewFrontier(const std::string& reason);
  bool isFrontierRejected(const Eigen::Vector3d& frontier) const;
  void rejectActiveFrontier(const std::string& reason);
  void resetActiveFrontierTracking();
  void resetFrontierExhaustionTracking();
  
  // ========================================================================
  // Helper Methods
  // ========================================================================
  
  void publishDebugMarkers();
  bool hasReachedCaveEntrance();
  void transitionToState(ExplorationState new_state);
  
  // Logging
  void logState();
};
