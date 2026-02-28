#include "ompl_planner_pkg/exploration_manager.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/msg/marker.hpp>

ExplorationManager::ExplorationManager() 
  : rclcpp::Node("exploration_manager") {
  
  RCLCPP_INFO(this->get_logger(), "Initializing Exploration Manager...");
  
  // Load parameters
  this->declare_parameter("cave_entrance_x", -330.0);
  this->declare_parameter("cave_entrance_y", 10.0);
  this->declare_parameter("cave_entrance_z", 20.0);
  this->declare_parameter("entrance_reach_tolerance", 1.5);
  this->declare_parameter("frontier_update_rate", 2.0);
  this->declare_parameter("min_frontier_distance", 0.5);
  this->declare_parameter("max_frontier_distance", 50.0);
  
  cave_entrance_[0] = this->get_parameter("cave_entrance_x").as_double();
  cave_entrance_[1] = this->get_parameter("cave_entrance_y").as_double();
  cave_entrance_[2] = this->get_parameter("cave_entrance_z").as_double();
  entrance_reach_tolerance_ = this->get_parameter("entrance_reach_tolerance").as_double();
  frontier_update_rate_ = this->get_parameter("frontier_update_rate").as_double();
  min_frontier_distance_ = this->get_parameter("min_frontier_distance").as_double();
  max_frontier_distance_ = this->get_parameter("max_frontier_distance").as_double();
  
  // Subscribers
  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "current_state_est", 10,
    std::bind(&ExplorationManager::odometryCallback, this, std::placeholders::_1));
  
  sub_frontier_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/exploration/frontier_goal", 10,
    std::bind(&ExplorationManager::frontierGoalCallback, this, std::placeholders::_1));
  
  // Subscribe to planning results for failure handling
  sub_planning_result_ = this->create_subscription<std_msgs::msg::Bool>(
    "planning_result", 10,
    std::bind(&ExplorationManager::planningResultCallback, this, std::placeholders::_1));
  
  // Publishers
  pub_next_frontier_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "target_frontier", 10);
  
  pub_debug_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "exploration_markers", 10);  
  pub_state_ = this->create_publisher<std_msgs::msg::String>(
    "/exploration_state", 10);  
  // Control loop timer (10Hz for state machine)
  control_loop_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ExplorationManager::controlLoop, this));
  
  last_frontier_selection_ = this->get_clock()->now();
  
  RCLCPP_INFO(this->get_logger(), 
    "Cave entrance set to: [%.2f, %.2f, %.2f]",
    cave_entrance_[0], cave_entrance_[1], cave_entrance_[2]);
}

// ============================================================================
// Callbacks
// ============================================================================

void ExplorationManager::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_position_ = odometryToPosition(*msg);
  current_velocity_ << 
    msg->twist.twist.linear.x,
    msg->twist.twist.linear.y,
    msg->twist.twist.linear.z;
}

void ExplorationManager::frontierGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // frontier_exploration publishes the best frontier as a PoseStamped
  // We just need to forward this to RRT planner for path generation
  FrontierPoint frontier;
  frontier.position = Eigen::Vector3d(
    msg->pose.position.x,
    msg->pose.position.y,
    msg->pose.position.z
  );
  frontier.distance = (frontier.position - current_position_).norm();
  frontier.info_gain = 1.0; // frontier_exploration already selected the best one
  
  selected_frontier_ = frontier;
  
  // Reset planning failures counter when receiving a new frontier from frontier_exploration
  planning_failures_ = 0;
}

void ExplorationManager::planningResultCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    // Planning succeeded
    RCLCPP_DEBUG(this->get_logger(), "Planning succeeded for frontier");
    planning_failures_ = 0;
    last_sent_frontier_ = selected_frontier_.position;
  } else {
    // Planning failed
    planning_failures_++;
    RCLCPP_WARN(this->get_logger(),
      "Planning failed for frontier [%.2f, %.2f, %.2f] (failure count: %d/%d)",
      selected_frontier_.position[0],
      selected_frontier_.position[1],
      selected_frontier_.position[2],
      planning_failures_,
      MAX_PLANNING_FAILURES);
    
    // If too many failures on same frontier, clear it and wait for next one
    if (planning_failures_ >= MAX_PLANNING_FAILURES) {
      RCLCPP_ERROR(this->get_logger(),
        "Frontier unreachable after %d attempts - rejecting and waiting for next",
        MAX_PLANNING_FAILURES);
      selected_frontier_.distance = 0.0;  // Clear frontier
      planning_failures_ = 0;
    }
  }
}

// ============================================================================
// Control Loop - State Machine
// ============================================================================

void ExplorationManager::controlLoop() {
  // State machine
  switch (state_) {
    case ExplorationState::INITIALIZATION:
      handleInitialization();
      break;
    case ExplorationState::WAYPOINT_NAVIGATION:
      handleWaypointNavigation();
      break;
    case ExplorationState::WAITING_AT_ENTRANCE:
      handleWaitingAtEntrance();
      break;
    case ExplorationState::AUTONOMOUS_EXPLORATION:
      handleAutonomousExploration();
      break;
    case ExplorationState::EXPLORATION_COMPLETE:
      handleExplorationComplete();
      break;
    case ExplorationState::ERROR_STATE:
      RCLCPP_ERROR(this->get_logger(), "ERROR STATE - Check logs");
      break;
  }
  
  publishDebugMarkers();
}

// ============================================================================
// State Handlers
// ============================================================================

void ExplorationManager::handleInitialization() {
  // Wait for first odometry message
  if ((current_position_ - Eigen::Vector3d::Zero()).norm() < 0.1) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Waiting for first odometry message...");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), 
    "Odometry received. Current position: [%.2f, %.2f, %.2f]",
    current_position_[0], current_position_[1], current_position_[2]);
  
  transitionToState(ExplorationState::WAYPOINT_NAVIGATION);
}

void ExplorationManager::handleWaypointNavigation() {
  // Monitor progress towards cave entrance
  double dist_to_entrance = (current_position_ - cave_entrance_).norm();
  
  static int nav_log_count = 0;
  if (++nav_log_count % 10 == 0) {  // Log every 10 calls (1 second at 10Hz)
    RCLCPP_INFO(this->get_logger(),
      "WAYPOINT_NAV: current=[%.2f, %.2f, %.2f], dist_to_entrance=%.2f m (tolerance=%.2f m)",
      current_position_[0], current_position_[1], current_position_[2],
      dist_to_entrance, entrance_reach_tolerance_);
  }
  
  if (dist_to_entrance < entrance_reach_tolerance_) {
    RCLCPP_INFO(this->get_logger(), 
      "Cave entrance reached! Distance: %.2f m, transitioning to WAITING_AT_ENTRANCE",
      dist_to_entrance);
    transitionToState(ExplorationState::WAITING_AT_ENTRANCE);
  }
}

void ExplorationManager::handleWaitingAtEntrance() {
  // Brief pause at cave entrance before starting exploration (5 seconds)
  // This gives time for occupancy grid to stabilize
  
  // Initialize timer on first call to this state
  if (state_transition_time_.nanoseconds() == 0) {
    state_transition_time_ = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), 
      "At cave entrance. Waiting 5 seconds for grid stabilization...");
    return;
  }
  
  // Check if 5 seconds have elapsed
  auto elapsed = (this->get_clock()->now() - state_transition_time_).seconds();
  if (elapsed >= 5.0) {
    RCLCPP_INFO(this->get_logger(), 
      "Grid stabilized. Starting autonomous exploration...");
    transitionToState(ExplorationState::AUTONOMOUS_EXPLORATION);
  } else {
    RCLCPP_DEBUG(this->get_logger(),
      "Waiting at entrance: %.1f / 5.0 seconds elapsed", elapsed);
  }
}

void ExplorationManager::handleAutonomousExploration() {
  // Main exploration loop
  // frontier_exploration already selects best frontiers and publishes to /exploration/frontier_goal
  // We just forward it to RRT planner for path generation
  
  static int log_count = 0;
  if (++log_count % 10 == 0) {  // Log every 10 calls (1 second at 10Hz)
    RCLCPP_INFO(this->get_logger(),
      "AUTONOMOUS: distance=%.2f, pos=[%.2f, %.2f, %.2f], failures=%d",
      selected_frontier_.distance,
      selected_frontier_.position[0],
      selected_frontier_.position[1],
      selected_frontier_.position[2],
      planning_failures_);
  }
  
  if (selected_frontier_.distance > 0.1) {
    // We have a valid frontier from frontier_exploration
    publishTargetFrontier(selected_frontier_);
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "No frontier available (distance=%.2f)",
      selected_frontier_.distance);
  }
}

void ExplorationManager::handleExplorationComplete() {
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 10000,
    "Exploration complete - no more frontiers to explore");
}

// ============================================================================
// Frontier Selection
// ============================================================================

void ExplorationManager::publishTargetFrontier(const FrontierPoint& frontier) {
  auto msg = std::make_unique<geometry_msgs::msg::PointStamped>();
  msg->header.stamp = this->get_clock()->now();
  msg->header.frame_id = "world";
  msg->point.x = frontier.position[0];
  msg->point.y = frontier.position[1];
  msg->point.z = frontier.position[2];
  
  pub_next_frontier_->publish(std::move(msg));
}

// ============================================================================
// Helper Methods
// ============================================================================

void ExplorationManager::transitionToState(ExplorationState new_state) {
  if (new_state == state_) return;
  
  state_ = new_state;
  state_transition_time_ = rclcpp::Time(0); // Reset timer for new state
  logState();
  
  // Publish state for other nodes (e.g., rrt_path_planner)
  const char* state_names[] = {
    "INITIALIZATION",
    "WAYPOINT_NAVIGATION",
    "WAITING_AT_ENTRANCE",
    "AUTONOMOUS_EXPLORATION",
    "EXPLORATION_COMPLETE",
    "ERROR_STATE"
  };
  
  auto state_msg = std::make_unique<std_msgs::msg::String>();
  state_msg->data = state_names[static_cast<int>(state_)];
  pub_state_->publish(std::move(state_msg));
}

void ExplorationManager::logState() {
  const char* state_names[] = {
    "INITIALIZATION",
    "WAYPOINT_NAVIGATION",
    "WAITING_AT_ENTRANCE",
    "AUTONOMOUS_EXPLORATION",
    "EXPLORATION_COMPLETE",
    "ERROR_STATE"
  };
  
  RCLCPP_WARN(this->get_logger(),
    "=== State Transition: %s ===",
    state_names[static_cast<int>(state_)]);
}

void ExplorationManager::publishDebugMarkers() {
  auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
  
  // Cave entrance marker
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "cave_entrance";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = cave_entrance_[0];
    marker.pose.position.y = cave_entrance_[1];
    marker.pose.position.z = cave_entrance_[2];
    marker.scale.x = marker.scale.y = marker.scale.z = 2.0;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.7f;
    markers->markers.push_back(marker);
  }
  
  // Current position marker
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "current_position";
    marker.id = 1;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = current_position_[0];
    marker.pose.position.y = current_position_[1];
    marker.pose.position.z = current_position_[2];
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.8f;
    markers->markers.push_back(marker);
  }
  
  // Selected frontier marker
  if (selected_frontier_.distance > 0.1) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "selected_frontier";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = selected_frontier_.position[0];
    marker.pose.position.y = selected_frontier_.position[1];
    marker.pose.position.z = selected_frontier_.position[2];
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.7f;
    markers->markers.push_back(marker);
  }
  
  if (!markers->markers.empty()) {
    pub_debug_markers_->publish(std::move(markers));
  }
}

// ============================================================================
// Main Entry Point
// ============================================================================

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ExplorationManager>());
  rclcpp::shutdown();
  return 0;
}
