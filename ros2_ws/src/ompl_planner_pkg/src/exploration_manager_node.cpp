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
  this->declare_parameter("max_frontier_distance", 1000.0);  // Increased to allow deep cave exploration
  this->declare_parameter("cave_interior_margin_x", 1.0);
  this->declare_parameter("frontier_blacklist_radius", 15.0);
  this->declare_parameter("frontier_blacklist_timeout_s", 60.0);
  this->declare_parameter("frontier_stall_timeout_s", 20.0);
  this->declare_parameter("frontier_progress_epsilon_m", 0.3);
  this->declare_parameter("frontier_exhaustion_timeout_s", 8.0);
  this->declare_parameter("frontier_exhaustion_min_requests", 3);
  
  cave_entrance_[0] = this->get_parameter("cave_entrance_x").as_double();
  cave_entrance_[1] = this->get_parameter("cave_entrance_y").as_double();
  cave_entrance_[2] = this->get_parameter("cave_entrance_z").as_double();
  entrance_reach_tolerance_ = this->get_parameter("entrance_reach_tolerance").as_double();
  frontier_update_rate_ = this->get_parameter("frontier_update_rate").as_double();
  min_frontier_distance_ = this->get_parameter("min_frontier_distance").as_double();
  max_frontier_distance_ = this->get_parameter("max_frontier_distance").as_double();
  cave_interior_margin_x_ = this->get_parameter("cave_interior_margin_x").as_double();
  frontier_blacklist_radius_ = this->get_parameter("frontier_blacklist_radius").as_double();
  frontier_blacklist_timeout_s_ = this->get_parameter("frontier_blacklist_timeout_s").as_double();
  frontier_stall_timeout_s_ = this->get_parameter("frontier_stall_timeout_s").as_double();
  frontier_progress_epsilon_m_ = this->get_parameter("frontier_progress_epsilon_m").as_double();
  frontier_exhaustion_timeout_s_ = this->get_parameter("frontier_exhaustion_timeout_s").as_double();
  frontier_exhaustion_min_requests_ = this->get_parameter("frontier_exhaustion_min_requests").as_int();
  
  // Subscribers
  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/current_state_est", 10,
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
  
  pub_accepted_frontier_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "accepted_frontier_position", 10);
  
  pub_debug_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "exploration_markers", 10);  
  pub_state_ = this->create_publisher<std_msgs::msg::String>(
    "/exploration_state", 10);
  pub_frontier_request_ = this->create_publisher<std_msgs::msg::Bool>(
    "/exploration/request_frontier", 10);
  // Control loop timer (10Hz for state machine)
  control_loop_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&ExplorationManager::controlLoop, this));
  
  last_frontier_selection_ = this->get_clock()->now();
  last_frontier_request_time_ = this->get_clock()->now();
  active_frontier_last_progress_time_ = this->get_clock()->now();
  no_frontier_since_ = this->get_clock()->now();
  
  RCLCPP_INFO(this->get_logger(), 
    "Cave entrance set to: [%.2f, %.2f, %.2f]",
    cave_entrance_[0], cave_entrance_[1], cave_entrance_[2]);
  RCLCPP_INFO(this->get_logger(),
    "Frontier filters: min_dist=%.1f m, max_dist=%.1f m, blacklist_radius=%.1f m, cave_margin_x=%.1f m",
    min_frontier_distance_, max_frontier_distance_, frontier_blacklist_radius_, cave_interior_margin_x_);
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

  // Update frontier distance if frontier is active (distance >= 0; -1 means consumed/transitioning)
  if (selected_frontier_.distance >= 0.0) {
    selected_frontier_.distance = (selected_frontier_.position - current_position_).norm();
  }
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
  
  // Filter out frontiers that are too close (safety buffer to avoid wall collision)
  if (frontier.distance < min_frontier_distance_) {
    RCLCPP_DEBUG(this->get_logger(),
      "Frontier too close (%.2f m < min %.2f m) - rejecting",
      frontier.distance, min_frontier_distance_);
    return; // Ignore this frontier
  }

  // Reject frontiers beyond max distance (often stale/outside artifacts)
  if (frontier.distance > max_frontier_distance_) {
    RCLCPP_WARN(this->get_logger(),
      "Rejecting frontier [%.2f, %.2f, %.2f]: too far (%.2f m > max %.2f m)",
      frontier.position[0], frontier.position[1], frontier.position[2],
      frontier.distance, max_frontier_distance_);
    return;
  }

  // Keep frontiers strictly inside cave boundary with safety margin
  if (frontier.position[0] >= (cave_entrance_[0] - cave_interior_margin_x_)) {
    RCLCPP_WARN(this->get_logger(),
      "Rejecting frontier [%.2f, %.2f, %.2f] near/outside cave boundary (x=%.2f >= %.2f)",
      frontier.position[0], frontier.position[1], frontier.position[2],
      frontier.position[0], cave_entrance_[0] - cave_interior_margin_x_);
    return;
  }

  // Reject frontiers near recently failed ones to avoid infinite retry loops
  if (isFrontierRejected(frontier.position)) {
    RCLCPP_WARN(this->get_logger(),
      "Rejecting frontier [%.2f, %.2f, %.2f]: within blacklist radius %.2f m",
      frontier.position[0], frontier.position[1], frontier.position[2],
      frontier_blacklist_radius_);
    return;
  }
  
  // Only reset failure counter if this is a NEW frontier (position changed significantly)
  const double FRONTIER_CHANGE_THRESHOLD = 0.5; // meters
  double frontier_change = (frontier.position - selected_frontier_.position).norm();
  
  if (frontier_change > FRONTIER_CHANGE_THRESHOLD) {
    // This is a new frontier, reset the planning failure counter
    planning_failures_ = 0;
    RCLCPP_DEBUG(this->get_logger(),
      "New frontier detected (change: %.2f m), resetting failure counter",
      frontier_change);
    resetActiveFrontierTracking();
  }
  
  selected_frontier_ = frontier;
  frontier_request_pending_ = false;
  resetFrontierExhaustionTracking();
}

void ExplorationManager::planningResultCallback(const std_msgs::msg::Bool::SharedPtr msg) {
  if (msg->data) {
    // Planning succeeded - publish accepted frontier for continuity tracking
    RCLCPP_DEBUG(this->get_logger(), "Planning succeeded for frontier");
    planning_failures_ = 0;
    last_sent_frontier_ = selected_frontier_.position;
    
    // Publish accepted frontier position for frontier_exploration continuity scoring
    auto accepted_msg = geometry_msgs::msg::PoseStamped();
    accepted_msg.header.stamp = this->get_clock()->now();
    accepted_msg.header.frame_id = "world";
    accepted_msg.pose.position.x = selected_frontier_.position[0];
    accepted_msg.pose.position.y = selected_frontier_.position[1];
    accepted_msg.pose.position.z = selected_frontier_.position[2];
    accepted_msg.pose.orientation.w = 1.0;
    pub_accepted_frontier_->publish(accepted_msg);
    
    RCLCPP_INFO(this->get_logger(),
      "Published accepted frontier [%.2f, %.2f, %.2f] for continuity tracking",
      selected_frontier_.position[0], selected_frontier_.position[1], selected_frontier_.position[2]);
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
      rejectActiveFrontier("planning failed repeatedly");
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
  
  // Disabled: exploration_markers topic caused duplicate frontier visualization
  // publishDebugMarkers();
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
    planning_failures_ = 0;
    selected_frontier_.distance = -1.0;
    last_sent_frontier_ = Eigen::Vector3d::Zero();
    resetActiveFrontierTracking();
    resetFrontierExhaustionTracking();

    RCLCPP_INFO(this->get_logger(), 
      "Grid stabilized. Starting autonomous exploration (keeping %zu blacklisted frontiers)",
      rejected_frontiers_.size());
    transitionToState(ExplorationState::AUTONOMOUS_EXPLORATION);
    requestNewFrontier("entered autonomous exploration");
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
      "AUTONOMOUS: distance=%.2f, drone=[%.2f, %.2f, %.2f], frontier=[%.2f, %.2f, %.2f], failures=%d",
      selected_frontier_.distance,
      current_position_[0],
      current_position_[1],
      current_position_[2],
      selected_frontier_.position[0],
      selected_frontier_.position[1],
      selected_frontier_.position[2],
      planning_failures_);
  }

  const double FRONTIER_REACHED_THRESHOLD = 2.0; // meters

  // If we keep replanning/finishing without improving distance, abandon this frontier.
  if (selected_frontier_.distance > 0.1) {
    const rclcpp::Time now = this->get_clock()->now();

    if (std::isinf(active_frontier_min_distance_)) {
      active_frontier_min_distance_ = selected_frontier_.distance;
      active_frontier_last_progress_time_ = now;
    } else if ((active_frontier_min_distance_ - selected_frontier_.distance) >= frontier_progress_epsilon_m_) {
      active_frontier_min_distance_ = selected_frontier_.distance;
      active_frontier_last_progress_time_ = now;
    } else {
      const double stalled_for_s = (now - active_frontier_last_progress_time_).seconds();
      if (stalled_for_s >= frontier_stall_timeout_s_) {
        RCLCPP_WARN(this->get_logger(),
          "Rejecting stalled frontier [%.2f, %.2f, %.2f]: no progress >= %.2f m for %.1f s",
          selected_frontier_.position[0],
          selected_frontier_.position[1],
          selected_frontier_.position[2],
          frontier_progress_epsilon_m_,
          stalled_for_s);
        rejectActiveFrontier("frontier progress stalled");
        return;
      }
    }
  }

  // Frontier reached: distance <= threshold (including 0.0)
  if (selected_frontier_.distance >= 0.0 && selected_frontier_.distance <= FRONTIER_REACHED_THRESHOLD) {
    RCLCPP_INFO(this->get_logger(),
      "Frontier reached (distance=%.2f m), requesting next frontier",
      selected_frontier_.distance);
    selected_frontier_.distance = -1.0;  // Mark as consumed to avoid re-triggering
    planning_failures_ = 0;
    last_sent_frontier_ = Eigen::Vector3d::Zero();  // Reset frontier tracking to force republish
    resetActiveFrontierTracking();
    requestNewFrontier("frontier reached");
    return;
  }

  // If frontier is consumed (distance=-1.0) and we haven't received a response yet,
  // reset the pending flag after a timeout so we can request again aggressively.
  if (selected_frontier_.distance < -0.5) {  // Consumed frontier marker
    double elapsed_since_request = (this->get_clock()->now() - last_frontier_request_time_).seconds();
    const double REQUEST_TIMEOUT_S = 1.2;  // Keep retries responsive when no frontier can be produced
    
    if (frontier_request_pending_ && elapsed_since_request >= REQUEST_TIMEOUT_S) {
      RCLCPP_WARN(this->get_logger(),
        "Frontier request timeout (%.1f s without response), resetting and requesting again",
        elapsed_since_request);
      frontier_request_pending_ = false;
      requestNewFrontier("request timeout - retrying");
      return;
    } else if (!frontier_request_pending_ && elapsed_since_request < 2.0) {
      // Keep requesting frequently until we get a response
      requestNewFrontier("waiting for new frontier");
      return;
    }
  }
  
  if (selected_frontier_.distance > 0.1) {
    // Only publish when frontier target changed significantly.
    // This avoids re-triggering RRT planning on the same frontier at 10 Hz.
    const double TARGET_PUBLISH_THRESHOLD = 0.5; // meters
    const double target_change = (selected_frontier_.position - last_sent_frontier_).norm();
    const bool first_publish = last_sent_frontier_.norm() < 0.1;

    if (first_publish || target_change > TARGET_PUBLISH_THRESHOLD) {
      publishTargetFrontier(selected_frontier_);
      last_sent_frontier_ = selected_frontier_.position;
      RCLCPP_INFO(this->get_logger(),
        "Published new target frontier [%.2f, %.2f, %.2f] (change: %.2f m)",
        selected_frontier_.position[0],
        selected_frontier_.position[1],
        selected_frontier_.position[2],
        target_change);
    }
  } else {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "No frontier available (distance=%.2f)",
      selected_frontier_.distance);

    if (no_frontier_request_count_ == 0) {
      no_frontier_since_ = this->get_clock()->now();
    }

    const double no_frontier_elapsed_s = (this->get_clock()->now() - no_frontier_since_).seconds();
    if (no_frontier_elapsed_s >= frontier_exhaustion_timeout_s_ &&
        no_frontier_request_count_ >= frontier_exhaustion_min_requests_) {
      RCLCPP_WARN(this->get_logger(),
        "No valid frontier for %.1f s after %d requests. Marking exploration complete.",
        no_frontier_elapsed_s,
        no_frontier_request_count_);
      transitionToState(ExplorationState::EXPLORATION_COMPLETE);
      return;
    }

    const double request_cooldown_s = 0.5;  // Reduced from 1.0 to be more responsive
    double elapsed_since_request = (this->get_clock()->now() - last_frontier_request_time_).seconds();
    if (!frontier_request_pending_ && elapsed_since_request >= request_cooldown_s) {
      requestNewFrontier("no active frontier");
    }
  }
}

void ExplorationManager::handleExplorationComplete() {
  frontier_request_pending_ = false;

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

void ExplorationManager::requestNewFrontier(const std::string& reason) {
  if (state_ != ExplorationState::AUTONOMOUS_EXPLORATION) {
    return;
  }

  if (!pub_frontier_request_) {
    return;
  }

  auto msg = std::make_unique<std_msgs::msg::Bool>();
  msg->data = true;
  pub_frontier_request_->publish(std::move(msg));
  frontier_request_pending_ = true;
  last_frontier_request_time_ = this->get_clock()->now();
  no_frontier_request_count_++;

  RCLCPP_INFO(this->get_logger(),
    "Requested new frontier (%s)",
    reason.c_str());
}

bool ExplorationManager::isFrontierRejected(const Eigen::Vector3d& frontier) const {
  rclcpp::Time now = this->get_clock()->now();
  for (const auto& [rejected_pos, rejection_time] : rejected_frontiers_) {
    // Check if blacklist entry has expired
    double age_s = (now - rejection_time).seconds();
    if (age_s > frontier_blacklist_timeout_s_) {
      continue; // This blacklist entry has expired, skip it
    }
    
    // Check if new frontier is within blacklist radius of rejected position
    if ((frontier - rejected_pos).norm() <= frontier_blacklist_radius_) {
      return true;
    }
  }
  return false;
}

void ExplorationManager::rejectActiveFrontier(const std::string& reason) {
  if (selected_frontier_.distance > 0.0) {
    rejected_frontiers_.push_back({selected_frontier_.position, this->get_clock()->now()});
    RCLCPP_WARN(this->get_logger(),
      "Blacklisting frontier [%.2f, %.2f, %.2f] for %.1f s (%s)",
      selected_frontier_.position[0],
      selected_frontier_.position[1],
      selected_frontier_.position[2],
      frontier_blacklist_timeout_s_,
      reason.c_str());
  }

  selected_frontier_.distance = 0.0;
  planning_failures_ = 0;
  last_sent_frontier_ = Eigen::Vector3d::Zero();
  resetActiveFrontierTracking();
  requestNewFrontier(reason);
}

void ExplorationManager::resetActiveFrontierTracking() {
  active_frontier_min_distance_ = std::numeric_limits<double>::infinity();
  active_frontier_last_progress_time_ = this->get_clock()->now();
}

void ExplorationManager::resetFrontierExhaustionTracking() {
  no_frontier_since_ = this->get_clock()->now();
  no_frontier_request_count_ = 0;
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
    marker.color.g = 0.0f;
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
