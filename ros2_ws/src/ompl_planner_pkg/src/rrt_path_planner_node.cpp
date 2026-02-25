/**
 * RRT Path Planner Node
 * 
 * Plans 3D paths from current position to frontier targets using RRT*.
 * 
 * Phase 1: Simple implementation that:
 * - Subscribes to target frontier points from exploration manager
 * - Filters out invalid frontiers (too low, not forward-facing)
 * - Plans a path using RRT* (or simple straight line for quick demo)
 * - Publishes resulting trajectory for mav_trajectory_generation
 */

#include "ompl_planner_pkg/rrt_path_planner.hpp"

RRTPathPlanner::RRTPathPlanner() 
  : rclcpp::Node("rrt_path_planner") {
  
  RCLCPP_INFO(this->get_logger(), "Initializing RRT Path Planner...");
  
  // Load parameters
  this->declare_parameter("max_planning_time", 1.0);
  this->declare_parameter("step_size", 3.0);  // Larger steps = fewer waypoints
  this->declare_parameter("max_iterations", 1000);
  this->declare_parameter("max_velocity", 10.0);  // Increased from 0.5
  this->declare_parameter("max_acceleration", 2.0);  // Increased from 0.3
  this->declare_parameter("replan_threshold", 20.0);  // Meters
  this->declare_parameter("cave_entrance_x", -330.0);
  this->declare_parameter("cave_entrance_y", 10.0);
  this->declare_parameter("cave_entrance_z", 20.0);
  this->declare_parameter("min_frontier_z", -13.5); // Keep above cave floor
  
  max_planning_time_ = this->get_parameter("max_planning_time").as_double();
  step_size_ = this->get_parameter("step_size").as_double();
  max_iterations_ = this->get_parameter("max_iterations").as_int();
  max_velocity_ = this->get_parameter("max_velocity").as_double();
  max_acceleration_ = this->get_parameter("max_acceleration").as_double();
  replan_threshold_ = this->get_parameter("replan_threshold").as_double();
  cave_entrance_[0] = this->get_parameter("cave_entrance_x").as_double();
  cave_entrance_[1] = this->get_parameter("cave_entrance_y").as_double();
  cave_entrance_[2] = this->get_parameter("cave_entrance_z").as_double();
  min_frontier_z_ = this->get_parameter("min_frontier_z").as_double();
  
  // Subscribers
  sub_target_frontier_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "target_frontier", 10,
    std::bind(&RRTPathPlanner::targetFrontierCallback, this, std::placeholders::_1));
  
  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "current_state_est", 10,
    std::bind(&RRTPathPlanner::odometryCallback, this, std::placeholders::_1));
  
  sub_occupancy_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "octomap_point_cloud_centers", 10,
    std::bind(&RRTPathPlanner::occupancyCallback, this, std::placeholders::_1));
  
  // State monitoring (only plan when in AUTONOMOUS_EXPLORATION state)
  sub_state_ = this->create_subscription<std_msgs::msg::String>(
    "/exploration_state", 10,
    std::bind(&RRTPathPlanner::stateCallback, this, std::placeholders::_1));
  
  // Publishers
  pub_trajectory_ = this->create_publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>(
    "trajectory", 10);
  
  pub_plan_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "planned_path_markers", 10);
  
  // Planning timer (2Hz planning rate)
  planning_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&RRTPathPlanner::planPath, this));
  
  RCLCPP_INFO(this->get_logger(),
    "RRT Path Planner initialized (step_size: %.2f m, max_v: %.2f m/s, max_a: %.2f m/s²)",
    step_size_, max_velocity_, max_acceleration_);
}

void RRTPathPlanner::targetFrontierCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  Eigen::Vector3d frontier;
  frontier << msg->point.x, msg->point.y, msg->point.z;
  
  // Validate frontier before accepting
  if (!isFrontierValid(frontier)) {
    RCLCPP_WARN(this->get_logger(),
      "Rejecting frontier [%.2f, %.2f, %.2f] (Z=%.2f, cave_x_min=%.2f)",
      frontier[0], frontier[1], frontier[2], frontier[2], cave_entrance_[0]);
    return;
  }
  
  target_frontier_ = frontier;
  has_target_ = true;
  
  RCLCPP_DEBUG(this->get_logger(),
    "New valid target frontier: [%.2f, %.2f, %.2f]",
    target_frontier_[0], target_frontier_[1], target_frontier_[2]);
}

void RRTPathPlanner::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_position_ << 
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z;
}

void RRTPathPlanner::occupancyCallback(const sensor_msgs::msg::PointCloud2::SharedPtr /*msg*/) {
  // TODO: Store occupancy grid for collision checking
  // For Phase 1, we ignore collision checking (assumes exploration is safe)
}

void RRTPathPlanner::stateCallback(const std_msgs::msg::String::SharedPtr msg) {
  exploration_state_ = msg->data;
  
  RCLCPP_DEBUG(this->get_logger(),
    "Exploration state changed to: %s", exploration_state_.c_str());
}

void RRTPathPlanner::planPath() {
  // Only plan during autonomous exploration
  if (exploration_state_ != "AUTONOMOUS_EXPLORATION") {
    return;
  }
  
  // Only plan if we have a target
  if (!has_target_) {
    return;
  }
  
  // Check if we need to replan:
  // 1. Target has changed significantly, OR
  // 2. Drone is close to last planned target (goal reached)
  double target_change = (target_frontier_ - last_planned_target_).norm();
  double distance_to_goal = (current_position_ - last_planned_target_).norm();
  
  bool target_changed = target_change >= replan_threshold_;
  bool goal_reached = (distance_to_goal < 5.0 && last_planned_target_.norm() > 0.1);
  bool first_plan = last_planned_target_.norm() < 0.1;
  
  if (!target_changed && !goal_reached && !first_plan) {
    RCLCPP_DEBUG(this->get_logger(),
      "No replan needed: target_change=%.2f < %.2f, dist_to_goal=%.2f",
      target_change, replan_threshold_, distance_to_goal);
    return;
  }
  
  if (goal_reached) {
    RCLCPP_INFO(this->get_logger(),
      "Goal reached (dist=%.2f m), planning to next frontier", distance_to_goal);
  }
  
  RCLCPP_DEBUG(this->get_logger(),
    "Planning path from [%.2f, %.2f, %.2f] to [%.2f, %.2f, %.2f]",
    current_position_[0], current_position_[1], current_position_[2],
    target_frontier_[0], target_frontier_[1], target_frontier_[2]);
  
  // Plan path to target
  auto path = planPathToTarget(current_position_, target_frontier_);
  
  if (!path.empty()) {
    // Publish trajectory
    publishTrajectory(path);
    publishPlanMarkers(path);
    
    // Remember what we planned
    last_planned_target_ = target_frontier_;
    
    RCLCPP_INFO(this->get_logger(),
      "Published trajectory with %zu waypoints to [%.1f, %.1f, %.1f]",
      path.size(), target_frontier_[0], target_frontier_[1], target_frontier_[2]);
  }
}

std::vector<Eigen::Vector3d> RRTPathPlanner::planPathToTarget(
  const Eigen::Vector3d& start,
  const Eigen::Vector3d& goal) {
  
  // Phase 1: Simple straight-line path
  // TODO: Replace with actual RRT* implementation
  
  return generateSimplePath(start, goal, step_size_);
}

std::vector<Eigen::Vector3d> RRTPathPlanner::generateSimplePath(
  const Eigen::Vector3d& start,
  const Eigen::Vector3d& goal,
  double step_size) {
  
  std::vector<Eigen::Vector3d> path;
  
  // For straight-line path: only use start and goal (no intermediate waypoints)
  // This creates a simple direct trajectory that is fast and efficient
  path.push_back(start);
  path.push_back(goal);
  
  return path;
}

void RRTPathPlanner::publishTrajectory(const std::vector<Eigen::Vector3d>& path) {
  if (path.size() < 2) {
    RCLCPP_WARN(this->get_logger(), "Path too short to create trajectory");
    return;
  }
  
  try {
    // Create trajectory using mav_trajectory_generation
    const int dimension = 3;
    const int derivative_to_optimize = mav_trajectory_generation::derivative_order::SNAP;
    
    mav_trajectory_generation::Vertex::Vector vertices;
    
    // Start vertex
    mav_trajectory_generation::Vertex start(dimension);
    start.makeStartOrEnd(current_position_, derivative_to_optimize);
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                        Eigen::Vector3d::Zero());
    vertices.push_back(start);
    
    // For straight-line paths: NO intermediate waypoints
    // Direct path from start to goal is fastest and most efficient
    
    // End vertex (goal = last path point)
    mav_trajectory_generation::Vertex end(dimension);
    end.makeStartOrEnd(path.back(), derivative_to_optimize);
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                      Eigen::Vector3d::Zero());
    vertices.push_back(end);
    
    // Estimate segment times based on distances and velocities
    std::vector<double> segment_times;
    segment_times = mav_trajectory_generation::estimateSegmentTimes(
      vertices, max_velocity_, max_acceleration_);
    
    // Create trajectory using linear optimization (Phase 1)
    // For Phase 1, linear optimization is sufficient for straight-line paths
    mav_trajectory_generation::NonlinearOptimizationParameters nlp_params;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<10> planning_problem(
      dimension, nlp_params);
    
    if (!planning_problem.setupFromVertices(vertices, segment_times, 
        mav_trajectory_generation::PolynomialOptimization<10>::kHighestDerivativeToOptimize)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to setup trajectory vertices");
      return;
    }
    
    // Solve using linear optimization for Phase 1
    if (!planning_problem.solveLinear()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to solve linear trajectory optimization");
      return;
    }
    
    mav_trajectory_generation::Trajectory trajectory;
    planning_problem.getTrajectory(&trajectory);
    
    // Convert to ROS message
    mav_planning_msgs::msg::PolynomialTrajectory4D msg;
    mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory, &msg);
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "world";
    
    pub_trajectory_->publish(msg);
    
    RCLCPP_DEBUG(this->get_logger(), 
      "Published trajectory with %zu segments",
      msg.segments.size());
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(),
      "Failed to create trajectory: %s", e.what());
  }
}

void RRTPathPlanner::publishPlanMarkers(const std::vector<Eigen::Vector3d>& path) {
  auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();
  
  // Publish line strip for planned path
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "planned_path";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    for (const auto& point : path) {
      geometry_msgs::msg::Point p;
      p.x = point[0];
      p.y = point[1];
      p.z = point[2];
      marker.points.push_back(p);
    }
    
    marker.scale.x = 0.1; // Line width
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.8f;
    
    markers->markers.push_back(marker);
  }
  
  // Publish waypoint spheres
  for (size_t i = 0; i < path.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "path_waypoints";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = path[i][0];
    marker.pose.position.y = path[i][1];
    marker.pose.position.z = path[i][2];
    marker.scale.x = marker.scale.y = marker.scale.z = 0.2;
    marker.color.r = 0.0f;
    marker.color.g = 0.5f;
    marker.color.b = 1.0f;
    marker.color.a = 0.6f;
    markers->markers.push_back(marker);
  }
  
  if (!markers->markers.empty()) {
    pub_plan_markers_->publish(std::move(markers));
  }
}

// ============================================================================
// Frontier Validation
// ============================================================================

bool RRTPathPlanner::isFrontierValid(const Eigen::Vector3d& frontier) {
  // Check 1: Frontier must be above minimum safe height (cave floor)
  if (frontier[2] < min_frontier_z_) {
    RCLCPP_DEBUG(this->get_logger(),
      "Rejecting frontier [%.2f, %.2f, %.2f] - Z=%.2f < min_z=%.2f (too close to cave floor)",
      frontier[0], frontier[1], frontier[2], frontier[2], min_frontier_z_);
    return false;
  }
  
  // Check 2: Cave X-coordinates are always < -330 (cave entrance X)
  // Reject frontiers outside the cave (X >= -330)
  if (frontier[0] >= cave_entrance_[0]) {
    RCLCPP_DEBUG(this->get_logger(),
      "Rejecting frontier [%.2f, %.2f, %.2f] - outside cave (X=%.2f >= cave_entrance_x=%.2f)",
      frontier[0], frontier[1], frontier[2], frontier[0], cave_entrance_[0]);
    return false;
  }
  
  // All checks passed - accept the frontier
  return true;
}

// ============================================================================
// Main Entry Point
// ============================================================================

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RRTPathPlanner>());
  rclcpp::shutdown();
  return 0;
}
