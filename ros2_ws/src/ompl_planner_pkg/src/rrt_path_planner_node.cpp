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
#include "ompl_planner_pkg/collision_checker.hpp"

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
  this->declare_parameter("min_frontier_z", -33.5); // Keep above cave floor
  
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
  sub_frontier_goals_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
    "/exploration/frontier_goals", 10,
    std::bind(&RRTPathPlanner::frontierGoalsCallback, this, std::placeholders::_1));
  
  sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "current_state_est", 10,
    std::bind(&RRTPathPlanner::odometryCallback, this, std::placeholders::_1));
  
  sub_occupancy_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "octomap_point_cloud_centers", 10,
    std::bind(&RRTPathPlanner::occupancyCallback, this, std::placeholders::_1));

  sub_octomap_ = this->create_subscription<octomap_msgs::msg::Octomap>(
    "octomap_full", 10,
    std::bind(&RRTPathPlanner::octomapCallback, this, std::placeholders::_1));
  
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

  // Initialize collision checker
  collision_checker_ = std::make_shared<FCLCollisionChecker>(0.25);
  
  RCLCPP_INFO(this->get_logger(),
    "RRT Path Planner initialized (step_size: %.2f m, max_v: %.2f m/s, max_a: %.2f m/s²)",
    step_size_, max_velocity_, max_acceleration_);
}

void RRTPathPlanner::frontierGoalsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
  if (!msg || msg->poses.empty()) {
    RCLCPP_DEBUG(this->get_logger(), "Received empty frontier goals list");
    return;
  }
  
  // Buffer the incoming goals instead of directly replacing active goals
  // This prevents interrupting an ongoing planning cycle
  {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    pending_frontier_goals_ = msg->poses;
  }
  
  RCLCPP_DEBUG(this->get_logger(),
    "Received %zu frontier goal(s) - buffering for next planning cycle",
    msg->poses.size());
}

void RRTPathPlanner::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  current_position_ << 
    msg->pose.pose.position.x,
    msg->pose.pose.position.y,
    msg->pose.pose.position.z;
}

void RRTPathPlanner::occupancyCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // Convert incoming pointcloud to collision objects for FCL
  if (collision_checker_ && msg) {
    collision_checker_->setPointCloud(msg);
  }
}

void RRTPathPlanner::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg){
  // Convert OctoMap message to OcTree
  auto tree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
  if (!tree) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "Failed to convert octomap message to OcTree");
    return;
  }
  
  // Update octree (thread-safe)
  std::lock_guard<std::mutex> lock(octree_mutex_);
  octree_.reset(tree);
  
  RCLCPP_DEBUG(this->get_logger(),
    "OctoMap updated: resolution=%.3f, tree_depth=%d",
    octree_->getResolution(), octree_->getTreeDepth());
}

void RRTPathPlanner::stateCallback(const std_msgs::msg::String::SharedPtr msg) {
  exploration_state_ = msg->data;
  
  RCLCPP_DEBUG(this->get_logger(),
    "Exploration state changed to: %s", exploration_state_.c_str());
}

void RRTPathPlanner::planPath() {
  // Mark planning as in progress (simple flag, not a scoped guard for clarity)
  is_planning_ = true;
  
  // At the start of each planning cycle, check if new goals are pending
  // and swap them in only when not actively checking goals
  {
    std::lock_guard<std::mutex> lock(goals_mutex_);
    if (!pending_frontier_goals_.empty()) {
      if (in_loop==false) {
      frontier_goals_ = pending_frontier_goals_;
      pending_frontier_goals_.clear();
      current_goal_index_ = 0;
      has_targets_ = true;
      
      RCLCPP_INFO(this->get_logger(),
        "Swapped in pending goals: now checking %zu frontier goals",
        frontier_goals_.size());}
    }
  }
  
  // Only plan during autonomous exploration
  if (exploration_state_ != "AUTONOMOUS_EXPLORATION") {
    RCLCPP_DEBUG(this->get_logger(), "Not in AUTONOMOUS_EXPLORATION mode, skipping planning");
    is_planning_ = false;
    return;
  }
  
  // Only plan if we have targets
  if (!has_targets_ || frontier_goals_.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
      "No frontier goals available for planning");
    is_planning_ = false;
    return;
  }
  
  // Get first goal (best goal)
  Eigen::Vector3d first_goal(
    frontier_goals_[0].position.x,
    frontier_goals_[0].position.y,
    frontier_goals_[0].position.z
  );
  
  // Check if we need to replan:
  // 1. Target has changed significantly, OR
  // 2. Drone is close to last planned target (goal reached)
  double target_change = (first_goal - last_planned_target_).norm();
  double distance_to_goal = (current_position_ - last_planned_target_).norm();
  
  bool target_changed = target_change >= replan_threshold_;
  bool goal_reached = (distance_to_goal < 1.0 && last_planned_target_.norm() > 0.1);
  bool first_plan = last_planned_target_.norm() < 0.1;
  
  if (!target_changed && !goal_reached && !first_plan) {
    RCLCPP_DEBUG(this->get_logger(),
      "No replan needed: target_change=%.2f < %.2f, dist_to_goal=%.2f",
      target_change, replan_threshold_, distance_to_goal);
    is_planning_ = false;
    return;
  }
  
  if (goal_reached) {
    RCLCPP_INFO(this->get_logger(),
      "Goal reached (dist=%.2f m), planning to next frontier", distance_to_goal);
  } else if (target_changed) {
    RCLCPP_INFO(this->get_logger(),
      "Goal list updated, checking all %zu goals for collision-free path",
      frontier_goals_.size());
  }
  
  // Try each goal in order until we find a collision-free path
  bool found_path = false;
  for (size_t i = 0; i < frontier_goals_.size(); ++i) {
    Eigen::Vector3d goal(
      frontier_goals_[i].position.x,
      frontier_goals_[i].position.y,
      frontier_goals_[i].position.z
    );
    
    RCLCPP_INFO(this->get_logger(),
      "RRT*: Attempting goal #%zu/%zu [%.2f, %.2f, %.2f]",
      i + 1, frontier_goals_.size(), goal[0], goal[1], goal[2]);
    
    // Try to plan path to this goal
    auto path = planPathToTarget(current_position_, goal, true);
    
    if (!path.empty()) {
      // Successfully found a collision-free path
      publishTrajectory(path);
      publishPlanMarkers(path);
      last_planned_target_ = goal;
      current_goal_index_ = i;
      found_path = true;
      
      RCLCPP_INFO(this->get_logger(),
        "RRT*: SUCCESS: Published trajectory to goal #%zu/%zu with %zu waypoints [%.1f, %.1f, %.1f]",
        i + 1, frontier_goals_.size(), path.size(), goal[0], goal[1], goal[2]);
      
      is_planning_ = false;
      in_loop = false;
      return;
    } else {
      RCLCPP_WARN(this->get_logger(),
        "RRT*: Goal #%zu/%zu [%.1f, %.1f, %.1f] blocked - path collision detected",
        i + 1, frontier_goals_.size(), goal[0], goal[1], goal[2]);
    }
  }
  
  // If we reach here, no path found to any goal
  RCLCPP_ERROR(this->get_logger(),
    "FAILED: No collision-free path found to any of %zu frontier goals with RRT*",
    frontier_goals_.size());
  in_loop = false;  

  
  is_planning_ = false;
}

std::vector<Eigen::Vector3d> RRTPathPlanner::planPathToTarget(
  const Eigen::Vector3d& start,
  const Eigen::Vector3d& goal, bool RRT_planning) {
  
  // Phase 1: Simple straight-line path or RRT* path
  std::vector<Eigen::Vector3d> path;
  //    path = generateSimplePath(start, goal, step_size_);
  path = generateRRTStarPath(start, goal);

  // If we have a collision checker, validate the straight-line path.
  if (collision_checker_) {
    if (collision_checker_->isPathCollisionFree(path, /*robot_radius=*/0.5, /*sample_resolution=*/0.25)) {
      return path;
    }

    // Path collision detected - return empty to let planPath() try next goal
    RCLCPP_DEBUG(this->get_logger(), 
      "Straight-line path to [%.2f, %.2f, %.2f] collides with obstacles", 
      goal[0], goal[1], goal[2]);
    return {};
  }

  // No collision checker available -> return straight path
  RCLCPP_DEBUG(this->get_logger(), 
      "No collision checker available with goal [%.2f, %.2f, %.2f].", 
      goal[0], goal[1], goal[2]);
  return {};
}

std::vector<Eigen::Vector3d> RRTPathPlanner::generateSimplePath(
  const Eigen::Vector3d& start,
  const Eigen::Vector3d& goal,
  double step_size) {
  
  std::vector<Eigen::Vector3d> path;
  
  // Add start point
  path.push_back(start);
  path.push_back(goal);
  
  return path;
}

bool RRTPathPlanner::isStateValid(const ompl::base::State *state) {
  const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
  
  octomap::point3d query((*pos)[0], (*pos)[1], (*pos)[2]);
  
  std::lock_guard<std::mutex> lock(octree_mutex_);
  
  if (!octree_) {
    return true;  // No map, assume valid
  }
  
  // Check if point is in occupied space (with safety margin)
  octomap::OcTreeNode* node = octree_->search(query);
  
  if (!node) {
    // Unknown space - be conservative in cave exploration
    return false;
  }
  
  // Check if occupied
  if (octree_->isNodeOccupied(node)) {
    return false;
  }
  
  // Check safety radius around robot
  // Sample points in a sphere around the position
  const int num_samples = 8;
  for (int i = 0; i < num_samples; ++i) {
    double theta = 2.0 * M_PI * i / num_samples;
    double phi = M_PI * (i % 3) / 3.0;
    
    double dx = robot_radius_rrt_ * sin(phi) * cos(theta);
    double dy = robot_radius_rrt_ * sin(phi) * sin(theta);
    double dz = robot_radius_rrt_ * cos(phi);
    
    octomap::point3d sample(
      (*pos)[0] + dx,
      (*pos)[1] + dy,
      (*pos)[2] + dz
    );
    
    octomap::OcTreeNode* sample_node = octree_->search(sample);
    if (sample_node && octree_->isNodeOccupied(sample_node)) {
      return false;  // Too close to obstacle
    }
  }
  
  return true;
}

std::vector<Eigen::Vector3d> RRTPathPlanner::generateRRTStarPath(
  const Eigen::Vector3d& start,
  const Eigen::Vector3d& goal) {
  
  RCLCPP_INFO(this->get_logger(), "Planning path with RRT*...");
  
  // Create 3D state space (R^3)
  auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);
  
  // Set bounds based on OctoMap or reasonable defaults
  ompl::base::RealVectorBounds bounds(3);
  {
    std::lock_guard<std::mutex> lock(octree_mutex_);
    if (octree_) {
      double minX, minY, minZ, maxX, maxY, maxZ;
      octree_->getMetricMin(minX, minY, minZ);
      octree_->getMetricMax(maxX, maxY, maxZ);
      
      // // Expand bounds slightly
      bounds.setLow(0, minX - 10.0);
      bounds.setLow(1, minY - 10.0);
      bounds.setLow(2, minZ - 10.0);
      bounds.setHigh(0, maxX + 10.0);
      bounds.setHigh(1, maxY + 10.0);
      bounds.setHigh(2, maxZ + 10.0);
      
      RCLCPP_DEBUG(this->get_logger(),
        "OctoMap bounds: [%.1f,%.1f] [%.1f,%.1f] [%.1f,%.1f]",
        minX, maxX, minY, maxY, minZ, maxZ);
    } else {
      // Default bounds (large cave area)
      bounds.setLow(0, -1500.0);
      bounds.setLow(1, -500.0);
      bounds.setLow(2, -100.0);
      bounds.setHigh(0, 0.0);
      bounds.setHigh(1, 500.0);
      bounds.setHigh(2, 100.0);
    }
  }
  space->setBounds(bounds);
  
  // Create SimpleSetup for planning
  ompl::geometric::SimpleSetup ss(space);
  
  // Set state validity checker (collision detection)
  ss.setStateValidityChecker([this](const ompl::base::State *state) {
    return this->isStateValid(state);
  });
  
  // Set collision checking resolution
  ss.getSpaceInformation()->setStateValidityCheckingResolution(
    collision_check_resolution_rrt_ / space->getMaximumExtent());
  
  // Set start and goal states
  ompl::base::ScopedState<> startState(space);
  startState[0] = start[0];
  startState[1] = start[1];
  startState[2] = start[2];
  
  ompl::base::ScopedState<> goalState(space);
  goalState[0] = goal[0];
  goalState[1] = goal[1];
  goalState[2] = goal[2];
  
  ss.setStartAndGoalStates(startState, goalState);
  
  // Create RRT* planner
  auto planner = std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());
  planner->setRange(10.0);  // Maximum distance between states
  ss.setPlanner(planner);
  
  // Set optimization objective (minimize path length)
  ss.setOptimizationObjective(
    std::make_shared<ompl::base::PathLengthOptimizationObjective>(ss.getSpaceInformation()));
  
  // Solve
  ompl::base::PlannerStatus solved = ss.solve(max_planning_time_);
  
  std::vector<Eigen::Vector3d> path;
  
  if (solved) {
    // Simplify the solution
    ss.simplifySolution();
    
    // Get solution path
    auto solution = ss.getSolutionPath();
    
    RCLCPP_INFO(this->get_logger(),
      "RRT* found path with %ld states, length %.2f m",
      solution.getStateCount(), solution.length());
    
    // Interpolate to reasonable number of waypoints (1 per 4 meters for faster flight)
    // Fewer waypoints = less accel/decel overhead = faster trajectories
    const double waypoint_spacing = 4.0;  // meters
    if (solution.getStateCount() > 2 && solution.length() > waypoint_spacing) {
      unsigned int target_waypoints = static_cast<unsigned int>(solution.length() / waypoint_spacing) + 1;
      solution.interpolate(std::max(target_waypoints, 3u));  // At least 3 waypoints (start, mid, end)
      RCLCPP_INFO(this->get_logger(),
        "Interpolated path to %ld waypoints (~%.1f m spacing)",
        solution.getStateCount(), waypoint_spacing);
    }
    
    // Convert to Eigen vectors
    for (std::size_t i = 0; i < solution.getStateCount(); ++i) {
      const auto *state = solution.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
      Eigen::Vector3d point;
      point << (*state)[0], (*state)[1], (*state)[2];
      path.push_back(point);
    }
    
    RCLCPP_INFO(this->get_logger(),
      "Final path has %zu waypoints for trajectory generation",
      path.size());
  } else {
    RCLCPP_ERROR(this->get_logger(), "RRT* failed to find a path");
  }
  
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
  
  // Check 1: Cave X-coordinates are always < -330 (cave entrance X)
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
