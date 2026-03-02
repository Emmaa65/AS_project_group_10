/**
 * RRT Path Planner Header
 */

#ifndef RRT_PATH_PLANNER_HPP_
#define RRT_PATH_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mav_planning_msgs/msg/polynomial_trajectory4_d.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation/ros_conversions.h>
#include <mav_msgs/eigen_mav_msgs.hpp>
#include <Eigen/Dense>
#include <vector>
#include <memory>
#include <mutex>
#include <random>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include "ompl_planner_pkg/collision_checker.hpp"

// OMPL headers
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

// OctoMap headers
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

class RRTPathPlanner : public rclcpp::Node {
public:
  RRTPathPlanner();
  
private:
  // ROS2 Subscribers/Publishers
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr sub_frontier_goals_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_occupancy_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_octomap_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_;
  
  rclcpp::Publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_plan_markers_;
  
  rclcpp::TimerBase::SharedPtr planning_timer_;
  
  // Current state
  std::string exploration_state_ = "INITIALIZATION";
  Eigen::Vector3d current_position_ = Eigen::Vector3d::Zero();
  std::vector<geometry_msgs::msg::Pose> frontier_goals_; // List of ranked goals from frontier_exploration
  std::vector<geometry_msgs::msg::Pose> pending_frontier_goals_; // Buffered goals from callback
  size_t current_goal_index_ = 0;  // Current goal being planned to
  Eigen::Vector3d last_planned_target_ = Eigen::Vector3d::Zero();
  bool has_targets_ = false;  // Changed from has_target to indicate multiple goals
  bool is_planning_ = false;  // Flag to prevent goal updates during planning cycle
  std::mutex goals_mutex_;  // Protect goal list access from callback and planning thread
  
  // Cave entrance for filtering invalid frontiers
  Eigen::Vector3d cave_entrance_ = Eigen::Vector3d(-330.0, 10.0, 20.0);
  double min_frontier_z_ = 33.5; // Minimum safe height (cave is above Z=14, safety margin)
  
  // Planning parameters
  double max_planning_time_ = 1.0;
  double step_size_ = 0.5;
  int max_iterations_ = 1000;
  double max_velocity_ = 10.0;
  double max_acceleration_ = 2.0;
  double replan_threshold_ = 2.0; // Only replan if target moves more than this
  
  // OMPL RRT* parameters
  double collision_check_resolution_rrt_ = 0.3;  // Resolution for collision checking (meters)
  double robot_radius_rrt_ = 0.3;  // Safety sphere around drone (meters); drone is 0.2x0.2m

  // Collision checking
  std::shared_ptr<FCLCollisionChecker> collision_checker_;

  // OctoMap for collision checking
  std::shared_ptr<octomap::OcTree> octree_;
  std::mutex octree_mutex_;
  
  // Callbacks
  void frontierGoalsCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void occupancyCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void stateCallback(const std_msgs::msg::String::SharedPtr msg);
  void planPath();
  
  // Helper method to get current target goal
  Eigen::Vector3d getCurrentGoal() const {
    if (frontier_goals_.empty()) return Eigen::Vector3d::Zero();
    if (current_goal_index_ >= frontier_goals_.size()) return Eigen::Vector3d::Zero();
    const auto& pose = frontier_goals_[current_goal_index_];
    return Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  }
  
  // Path planning methods
  std::vector<Eigen::Vector3d> planPathToTarget(
    const Eigen::Vector3d& start, 
    const Eigen::Vector3d& goal,
    bool RRT_planning);
  bool in_loop = false;

  bool isStateValid(const ompl::base::State *state);
  
  std::vector<Eigen::Vector3d> generateRRTStarPath(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal);
  
  std::vector<Eigen::Vector3d> generateSimplePath(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal,
    double step_size = 0.5);
  
  // Frontier validation
  bool isFrontierValid(const Eigen::Vector3d& frontier);
  
  void publishTrajectory(const std::vector<Eigen::Vector3d>& path);
  void publishPlanMarkers(const std::vector<Eigen::Vector3d>& path);
};

#endif // RRT_PATH_PLANNER_HPP_
