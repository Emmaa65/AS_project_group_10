/**
 * RRT Path Planner Header
 */

#ifndef RRT_PATH_PLANNER_HPP_
#define RRT_PATH_PLANNER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mav_planning_msgs/msg/polynomial_trajectory4_d.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation/ros_conversions.h>
#include <mav_msgs/eigen_mav_msgs.hpp>
#include <Eigen/Dense>
#include <vector>
#include <memory>

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
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_target_frontier_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_occupancy_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr sub_octomap_;
  
  rclcpp::Publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr pub_trajectory_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_plan_markers_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_planning_result_;
  
  rclcpp::TimerBase::SharedPtr planning_timer_;
  
  // Current state
  std::string exploration_state_ = "INITIALIZATION";
  Eigen::Vector3d current_position_    = Eigen::Vector3d::Zero();
  Eigen::Vector3d target_frontier_     = Eigen::Vector3d::Zero();
  Eigen::Vector3d last_planned_target_ = Eigen::Vector3d::Zero();
  bool has_target_   = false;
  bool need_replan_  = false;  // set true when a new frontier is committed

  // Cave entrance for filtering invalid frontiers
  Eigen::Vector3d cave_entrance_ = Eigen::Vector3d(-330.0, 10.0, 20.0);
  double min_frontier_z_ = 33.5;
  
  // Planning parameters
  double max_planning_time_  = 2.0;
  double step_size_          = 3.0;
  int    max_iterations_     = 1000;
  double max_velocity_       = 10.0;
  double max_acceleration_   = 2.0;
  double replan_threshold_   = 5.0;
  
  // OMPL / collision parameters
  double collision_check_resolution_ = 0.5;
  double robot_radius_               = 1.0;
  
  // OctoMap for collision checking
  std::shared_ptr<octomap::OcTree> octree_;
  std::mutex octree_mutex_;
  
  // Callbacks
  void targetFrontierCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void occupancyCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg);
  void stateCallback(const std_msgs::msg::String::SharedPtr msg);
  void planPath();
  
  // Path planning methods
  std::vector<Eigen::Vector3d> planPathToTarget(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal);
  
  std::vector<Eigen::Vector3d> planPathWithRRTStar(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal);
  
  bool isStateValid(const ompl::base::State *state);
  
  std::vector<Eigen::Vector3d> generateSimplePath(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& goal,
    double step_size = 3.0);
  
  void publishPlanningResult(bool success);
  bool isFrontierValid(const Eigen::Vector3d& frontier);
  void publishTrajectory(const std::vector<Eigen::Vector3d>& path);
  void publishPlanMarkers(const std::vector<Eigen::Vector3d>& path);
};

#endif // RRT_PATH_PLANNER_HPP_