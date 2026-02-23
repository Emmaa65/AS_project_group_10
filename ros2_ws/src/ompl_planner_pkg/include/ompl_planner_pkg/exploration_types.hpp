#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <Eigen/Dense>
#include <vector>

/**
 * Exploration State enum
 */
enum class ExplorationState {
  INITIALIZATION,           // Waiting for initial odometry
  WAYPOINT_NAVIGATION,      // Flying to cave entrance via predefined waypoints  
  WAITING_AT_ENTRANCE,      // Reached cave entrance, preparing for exploration
  AUTONOMOUS_EXPLORATION,   // Exploring cave with frontiers
  EXPLORATION_COMPLETE,     // No more frontiers found
  ERROR_STATE               // Something went wrong
};

/**
 * Helper struct for frontier point data
 */
struct FrontierPoint {
  Eigen::Vector3d position;
  double info_gain;        // How much new information at this frontier
  double distance;         // Distance from current position
  int frontier_id;
  
  // Score: balance between info gain and distance
  double getScore() const {
    if (distance < 0.1) return -1e9; // Avoid division by zero
    return info_gain / (distance + 0.1); // +0.1 to avoid division by zero
  }
};

/**
 * Helper function to convert ROS Odometry to Eigen format
 */
inline Eigen::Vector3d odometryToPosition(const nav_msgs::msg::Odometry& odom) {
  return Eigen::Vector3d(
    odom.pose.pose.position.x,
    odom.pose.pose.position.y,
    odom.pose.pose.position.z
  );
}

/**
 * Helper function to check if drone reached a goal position
 */
inline bool hasReachedGoal(const Eigen::Vector3d& current, 
                          const Eigen::Vector3d& goal,
                          double tolerance = 0.5) {
  return (current - goal).norm() < tolerance;
}
