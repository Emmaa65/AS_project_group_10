// Lightweight FCL-based collision checker wrapper
// Converts PointCloud2 occupancy into a set of collision objects (spheres)
// and checks collision for robot sphere along a path.

#ifndef OMPL_PLANNER_PKG_COLLISION_CHECKER_HPP_
#define OMPL_PLANNER_PKG_COLLISION_CHECKER_HPP_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>
#include <memory>
#include <vector>

#include <fcl/fcl.h>

class FCLCollisionChecker {
public:
  explicit FCLCollisionChecker(double obstacle_radius = 0.3);

  // Set occupancy point cloud (stores obstacles as small spheres)
  void setPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg);

  // Check whether a single point (robot center) is in collision
  bool isPointInCollision(const Eigen::Vector3d& point, double robot_radius) const;

  // Check whether a path (list of 3D points) is collision free for a spherical robot
  // Samples along straight-line segments at the given resolution (m).
  bool isPathCollisionFree(const std::vector<Eigen::Vector3d>& path,
                           double robot_radius,
                           double sample_resolution = 0.25) const;

private:
  std::vector<std::shared_ptr<fcl::CollisionObjectd>> obstacles_;
  double obstacle_radius_ = 0.3;
};

#endif // OMPL_PLANNER_PKG_COLLISION_CHECKER_HPP_
