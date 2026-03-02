#include "ompl_planner_pkg/collision_checker.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/msg/point.hpp>

#include <fcl/narrowphase/collision.h>
#include <fcl/narrowphase/collision_request.h>
#include <fcl/narrowphase/collision_result.h>

FCLCollisionChecker::FCLCollisionChecker(double obstacle_radius)
  : obstacle_radius_(obstacle_radius) {}

void FCLCollisionChecker::setPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_msg) {
  obstacles_.clear();

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(*cloud_msg, cloud);

  // Convert each point into a small sphere collision object
  for (const auto& p : cloud.points) {
    // Skip invalid points
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) continue;

    auto sphere = std::make_shared<fcl::Sphered>(obstacle_radius_);
    std::shared_ptr<fcl::CollisionObjectd> obj = std::make_shared<fcl::CollisionObjectd>(
      sphere, fcl::Transform3d(fcl::Translation3d(p.x, p.y, p.z)));
    obstacles_.push_back(obj);
  }
}

bool FCLCollisionChecker::isPointInCollision(const Eigen::Vector3d& point, double robot_radius) const {
  // Create robot sphere
  auto robot_shape = std::make_shared<fcl::Sphered>(robot_radius);
  fcl::CollisionObjectd robot_obj(robot_shape, fcl::Transform3d(fcl::Translation3d(
    point[0], point[1], point[2])));

  fcl::CollisionRequestd request;
  fcl::CollisionResultd result;

  for (const auto& obs : obstacles_) {
    result.clear();
    fcl::collide(&robot_obj, obs.get(), request, result);
    if (result.isCollision()) return true;
  }

  return false;
}

bool FCLCollisionChecker::isPathCollisionFree(const std::vector<Eigen::Vector3d>& path,
                                             double robot_radius,
                                             double sample_resolution) const {
  if (path.size() < 2) return true;

  for (size_t i = 0; i + 1 < path.size(); ++i) {
    Eigen::Vector3d a = path[i];
    Eigen::Vector3d b = path[i+1];
    double seg_len = (b - a).norm();
    int samples = std::max(1, static_cast<int>(std::ceil(seg_len / sample_resolution)));

    for (int s = 0; s <= samples; ++s) {
      double t = static_cast<double>(s) / samples;
      Eigen::Vector3d sample = a + t * (b - a);
      if (isPointInCollision(sample, robot_radius)) return false;
    }
  }

  return true;
}
