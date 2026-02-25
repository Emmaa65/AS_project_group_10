// Simple OMPL RRT* planner node
// Subscribes to `/exploration/frontier_goal` (PoseStamped) and `/current_state` (Odometry)
// Plans in SE(2) using RRT* and publishes `nav_msgs::msg::Path` on `/planned_path`.

#include <memory>
#include <vector>
#include <chrono>
#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>

// OMPL
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/spaces/SE2StateSpace.h>


// Octomap
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/OcTree.h>

// FCL
#include <fcl/fcl.h>


namespace ob = ompl::base;
namespace og = ompl::geometric;



class PathPlanner : public rclcpp::Node {
public:
  PathPlanner()
  : Node("path_planner")
  {
    using namespace std::chrono_literals;

    current_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "current_state", 10, std::bind(&PathPlanner::currentStateCallback, this, std::placeholders::_1)
    );

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/exploration/frontier_goal", 10,
      std::bind(&PathPlanner::goalCallback, this, std::placeholders::_1)
    );

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      "octomap_full", 1, std::bind(&PathPlanner::octomapCallback, this, std::placeholders::_1)
    );
    
    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);


    // Collision manager (FCL)
    collision_manager_ = std::make_shared<fcl::DynamicAABBTreeCollisionManagerd>();

    // Parameters
    this->declare_parameter<double>("planning_time", 1.0);
    this->declare_parameter<double>("min_x", -500.0);
    this->declare_parameter<double>("max_x", 500.0);
    this->declare_parameter<double>("min_y", -100.0);
    this->declare_parameter<double>("max_y", 100.0);
    this->declare_parameter<double>("drone_radius", 0.35);
    this->declare_parameter<double>("goal_bias", 0.05);

    RCLCPP_INFO(this->get_logger(), "PathPlanner ready");
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_state_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;

  // FCL collision manager and stored obstacle objects
  std::shared_ptr<fcl::DynamicAABBTreeCollisionManagerd> collision_manager_;
  std::vector<std::shared_ptr<fcl::CollisionObjectd>> obstacle_objs_;
  std::mutex octree_mutex_;
  double drone_radius_ = 0.35;

  // latest start
  std::mutex start_mutex_;
  geometry_msgs::msg::Pose current_pose_;
  geometry_msgs::msg::Twist current_velocity_;
  bool have_start_ = false;
  double goal_bias_ = 1.0;

  // copy current start pose thread-safely
  bool prepareStartFromCurrent(geometry_msgs::msg::Pose & start_copy) {
    std::lock_guard<std::mutex> lk(start_mutex_);
    if (!have_start_) return false;
    start_copy = current_pose_;
    return true;
  }

  // read planning parameters
  void getPlanningParameters(double &planning_time, double &min_x, double &max_x, double &min_y, double &max_y) {
    planning_time = this->get_parameter("planning_time").as_double();
    min_x = this->get_parameter("min_x").as_double();
    max_x = this->get_parameter("max_x").as_double();
    min_y = this->get_parameter("min_y").as_double();
    max_y = this->get_parameter("max_y").as_double();
    // optional parameters
    if (this->has_parameter("goal_bias")) goal_bias_ = this->get_parameter("goal_bias").as_double();
    if (this->has_parameter("drone_radius")) drone_radius_ = this->get_parameter("drone_radius").as_double();
  }

  // create bounded SE2 space
  std::shared_ptr<ob::SE2StateSpace> createSE2Space(double min_x, double max_x, double min_y, double max_y) {
    auto space = std::make_shared<ob::SE2StateSpace>();
    ob::RealVectorBounds bounds(2);
    bounds.setLow(0, min_x);
    bounds.setHigh(0, max_x);
    bounds.setLow(1, min_y);
    bounds.setHigh(1, max_y);
    space->setBounds(bounds);
    return space;
  }

  // build start ScopedState from Pose
  ob::ScopedState<ob::SE2StateSpace> makeStartState(const std::shared_ptr<ob::SE2StateSpace> &space, const geometry_msgs::msg::Pose &start_pose) {
    ob::ScopedState<ob::SE2StateSpace> s(space);
    s->setX(start_pose.position.x);
    s->setY(start_pose.position.y);
    double sx = start_pose.orientation.x;
    double sy = start_pose.orientation.y;
    double sz = start_pose.orientation.z;
    double sw = start_pose.orientation.w;
    double yaw = std::atan2(2.0*(sw*sz + sx*sy), 1.0 - 2.0*(sy*sy + sz*sz));
    s->setYaw(yaw);
    return s;
  }

  // build goal ScopedState from Pose
  ob::ScopedState<ob::SE2StateSpace> makeGoalState(const std::shared_ptr<ob::SE2StateSpace> &space, const geometry_msgs::msg::Pose &goal_pose) {
    ob::ScopedState<ob::SE2StateSpace> g(space);
    g->setX(goal_pose.position.x);
    g->setY(goal_pose.position.y);
    double gx = goal_pose.orientation.x;
    double gy = goal_pose.orientation.y;
    double gz = goal_pose.orientation.z;
    double gw = goal_pose.orientation.w;
    double yaw = std::atan2(2.0*(gw*gz + gx*gy), 1.0 - 2.0*(gy*gy + gz*gz));
    g->setYaw(yaw);
    return g;
  }

  // convert OMPL PathGeometric to nav_msgs::msg::Path
  nav_msgs::msg::Path makeRosPathFromGeometric(const og::PathGeometric &path_geom) {
    nav_msgs::msg::Path ros_path;

    ros_path.header.stamp = this->now();
    ros_path.header.frame_id = "map";
    for (std::size_t i = 0; i < path_geom.getStateCount(); ++i) {
      const ob::SE2StateSpace::StateType *s = path_geom.getState(i)->as<ob::SE2StateSpace::StateType>();
      geometry_msgs::msg::PoseStamped p;
      p.header = ros_path.header;
      p.pose.position.x = s->getX();
      p.pose.position.y = s->getY();
      p.pose.position.z = 0.0;
      double yaw = s->getYaw();
      p.pose.orientation.x = 0.0;
      p.pose.orientation.y = 0.0;
      p.pose.orientation.z = std::sin(yaw/2.0);
      p.pose.orientation.w = std::cos(yaw/2.0);
      ros_path.poses.push_back(p);
    }
    return ros_path;
  }

  // Octomap callback: convert octomap to FCL obstacles
  void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    if (!msg) return;
    octomap::AbstractOcTree *tree = octomap_msgs::fullMsgToMap(*msg);
    if (!tree) return;
    octomap::OcTree *octree = dynamic_cast<octomap::OcTree*>(tree);
    if (!octree) { delete tree; return; }

    std::vector<std::shared_ptr<fcl::CollisionObjectd>> new_objs;
    for (auto it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it) {
      if (!octree->isNodeOccupied(*it)) continue;
      double x = it.getX();
      double y = it.getY();
      double z = it.getZ();
      double size = it.getSize();
      // create box at voxel center
      auto box = std::make_shared<fcl::Boxd>(size, size, size);
      fcl::Transform3d tf = fcl::Transform3d(fcl::Translation3d(x, y, z));
      auto obj = std::make_shared<fcl::CollisionObjectd>(box, tf);
      new_objs.push_back(obj);
    }
    
    {
      std::lock_guard<std::mutex> lk(octree_mutex_);
      obstacle_objs_.swap(new_objs);
      collision_manager_->clear();
      if (!obstacle_objs_.empty()) {
        std::vector<fcl::CollisionObjectd*> raw_objs;
        raw_objs.reserve(obstacle_objs_.size());
        for (auto &o : obstacle_objs_) raw_objs.push_back(o.get());
        collision_manager_->registerObjects(raw_objs);
      }
      collision_manager_->setup();
      // Debug: verify FCL objects and do a sample collision test
      RCLCPP_INFO(this->get_logger(), "Registered %zu FCL obstacle objects", obstacle_objs_.size());
      if (!obstacle_objs_.empty()) {
      // build a test drone sphere at the same transform as the first obstacle
      auto &first_obs = obstacle_objs_[0];
      fcl::Transform3d obs_tf = first_obs->getTransform(); // get obstacle transform
      auto test_sphere = std::make_shared<fcl::Sphered>(drone_radius_);
      fcl::CollisionObjectd test_drone(test_sphere, obs_tf);

      fcl::CollisionRequestd req;
      fcl::CollisionResultd res;
      fcl::collide(&test_drone, first_obs.get(), req, res);

      RCLCPP_INFO(this->get_logger(), "FCL debug test against first obstacle: %s",
                  res.isCollision() ? "COLLISION" : "NO_COLLISION");
}
    }

    delete octree;
  }

  // State validity check using FCL collision manager (sphere drone)
  bool isStateValid(const ob::State *state) {
    if (!state) return false;
    const ob::SE2StateSpace::StateType *s = state->as<ob::SE2StateSpace::StateType>();
    double x = s->getX();
    double y = s->getY();

    std::lock_guard<std::mutex> lk(octree_mutex_);
    if (!collision_manager_ || obstacle_objs_.empty()) {
      // no map data yet -> consider valid
      return true;
    }

    auto sphere = std::make_shared<fcl::Sphered>(drone_radius_);
    fcl::Transform3d tf = fcl::Transform3d(fcl::Translation3d(x, y, 0.0));
    fcl::CollisionObjectd drone_obj(sphere, tf);

    fcl::CollisionRequestd req;
    fcl::CollisionResultd res;
    // pairwise collide against stored obstacles
    for (const auto &obs : obstacle_objs_) {
      res.clear();
      fcl::collide(&drone_obj, obs.get(), req, res);
      if (res.isCollision()) return false;
    }
    return true;
  }



  void currentStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    std::lock_guard<std::mutex> lk(start_mutex_);
    current_pose_ = msg->pose.pose;
    current_velocity_ = msg->twist.twist;
    have_start_ = true;
    RCLCPP_INFO(this->get_logger(), "Current pose at (%.2f, %.2f, %.2f)",
            current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!msg) return;
    geometry_msgs::msg::PoseStamped goal_msg = *msg;

    RCLCPP_INFO(this->get_logger(), "Received frontier goal: (%.3f, %.3f, %.3f) frame=%s",
                goal_msg.pose.position.x, goal_msg.pose.position.y, goal_msg.pose.position.z,
                goal_msg.header.frame_id.c_str());

    // prepare start
    geometry_msgs::msg::Pose start_copy;
    if (!prepareStartFromCurrent(start_copy)) {
      RCLCPP_WARN(this->get_logger(), "No start pose received yet; cannot plan");
      return;
    }

    // parameters and space
    double planning_time, min_x, max_x, min_y, max_y;
    getPlanningParameters(planning_time, min_x, max_x, min_y, max_y);
    auto space = createSE2Space(min_x, max_x, min_y, max_y);

    og::SimpleSetup ss(space);
    ss.setStateValidityChecker([this](const ob::State *state) { return this->isStateValid(state); });

    // start and goal states
    auto start = makeStartState(space, start_copy);
    auto goal_state = makeGoalState(space, goal_msg.pose);
    ss.setStartAndGoalStates(start, goal_state);

    // planner
    auto planner(std::make_shared<og::RRTstar>(ss.getSpaceInformation()));
    planner->setGoalBias(goal_bias_);
    ss.setPlanner(planner);
    ss.setup();

    ob::PlannerStatus solved = ss.solve(planning_time);
    if (solved) {
      RCLCPP_INFO(this->get_logger(), "Found solution; extracting path");
      og::PathGeometric path = ss.getSolutionPath();
      path.interpolate();
      auto ros_path = makeRosPathFromGeometric(path);
      RCLCPP_INFO(this->get_logger(), "ros_path: frame=%s, poses=%zu",
                  ros_path.header.frame_id.c_str(), ros_path.poses.size());
      // Print first few poses for debug (avoid flooding logs)
      for (std::size_t i = 0; i < ros_path.poses.size() && i < 10; ++i) {
        RCLCPP_INFO(this->get_logger(), "  pose[%zu]= (%.3f, %.3f, %.3f)",
                    i,
                    ros_path.poses[i].pose.position.x,
                    ros_path.poses[i].pose.position.y,
                    ros_path.poses[i].pose.position.z);
      }
      // validate all poses before publishing
      for (const auto &ps : ros_path.poses) {
      ob::ScopedState<ob::SE2StateSpace> s(space);
      s->setX(ps.pose.position.x);
      s->setY(ps.pose.position.y);
      double qz = ps.pose.orientation.z;
      double qw = ps.pose.orientation.w;
      double yaw = std::atan2(2.0*(qw*qz), 1.0 - 2.0*(qz*qz)); // approximate from z,w
      s->setYaw(yaw);
      if (!isStateValid(s.get())) {
          RCLCPP_WARN(this->get_logger(), "Planned path failed post-check at (%.3f, %.3f)", s->getX(), s->getY());
          return;
      }
      }
      path_pub_->publish(ros_path);
      RCLCPP_INFO(this->get_logger(), "Published path with %zu poses", ros_path.poses.size());
    } else {
      RCLCPP_WARN(this->get_logger(), "No path found within %.2f seconds", planning_time);
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PathPlanner>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
