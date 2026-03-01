/**
 * RRT Path Planner Node
 *
 * Plans 3D paths from current position to frontier targets using RRT*.
 * Replans ONLY when the frontier explorer commits a new target that is
 * significantly different from the current one.  While flying, it leaves
 * the trajectory executor alone.
 */

#include "ompl_planner_pkg/rrt_path_planner.hpp"

RRTPathPlanner::RRTPathPlanner()
    : rclcpp::Node("rrt_path_planner")
{
    RCLCPP_INFO(this->get_logger(), "Initializing RRT Path Planner...");

    this->declare_parameter("max_planning_time",         2.0);  // more time = better path
    this->declare_parameter("step_size",                 3.0);
    this->declare_parameter("max_iterations",            1000);
    this->declare_parameter("max_velocity",              10.0);
    this->declare_parameter("max_acceleration",          2.0);
    this->declare_parameter("replan_threshold",          5.0);   // metres — new target must differ by this much
    this->declare_parameter("cave_entrance_x",          -330.0);
    this->declare_parameter("cave_entrance_y",           10.0);
    this->declare_parameter("cave_entrance_z",           20.0);
    this->declare_parameter("min_frontier_z",           -33.5);
    this->declare_parameter("collision_check_resolution", 0.5);
    this->declare_parameter("robot_radius",              1.0);

    max_planning_time_        = this->get_parameter("max_planning_time").as_double();
    step_size_                = this->get_parameter("step_size").as_double();
    max_iterations_           = this->get_parameter("max_iterations").as_int();
    max_velocity_             = this->get_parameter("max_velocity").as_double();
    max_acceleration_         = this->get_parameter("max_acceleration").as_double();
    replan_threshold_         = this->get_parameter("replan_threshold").as_double();
    cave_entrance_[0]         = this->get_parameter("cave_entrance_x").as_double();
    cave_entrance_[1]         = this->get_parameter("cave_entrance_y").as_double();
    cave_entrance_[2]         = this->get_parameter("cave_entrance_z").as_double();
    min_frontier_z_           = this->get_parameter("min_frontier_z").as_double();
    collision_check_resolution_ = this->get_parameter("collision_check_resolution").as_double();
    robot_radius_             = this->get_parameter("robot_radius").as_double();

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

    sub_octomap_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        "octomap_full", 10,
        std::bind(&RRTPathPlanner::octomapCallback, this, std::placeholders::_1));

    sub_state_ = this->create_subscription<std_msgs::msg::String>(
        "/exploration_state", 10,
        std::bind(&RRTPathPlanner::stateCallback, this, std::placeholders::_1));

    // Publishers
    pub_trajectory_ = this->create_publisher<mav_planning_msgs::msg::PolynomialTrajectory4D>(
        "trajectory", 10);
    pub_plan_markers_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "planned_path_markers", 10);
    pub_planning_result_ = this->create_publisher<std_msgs::msg::Bool>(
        "planning_result", 10);

    // Planning timer — runs at 2 Hz but only does work when a new target arrives
    planning_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&RRTPathPlanner::planPath, this));

    RCLCPP_INFO(this->get_logger(),
        "RRT Path Planner ready (max_v=%.1f m/s, max_a=%.1f m/s², replan_thresh=%.1f m)",
        max_velocity_, max_acceleration_, replan_threshold_);
}

// =============================================================================
// Callbacks
// =============================================================================

void RRTPathPlanner::targetFrontierCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    Eigen::Vector3d incoming;
    incoming << msg->point.x, msg->point.y, msg->point.z;

    if (!isFrontierValid(incoming)) {
        RCLCPP_WARN(this->get_logger(),
            "Rejecting invalid frontier [%.2f, %.2f, %.2f]",
            incoming[0], incoming[1], incoming[2]);
        return;
    }

    // Only accept if sufficiently different from the target we are already flying to
    double jump = (incoming - target_frontier_).norm();
    if (has_target_ && jump < replan_threshold_) {
        RCLCPP_DEBUG(this->get_logger(),
            "Ignoring frontier [%.2f, %.2f, %.2f] — only %.2f m from current target",
            incoming[0], incoming[1], incoming[2], jump);
        return;
    }

    target_frontier_ = incoming;
    has_target_      = true;
    need_replan_     = true;   // signal planPath() to act immediately

    RCLCPP_INFO(this->get_logger(),
        "New frontier accepted [%.2f, %.2f, %.2f] (jump=%.2f m) — will replan",
        target_frontier_[0], target_frontier_[1], target_frontier_[2], jump);
}

void RRTPathPlanner::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_position_ <<
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z;
}

void RRTPathPlanner::occupancyCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr /*msg*/) {}

void RRTPathPlanner::octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
    auto *tree = dynamic_cast<octomap::OcTree*>(octomap_msgs::msgToMap(*msg));
    if (!tree) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Failed to convert octomap message to OcTree");
        return;
    }
    std::lock_guard<std::mutex> lock(octree_mutex_);
    octree_.reset(tree);
}

void RRTPathPlanner::stateCallback(const std_msgs::msg::String::SharedPtr msg) {
    exploration_state_ = msg->data;
    RCLCPP_INFO(this->get_logger(), "State: %s", exploration_state_.c_str());
}

// =============================================================================
// Planning timer — only fires when need_replan_ is true
// =============================================================================

void RRTPathPlanner::planPath() {
    if (exploration_state_ != "AUTONOMOUS_EXPLORATION") return;
    if (!has_target_) return;
    if (!need_replan_) return;   // nothing to do

    need_replan_ = false;  // consume the flag

    RCLCPP_INFO(this->get_logger(),
        "Planning path [%.2f,%.2f,%.2f] → [%.2f,%.2f,%.2f]",
        current_position_[0], current_position_[1], current_position_[2],
        target_frontier_[0],  target_frontier_[1],  target_frontier_[2]);

    auto path = planPathToTarget(current_position_, target_frontier_);

    if (!path.empty()) {
        publishTrajectory(path);
        publishPlanMarkers(path);
        publishPlanningResult(true);
        last_planned_target_ = target_frontier_;
        RCLCPP_INFO(this->get_logger(),
            "Trajectory published (%zu waypoints)", path.size());
    } else {
        RCLCPP_ERROR(this->get_logger(), "RRT* failed — no path found");
        publishPlanningResult(false);
    }
}

// =============================================================================
// Path planning
// =============================================================================

std::vector<Eigen::Vector3d> RRTPathPlanner::planPathToTarget(
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &goal)
{
    {
        std::lock_guard<std::mutex> lock(octree_mutex_);
        if (!octree_) {
            RCLCPP_WARN(this->get_logger(), "No OctoMap yet — using straight line");
            return generateSimplePath(start, goal, step_size_);
        }
    }

    auto path = planPathWithRRTStar(start, goal);
    if (path.empty()) {
        RCLCPP_WARN(this->get_logger(), "RRT* failed — falling back to straight line");
        return generateSimplePath(start, goal, step_size_);
    }
    return path;
}

std::vector<Eigen::Vector3d> RRTPathPlanner::planPathWithRRTStar(
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &goal)
{
    auto space = std::make_shared<ompl::base::RealVectorStateSpace>(3);

    ompl::base::RealVectorBounds bounds(3);
    {
        std::lock_guard<std::mutex> lock(octree_mutex_);
        if (octree_) {
            double mnX, mnY, mnZ, mxX, mxY, mxZ;
            octree_->getMetricMin(mnX, mnY, mnZ);
            octree_->getMetricMax(mxX, mxY, mxZ);
            bounds.setLow(0, mnX - 10); bounds.setHigh(0, mxX + 10);
            bounds.setLow(1, mnY - 10); bounds.setHigh(1, mxY + 10);
            bounds.setLow(2, mnZ - 10); bounds.setHigh(2, mxZ + 10);
        } else {
            bounds.setLow(0, -500); bounds.setHigh(0, 0);
            bounds.setLow(1, -100); bounds.setHigh(1, 100);
            bounds.setLow(2, -100); bounds.setHigh(2, 100);
        }
    }
    space->setBounds(bounds);

    ompl::geometric::SimpleSetup ss(space);
    ss.setStateValidityChecker([this](const ompl::base::State *s) {
        return this->isStateValid(s);
    });
    ss.getSpaceInformation()->setStateValidityCheckingResolution(
        collision_check_resolution_ / space->getMaximumExtent());

    ompl::base::ScopedState<> s0(space), s1(space);
    s0[0]=start[0]; s0[1]=start[1]; s0[2]=start[2];
    s1[0]=goal[0];  s1[1]=goal[1];  s1[2]=goal[2];
    ss.setStartAndGoalStates(s0, s1);

    auto planner = std::make_shared<ompl::geometric::RRTstar>(ss.getSpaceInformation());
    planner->setRange(10.0);
    ss.setPlanner(planner);
    ss.setOptimizationObjective(
        std::make_shared<ompl::base::PathLengthOptimizationObjective>(
            ss.getSpaceInformation()));

    std::vector<Eigen::Vector3d> path;
    if (!ss.solve(max_planning_time_)) {
        RCLCPP_ERROR(this->get_logger(), "RRT* found no solution");
        return path;
    }

    ss.simplifySolution();
    auto &sol = ss.getSolutionPath();

    RCLCPP_INFO(this->get_logger(),
        "RRT* solution: %ld states, length=%.1f m",
        sol.getStateCount(), sol.length());

    // Reduce to ~1 waypoint per 15 m (was 1 per 1 m — caused 200-segment trajectories)
    if (sol.getStateCount() > 2 && sol.length() > 1.0) {
        unsigned int n = std::max(2u,
            static_cast<unsigned int>(sol.length() / 15.0));
        sol.interpolate(n);
        RCLCPP_INFO(this->get_logger(), "Decimated to %u waypoints", n);
    }

    for (std::size_t i = 0; i < sol.getStateCount(); ++i) {
        const auto *st = sol.getState(i)
            ->as<ompl::base::RealVectorStateSpace::StateType>();
        path.push_back({(*st)[0], (*st)[1], (*st)[2]});
    }
    return path;
}

bool RRTPathPlanner::isStateValid(const ompl::base::State *state) {
    const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
    std::lock_guard<std::mutex> lock(octree_mutex_);
    if (!octree_) return true;

    octomap::OcTreeNode *node = octree_->search((*pos)[0], (*pos)[1], (*pos)[2]);
    if (!node) return false;                          // unknown = invalid in cave
    if (octree_->isNodeOccupied(node)) return false;

    // Safety sphere
    const int n = 8;
    for (int i = 0; i < n; ++i) {
        double theta = 2.0 * M_PI * i / n;
        double phi   = M_PI * (i % 3) / 3.0;
        octomap::point3d sample(
            (*pos)[0] + robot_radius_ * sin(phi) * cos(theta),
            (*pos)[1] + robot_radius_ * sin(phi) * sin(theta),
            (*pos)[2] + robot_radius_ * cos(phi));
        auto *sn = octree_->search(sample);
        if (sn && octree_->isNodeOccupied(sn)) return false;
    }
    return true;
}

std::vector<Eigen::Vector3d> RRTPathPlanner::generateSimplePath(
    const Eigen::Vector3d &start,
    const Eigen::Vector3d &goal,
    double /*step_size*/)
{
    return {start, goal};
}

// =============================================================================
// Trajectory publishing
// =============================================================================

void RRTPathPlanner::publishTrajectory(const std::vector<Eigen::Vector3d> &path) {
    if (path.size() < 2) return;

    try {
        const int dim = 3;
        const int deriv = mav_trajectory_generation::derivative_order::SNAP;
        mav_trajectory_generation::Vertex::Vector vertices;

        // Start
        mav_trajectory_generation::Vertex vstart(dim);
        vstart.makeStartOrEnd(current_position_, deriv);
        vstart.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                             Eigen::Vector3d::Zero());
        vertices.push_back(vstart);

        // Intermediate waypoints — spaced at least 10 m apart to keep
        // segment count low and trajectory fast
        const double MIN_SPACING = 10.0;
        Eigen::Vector3d last_added = path.front();
        for (size_t i = 1; i + 1 < path.size(); ++i) {
            if ((path[i] - last_added).norm() >= MIN_SPACING) {
                mav_trajectory_generation::Vertex wp(dim);
                wp.addConstraint(
                    mav_trajectory_generation::derivative_order::POSITION, path[i]);
                vertices.push_back(wp);
                last_added = path[i];
            }
        }

        // End
        mav_trajectory_generation::Vertex vend(dim);
        vend.makeStartOrEnd(path.back(), deriv);
        vend.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY,
                           Eigen::Vector3d::Zero());
        vertices.push_back(vend);

        RCLCPP_INFO(this->get_logger(),
            "Building trajectory: %zu vertices from %zu RRT waypoints",
            vertices.size(), path.size());

        auto seg_times = mav_trajectory_generation::estimateSegmentTimes(
            vertices, max_velocity_, max_acceleration_);

        double total = 0;
        for (auto t : seg_times) total += t;
        RCLCPP_INFO(this->get_logger(),
            "Estimated trajectory time: %.1f s (%zu segments)",
            total, seg_times.size());

        mav_trajectory_generation::NonlinearOptimizationParameters nlp;
        mav_trajectory_generation::PolynomialOptimizationNonLinear<10> prob(dim, nlp);
        if (!prob.setupFromVertices(vertices, seg_times,
            mav_trajectory_generation::PolynomialOptimization<10>::kHighestDerivativeToOptimize)) {
            RCLCPP_ERROR(this->get_logger(), "setupFromVertices failed");
            return;
        }
        if (!prob.solveLinear()) {
            RCLCPP_ERROR(this->get_logger(), "solveLinear failed");
            return;
        }

        mav_trajectory_generation::Trajectory traj;
        prob.getTrajectory(&traj);

        mav_planning_msgs::msg::PolynomialTrajectory4D msg;
        mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(traj, &msg);
        msg.header.stamp    = this->get_clock()->now();
        msg.header.frame_id = "world";
        pub_trajectory_->publish(msg);

    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "publishTrajectory exception: %s", e.what());
    }
}

void RRTPathPlanner::publishPlanMarkers(const std::vector<Eigen::Vector3d> &path) {
    auto markers = std::make_unique<visualization_msgs::msg::MarkerArray>();

    visualization_msgs::msg::Marker line;
    line.header.frame_id = "world";
    line.header.stamp    = this->get_clock()->now();
    line.ns   = "planned_path";
    line.id   = 0;
    line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line.action = visualization_msgs::msg::Marker::ADD;
    for (const auto &p : path) {
        geometry_msgs::msg::Point pt;
        pt.x=p[0]; pt.y=p[1]; pt.z=p[2];
        line.points.push_back(pt);
    }
    line.scale.x = 0.1;
    line.color.b = 1.0f; line.color.a = 0.8f;
    markers->markers.push_back(line);

    for (size_t i = 0; i < path.size(); ++i) {
        visualization_msgs::msg::Marker sphere;
        sphere.header.frame_id = "world";
        sphere.header.stamp    = this->get_clock()->now();
        sphere.ns   = "path_waypoints";
        sphere.id   = i;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position.x = path[i][0];
        sphere.pose.position.y = path[i][1];
        sphere.pose.position.z = path[i][2];
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2;
        sphere.color.g = 0.5f; sphere.color.b = 1.0f; sphere.color.a = 0.6f;
        markers->markers.push_back(sphere);
    }

    if (!markers->markers.empty())
        pub_plan_markers_->publish(std::move(markers));
}

bool RRTPathPlanner::isFrontierValid(const Eigen::Vector3d &f) {
    if (f[2] < min_frontier_z_) return false;
    if (f[0] >= cave_entrance_[0]) return false;
    return true;
}

void RRTPathPlanner::publishPlanningResult(bool success) {
    std_msgs::msg::Bool msg;
    msg.data = success;
    pub_planning_result_->publish(msg);
}

// =============================================================================
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RRTPathPlanner>());
    rclcpp::shutdown();
    return 0;
}