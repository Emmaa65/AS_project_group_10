#include <cmath>
#include <memory>
#include <string>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

// PCL Headers
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

// OctoMap Headers
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

#include <Eigen/Dense>
#include <mutex>
#include <chrono>
#include <vector>


class FrontierExploration : public rclcpp::Node {
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    using PointCloudPtr = PointCloud::Ptr;

    FrontierExploration()
        : Node("frontier_exploration_node"),
          octree_(std::make_unique<octomap::OcTree>(0.1))
    {
        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_full", 10,
            std::bind(&FrontierExploration::octomapCallback, this, std::placeholders::_1));

        current_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "current_state_est", 10,
            std::bind(&FrontierExploration::currentStateCallback, this, std::placeholders::_1));

        // Check goal progress at 2 Hz
        exploration_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.5),
            std::bind(&FrontierExploration::explorationTimerCallback, this));

        frontier_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/exploration/frontier_goal", 10);
    }

private:
    // -------------------------------------------------------------------------
    // ROS interfaces
    // -------------------------------------------------------------------------
    std::unique_ptr<octomap::OcTree>      octree_;
    std::unique_ptr<octomap::ColorOcTree> color_octree_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr   octomap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      current_state_sub_;
    rclcpp::TimerBase::SharedPtr                                   exploration_timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  frontier_goal_pub_;

    std::mutex mutex_;
    std::mutex drone_state_mutex_;

    // -------------------------------------------------------------------------
    // Drone state
    // -------------------------------------------------------------------------
    geometry_msgs::msg::Pose  current_pose_;
    geometry_msgs::msg::Twist current_velocity_;

    // -------------------------------------------------------------------------
    // Goal state machine
    //
    //  IDLE   → no committed goal; we pick a new frontier every timer tick
    //  FLYING → committed to a goal; we do NOT pick a new frontier until:
    //             (a) drone is within GOAL_REACHED_RADIUS, OR
    //             (b) no progress for PROGRESS_TIMEOUT_S, OR
    //             (c) total flight time exceeds STUCK_TIMEOUT_S
    // -------------------------------------------------------------------------
    enum class GoalState { IDLE, FLYING };
    GoalState goal_state_ = GoalState::IDLE;

    Eigen::Vector3d committed_goal_{0.0, 0.0, 0.0};

    // Tuning knobs
    const double GOAL_REACHED_RADIUS  = 10.0;  // metres — "close enough"
    const double PROGRESS_DISTANCE    = 3.0;   // metres — minimum movement to count as progress
    const double PROGRESS_TIMEOUT_S   = 20.0;  // seconds without progress → stuck
    const double STUCK_TIMEOUT_S      = 90.0;  // hard upper limit per goal

    rclcpp::Time    goal_committed_time_;
    Eigen::Vector3d last_progress_pos_{0.0, 0.0, 0.0};
    rclcpp::Time    last_progress_time_;

    // -------------------------------------------------------------------------
    // Map / clustering parameters
    // -------------------------------------------------------------------------
    double cluster_tolerance_ = 4.0;
    int    min_cluster_size_  = 15;
    int    max_cluster_size_  = 10000;

    double max_frontier_x_            = -330.0;
    double min_frontier_z_            = -33.5;
    double safety_margin_             = 2.0;
    const double MIN_Z_CLEARANCE      = 5.0;   // extra height above cave floor

    // -------------------------------------------------------------------------
    // Visited frontier history (for revisit penalty at selection time)
    // -------------------------------------------------------------------------
    struct VisitedFrontier {
        Eigen::Vector3d position;
        rclcpp::Time    timestamp;
    };
    std::vector<VisitedFrontier> visited_frontiers_;
    const double REVISIT_RADIUS      = 8.0;
    const size_t MAX_VISITED         = 20;
    const double REVISIT_DECAY_S     = 60.0;
    const double MAX_REVISIT_PENALTY = 80.0;

    // =========================================================================
    // Callbacks
    // =========================================================================

    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!msg) return;
        octomap::AbstractOcTree *tree = octomap_msgs::msgToMap(*msg);
        if (!tree) return;

        if (auto *ct = dynamic_cast<octomap::ColorOcTree*>(tree)) {
            color_octree_.reset(ct);
            octree_.reset();
            return;
        }
        if (auto *ot = dynamic_cast<octomap::OcTree*>(tree)) {
            octree_.reset(ot);
            color_octree_.reset();
            return;
        }
        delete tree;
    }

    void currentStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(drone_state_mutex_);
        current_pose_     = msg->pose.pose;
        current_velocity_ = msg->twist.twist;
    }

    // =========================================================================
    // Main timer callback — the heart of the state machine
    // =========================================================================
    void explorationTimerCallback() {
        std::lock_guard<std::mutex> lock(mutex_);

        Eigen::Vector3d drone_pos = getDronePosition();

        // ------------------------------------------------------------------
        // FLYING state: monitor progress, wait for arrival
        // ------------------------------------------------------------------
        if (goal_state_ == GoalState::FLYING) {
            double dist         = (drone_pos - committed_goal_).norm();
            double secs_flying  = (this->now() - goal_committed_time_).seconds();

            // (a) Goal reached
            if (dist < GOAL_REACHED_RADIUS) {
                RCLCPP_INFO(this->get_logger(),
                    "[STATE] Goal REACHED (dist=%.2f m after %.1f s) — going IDLE",
                    dist, secs_flying);
                recordVisited(committed_goal_);
                goal_state_ = GoalState::IDLE;
                return; // pick next frontier on next tick
            }

            // (b) Progress check
            double moved = (drone_pos - last_progress_pos_).norm();
            if (moved > PROGRESS_DISTANCE) {
                last_progress_pos_  = drone_pos;
                last_progress_time_ = this->now();
            }
            double secs_no_progress = (this->now() - last_progress_time_).seconds();

            // (c) Stuck / hard timeout
            if (secs_no_progress > PROGRESS_TIMEOUT_S || secs_flying > STUCK_TIMEOUT_S) {
                RCLCPP_WARN(this->get_logger(),
                    "[STATE] STUCK — no_progress=%.1fs, flying=%.1fs, dist=%.2f m — going IDLE",
                    secs_no_progress, secs_flying, dist);
                recordVisited(committed_goal_);
                goal_state_ = GoalState::IDLE;
                // fall through to IDLE branch immediately
            } else {
                // Still flying fine — do nothing this tick
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                    "[FLYING] target=(%.2f,%.2f,%.2f) dist=%.2f m | "
                    "flying=%.1fs no_progress=%.1fs",
                    committed_goal_.x(), committed_goal_.y(), committed_goal_.z(),
                    dist, secs_flying, secs_no_progress);
                return;
            }
        }

        // ------------------------------------------------------------------
        // IDLE state: pick the best frontier and commit
        // ------------------------------------------------------------------
        RCLCPP_INFO(this->get_logger(), "[STATE] IDLE — selecting next frontier...");

        PointCloudPtr frontier_cloud(new PointCloud);
        collectFrontierPoints(frontier_cloud);
        if (frontier_cloud->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "No frontier points found, retrying next tick");
            return;
        }

        auto clusters = clusterFrontiers(frontier_cloud);
        if (clusters.empty()) {
            RCLCPP_WARN(this->get_logger(), "No clusters found, retrying next tick");
            return;
        }

        std::vector<bool> safe;
        safe.reserve(clusters.size());
        for (const auto &c : clusters)
            safe.push_back(isSafePosition(computeClusterCentroid(frontier_cloud, c)));

        size_t best    = findBestCluster(clusters, frontier_cloud, safe);
        Eigen::Vector3d goal = computeClusterCentroid(frontier_cloud, clusters[best]);

        // Commit — lock in the goal, switch to FLYING
        committed_goal_      = goal;
        goal_state_          = GoalState::FLYING;
        goal_committed_time_ = this->now();
        last_progress_pos_   = drone_pos;
        last_progress_time_  = this->now();

        publishGoal(goal, clusters[best].indices.size(), best);
    }

    // =========================================================================
    // Frontier collection
    // =========================================================================
    void collectFrontierPoints(PointCloudPtr &cloud) {
        cloud->clear();
        const std::array<std::array<int,3>,6> offsets = {{
            {{1,0,0}},{{-1,0,0}},{{0,1,0}},{{0,-1,0}},{{0,0,1}},{{0,0,-1}}}};

        auto tryAdd = [&](double x, double y, double z) {
            if (x >= max_frontier_x_) return;
            if (z < min_frontier_z_ + MIN_Z_CLEARANCE) return;
            pcl::PointXYZ p;
            p.x = static_cast<float>(x);
            p.y = static_cast<float>(y);
            p.z = static_cast<float>(z);
            cloud->points.push_back(p);
        };

        if (color_octree_) {
            double res = color_octree_->getResolution();
            for (auto it = color_octree_->begin_leafs(); it != color_octree_->end_leafs(); ++it) {
                if (color_octree_->isNodeOccupied(*it)) continue;
                double x = it.getX(), y = it.getY(), z = it.getZ();
                for (const auto &o : offsets) {
                    if (!color_octree_->search(x+o[0]*res, y+o[1]*res, z+o[2]*res)) {
                        tryAdd(x, y, z); break;
                    }
                }
            }
        } else if (octree_) {
            double res = octree_->getResolution();
            for (auto it = octree_->begin_leafs(); it != octree_->end_leafs(); ++it) {
                if (octree_->isNodeOccupied(*it)) continue;
                double x = it.getX(), y = it.getY(), z = it.getZ();
                for (const auto &o : offsets) {
                    if (!octree_->search(x+o[0]*res, y+o[1]*res, z+o[2]*res)) {
                        tryAdd(x, y, z); break;
                    }
                }
            }
        }
        RCLCPP_INFO(this->get_logger(),
            "collectFrontierPoints: %zu points", cloud->points.size());
    }

    // =========================================================================
    // Clustering
    // =========================================================================
    std::vector<pcl::PointIndices> clusterFrontiers(const PointCloudPtr &cloud) {
        std::vector<pcl::PointIndices> indices;
        if (cloud->points.empty()) return indices;

        auto kd = std::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
        kd->setInputCloud(cloud);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(kd);
        ec.setInputCloud(cloud);
        ec.extract(indices);
        return indices;
    }

    // =========================================================================
    // Scoring  (called only when IDLE — no ping-pong possible)
    // =========================================================================
    size_t findBestCluster(const std::vector<pcl::PointIndices> &clusters,
                           const PointCloudPtr &cloud,
                           const std::vector<bool> & /*safe*/) {
        Eigen::Vector3d drone_pos = getDronePosition();

        std::vector<Eigen::Vector3d> centroids(clusters.size());
        std::vector<double>          dists(clusters.size());
        for (size_t i = 0; i < clusters.size(); ++i) {
            centroids[i] = computeClusterCentroid(cloud, clusters[i]);
            dists[i]     = (centroids[i] - drone_pos).norm();
        }

        size_t best       = 0;
        double best_score = -std::numeric_limits<double>::infinity();
        const double entrance_x = -350.0;
        const double min_dist   = 5.0;

        rclcpp::Time now = this->now();

        for (size_t i = 0; i < clusters.size(); ++i) {
            if (dists[i] < min_dist) continue;

            // --- Cluster size: more frontier points = more unknown area ---
            double size_score = static_cast<double>(clusters[i].indices.size());

            // --- Proximity bonus: prefer closer clusters to minimise travel time.
            //     Use inverse distance so very close clusters score highest.
            //     Clamped to avoid division by zero. ---
            double proximity_bonus = 500.0 / std::max(dists[i], 1.0);

            // --- Novelty bonus: how far is this cluster from ALL visited frontiers?
            //     A cluster surrounded by unvisited space gets a high bonus.
            //     We take the distance to the NEAREST visited frontier as the novelty
            //     signal — far from any visited point means genuinely new territory. ---
            double min_visited_dist = std::numeric_limits<double>::max();
            for (const auto &v : visited_frontiers_) {
                double d = (centroids[i] - v.position).norm();
                min_visited_dist = std::min(min_visited_dist, d);
            }
            // If no visited frontiers yet, treat as maximally novel
            if (visited_frontiers_.empty()) min_visited_dist = 200.0;
            // Scale: 5 pts per metre away from the nearest visited frontier,
            // capped so a single very-distant cluster can't dominate everything
            double novelty_bonus = std::min(min_visited_dist * 5.0, 400.0);

            // --- Entrance penalty: keep drone inside the cave ---
            double entrance_pen = (centroids[i].x() > entrance_x)
                ? (centroids[i].x() - entrance_x) * 100.0 : 0.0;

            // --- Time-decaying revisit penalty (immediate re-selection guard) ---
            double revisit_pen = 0.0;
            for (const auto &v : visited_frontiers_) {
                double d = (centroids[i] - v.position).norm();
                if (d < REVISIT_RADIUS) {
                    double age   = (now - v.timestamp).seconds();
                    double decay = std::max(0.0, 1.0 - age / REVISIT_DECAY_S);
                    revisit_pen += (REVISIT_RADIUS - d) * 15.0 * decay;
                }
            }
            revisit_pen = std::min(revisit_pen, MAX_REVISIT_PENALTY);

            double score = size_score + proximity_bonus + novelty_bonus
                         - entrance_pen - revisit_pen;

            RCLCPP_INFO(this->get_logger(),
                "  Cluster %zu: X=%.2f size=%zu dist=%.2f min_visited=%.1f | "
                "size=%.0f prox=%.1f novelty=%.1f entrance=%.1f revisit=%.1f = %.1f",
                i, centroids[i].x(), clusters[i].indices.size(), dists[i],
                min_visited_dist == 200.0 ? -1.0 : min_visited_dist,
                size_score, proximity_bonus, novelty_bonus,
                entrance_pen, revisit_pen, score);

            if (score > best_score) { best_score = score; best = i; }
        }
        return best;
    }

    // =========================================================================
    // Helpers
    // =========================================================================
    Eigen::Vector3d getDronePosition() {
        std::lock_guard<std::mutex> lock(drone_state_mutex_);
        return {current_pose_.position.x,
                current_pose_.position.y,
                current_pose_.position.z};
    }

    Eigen::Vector3d computeClusterCentroid(const PointCloudPtr &cloud,
                                           const pcl::PointIndices &idx) {
        Eigen::Vector3d c(0, 0, 0);
        if (idx.indices.empty()) return c;
        for (int i : idx.indices) {
            c.x() += cloud->points[i].x;
            c.y() += cloud->points[i].y;
            c.z() += cloud->points[i].z;
        }
        return c / static_cast<double>(idx.indices.size());
    }

    bool isSafePosition(const Eigen::Vector3d &pos) {
        const double r = safety_margin_, res = 1.0;
        for (double dx = -r; dx <= r; dx += res)
        for (double dy = -r; dy <= r; dy += res)
        for (double dz = -r; dz <= r; dz += res) {
            if (std::sqrt(dx*dx+dy*dy+dz*dz) > r) continue;
            bool occ = false;
            if (color_octree_) {
                auto n = color_octree_->search(pos.x()+dx, pos.y()+dy, pos.z()+dz);
                occ = n && color_octree_->isNodeOccupied(n);
            } else if (octree_) {
                auto n = octree_->search(pos.x()+dx, pos.y()+dy, pos.z()+dz);
                occ = n && octree_->isNodeOccupied(n);
            }
            if (occ) return false;
        }
        return true;
    }

    void recordVisited(const Eigen::Vector3d &pos) {
        visited_frontiers_.push_back({pos, this->now()});
        if (visited_frontiers_.size() > MAX_VISITED)
            visited_frontiers_.erase(visited_frontiers_.begin());
    }

    void publishGoal(const Eigen::Vector3d &goal, size_t size, size_t idx) {
        Eigen::Vector3d drone_pos = getDronePosition();
        double yaw      = std::atan2((goal - drone_pos).y(), (goal - drone_pos).x()) + M_PI;
        double half_yaw = yaw / 2.0;

        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp    = this->now();
        msg.header.frame_id = "map";
        msg.pose.position.x = goal.x();
        msg.pose.position.y = goal.y();
        msg.pose.position.z = goal.z();
        msg.pose.orientation.z = std::sin(half_yaw);
        msg.pose.orientation.w = std::cos(half_yaw);
        frontier_goal_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
            "[COMMITTED] cluster %zu at (%.2f, %.2f, %.2f) dist=%.2f m size=%zu",
            idx, goal.x(), goal.y(), goal.z(),
            (goal - drone_pos).norm(), size);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierExploration>());
    rclcpp::shutdown();
    return 0;
}