#include <cmath>
#include <memory>
#include <string>
#include <algorithm>
#include <array>
#include <deque>
#include <limits>

#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/bool.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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

    FrontierExploration() : Node("frontier_exploration_node"), octree_(std::make_unique<octomap::OcTree>(0.1)) {
        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_full", 10, 
            std::bind(&FrontierExploration::octomapCallback, this, std::placeholders::_1)
        );

        current_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "current_state_est", 10, std::bind(&FrontierExploration::currentStateCallback, this, std::placeholders::_1));

        frontier_request_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/exploration/request_frontier", 10,
            std::bind(&FrontierExploration::frontierRequestCallback, this, std::placeholders::_1)
        );
        
        accepted_frontier_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "accepted_frontier_position", 10,
            std::bind(&FrontierExploration::acceptedFrontierCallback, this, std::placeholders::_1)
        );
        
        // Timer Callback for Periodic Exploration (2 Hz)
        exploration_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.5), std::bind(&FrontierExploration::explorationTimerCallback, this)
        );

        // Frontier Goal Publisher
        frontier_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/exploration/frontier_goal", 10
        );

        frontier_goal_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/exploration/frontier_goal_history_markers", 10
        );


    }
    
private:
    // Member variables
    std::unique_ptr<octomap::OcTree> octree_;
    std::unique_ptr<octomap::ColorOcTree> color_octree_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr frontier_request_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr accepted_frontier_sub_;
    rclcpp::TimerBase::SharedPtr exploration_timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr frontier_goal_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_goal_markers_pub_;
    std::mutex mutex_;
    // Drone state
    std::mutex drone_state_mutex_;
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Twist current_velocity_;
    // Clustering parameters
    double cluster_tolerance_ = 3.0; // meters - distance threshold for points to belong to same cluster
    int min_cluster_size_ = 50;  // Ignore small clusters (noise/small gaps)
    int max_cluster_size_ = 10000;
    // Cave entrance filter (only consider frontiers inside cave)
    double max_frontier_x_ = -330.0; // Cave extends only in X < -330
    //double min_frontier_z_ = -33.5; // Minimum safe height
    double safety_margin_ = 0.5; // Minimum distance from obstacles (meters) - drone is 0.2x0.2m
    bool frontier_request_pending_ = false;
    std::deque<geometry_msgs::msg::Point> frontier_goal_history_;
    static constexpr size_t max_frontier_history_ = 5;
    Eigen::Vector3d last_published_frontier_ = Eigen::Vector3d(1e6, 1e6, 1e6); // Track last published to avoid near-duplicate re-suggestions
    std::vector<Eigen::Vector3d> accepted_frontier_positions_; // History of ACCEPTED (non-failed) frontier positions for continuity scoring

    // Callbacks (lightweight stubs to allow compilation)
    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!msg) {
            RCLCPP_WARN(this->get_logger(), "octomapCallback: received null message");
            return;
        }

        // Convert ROS message to octomap AbstractOcTree (caller allocates)
        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        if (!tree) {
            RCLCPP_WARN(this->get_logger(), "octomapCallback: msgToMap failed");
            return;
        }

        // If the incoming tree contains color information, store as ColorOcTree
        if (auto *ct = dynamic_cast<octomap::ColorOcTree*>(tree)) {
            color_octree_.reset(ct); // take ownership
            octree_.reset();
            RCLCPP_INFO(this->get_logger(), "octomapCallback: stored ColorOcTree (res=%.3f)", ct->getResolution());
            return;
        }

        // Otherwise, check for a regular OcTree
        if (auto *ot = dynamic_cast<octomap::OcTree*>(tree)) {
            octree_.reset(ot); // take ownership
            color_octree_.reset();
            RCLCPP_INFO(this->get_logger(), "octomapCallback: stored OcTree (res=%.3f)", ot->getResolution());
            return;
        }

        // Unknown tree type: avoid leak
        delete tree;
        RCLCPP_WARN(this->get_logger(), "octomapCallback: received unsupported octree type");
    }

    void currentStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(drone_state_mutex_);
        current_pose_ = msg->pose.pose;
        current_velocity_ = msg->twist.twist;
    }

    void acceptedFrontierCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // Receive accepted frontier positions from exploration_manager
        // This populates the history for continuity-based scoring
        if (!msg) return;
        std::lock_guard<std::mutex> lock(mutex_);
        
        Eigen::Vector3d accepted_pos(
            msg->pose.position.x,
            msg->pose.position.y,
            msg->pose.position.z
        );

        // Ignore near-duplicate accepted frontiers (e.g., replans to same goal)
        // to avoid zero-length direction vectors in continuity scoring.
        if (!accepted_frontier_positions_.empty()) {
            const double duplicate_distance =
                (accepted_pos - accepted_frontier_positions_.back()).norm();
            if (duplicate_distance < 1.0) {
                RCLCPP_INFO(this->get_logger(),
                    "Accepted frontier [%.2f, %.2f, %.2f] ignored as duplicate (%.2f m from last), history size: %zu",
                    accepted_pos.x(), accepted_pos.y(), accepted_pos.z(),
                    duplicate_distance, accepted_frontier_positions_.size());
                return;
            }
        }
        
        // Add to history (keep last N positions for continuity calculation)
        accepted_frontier_positions_.push_back(accepted_pos);
        
        // Limit history size (keep last 10 accepted frontiers)
        const size_t max_history = 10;
        if (accepted_frontier_positions_.size() > max_history) {
            accepted_frontier_positions_.erase(accepted_frontier_positions_.begin());
        }
        
        RCLCPP_INFO(this->get_logger(),
            "Received accepted frontier [%.2f, %.2f, %.2f] - history size: %zu",
            accepted_pos.x(), accepted_pos.y(), accepted_pos.z(), 
            accepted_frontier_positions_.size());
    }

    void frontierRequestCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (!msg || !msg->data) {
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        frontier_request_pending_ = true;
        RCLCPP_INFO(this->get_logger(), "Received frontier request from exploration_manager");
    }

    void explorationTimerCallback() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!frontier_goal_pub_) return;
        if (!frontier_request_pending_) return;

        // Collect frontier points
        PointCloudPtr frontier_cloud(new PointCloud);
        collectFrontierPoints(frontier_cloud);
        RCLCPP_INFO(this->get_logger(), "explorationTimerCallback: frontier_points=%zu", frontier_cloud->points.size());
        if (frontier_cloud->points.empty()) {
            RCLCPP_DEBUG(this->get_logger(), "explorationTimerCallback: no frontier points detected");
            return;
        }

        // Cluster frontiers
        auto cluster_indices = clusterFrontiers(frontier_cloud);
        RCLCPP_INFO(this->get_logger(), "explorationTimerCallback: clusters=%zu", cluster_indices.size());
        if (cluster_indices.size()==0) {
            RCLCPP_DEBUG(this->get_logger(), "explorationTimerCallback: no clusters found");
            return;
        }

    // Select best cluster: prefer clusters deeper in the cave (smaller X)
    // with a size bonus to avoid tiny outliers
    size_t best_idx = findBestCluster(cluster_indices, frontier_cloud);

    // Compute centroid (XY average of frontier cluster)
    Eigen::Vector3d centroid = computeClusterCentroid(frontier_cloud, cluster_indices[best_idx]);
    
    // Use centroid directly without Z adjustment
    // Z-adjustment to "safe defaults" can create unreachable target positions
    RCLCPP_INFO(this->get_logger(), 
        "Selected cluster %zu (size=%zu, X=%.2f): centroid: (%.2f, %.2f, %.2f)",
        best_idx, cluster_indices[best_idx].indices.size(), centroid.x(),
        centroid.x(), centroid.y(), centroid.z());
    
    publishGoalFromCentroid(centroid, cluster_indices[best_idx].indices.size());
    frontier_request_pending_ = false;
    }


    void collectFrontierPoints(PointCloudPtr &frontier_cloud) {
        frontier_cloud->clear();
        std::array<std::array<int,3>,6> offsets = {{{{1,0,0}},{{-1,0,0}},{{0,1,0}},{{0,-1,0}},{{0,0,1}},{{0,0,-1}}}};
        RCLCPP_INFO(this->get_logger(), "collectFrontierPoints: start (has_color=%s has_plain=%s)",
                    color_octree_?"true":"false", octree_?"true":"false");

        if (color_octree_) {
            auto &ct = *color_octree_;
            double res = ct.getResolution();
            size_t checked = 0;
            size_t found = 0;
            for (auto it = ct.begin_leafs(); it != ct.end_leafs(); ++it) {
                ++checked;
                if (ct.isNodeOccupied(*it)) continue;
                double x = it.getX();
                double y = it.getY();
                double z = it.getZ();

                bool is_frontier = false;
                for (const auto &off : offsets) {
                    double nx = x + off[0] * res;
                    double ny = y + off[1] * res;
                    double nz = z + off[2] * res;
                    auto node = ct.search(nx, ny, nz);
                    if (!node) { is_frontier = true; break; }
                    if (ct.isNodeOccupied(node)) continue;//{ is_frontier = true; break; }
                }
                if (is_frontier) {
                    // Filter: Only accept frontiers inside cave (X < -330)
                    if (x < max_frontier_x_) {
                        pcl::PointXYZ p;
                        p.x = static_cast<float>(x);
                        p.y = static_cast<float>(y);
                        p.z = static_cast<float>(z);
                        frontier_cloud->points.push_back(p);
                        ++found;
                    }
                }
            }
            RCLCPP_INFO(this->get_logger(), "collectFrontierPoints: color_octree checked=%zu frontier_points=%zu", checked, found);
        } else if (octree_) {
            auto &ot = *octree_;
            double res = ot.getResolution();
            size_t checked = 0;
            size_t found = 0;
            for (auto it = ot.begin_leafs(); it != ot.end_leafs(); ++it) {
                ++checked;
                if (ot.isNodeOccupied(*it)) continue;
                double x = it.getX();
                double y = it.getY();
                double z = it.getZ();

                bool is_frontier = false;
                for (const auto &off : offsets) {
                    double nx = x + off[0] * res;
                    double ny = y + off[1] * res;
                    double nz = z + off[2] * res;
                    auto node = ot.search(nx, ny, nz);
                    if (!node) { is_frontier = true; break; }
                    if (ot.isNodeOccupied(node)) continue;//{ is_frontier = true; break; }
                }
                if (is_frontier) {
                    // Filter: Only accept frontiers inside cave (X < -330)
                    if (x < max_frontier_x_) {
                        pcl::PointXYZ p;
                        p.x = static_cast<float>(x);
                        p.y = static_cast<float>(y);
                        p.z = static_cast<float>(z);
                        frontier_cloud->points.push_back(p);
                        ++found;
                    }
                }
            }
            RCLCPP_INFO(this->get_logger(), "collectFrontierPoints: oc_tree checked=%zu frontier_points=%zu", checked, found);
        }
    }

    std::vector<pcl::PointIndices> clusterFrontiers(const PointCloudPtr &frontier_cloud) {
        std::vector<pcl::PointIndices> cluster_indices;
        if (frontier_cloud->points.empty()) return cluster_indices;

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_kd(new pcl::search::KdTree<pcl::PointXYZ>);
        tree_kd->setInputCloud(frontier_cloud);

        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(cluster_tolerance_);
        ec.setMinClusterSize(min_cluster_size_);
        ec.setMaxClusterSize(max_cluster_size_);
        ec.setSearchMethod(tree_kd);
        ec.setInputCloud(frontier_cloud);
        ec.extract(cluster_indices);
        return cluster_indices;
    }

    size_t findLargestCluster(const std::vector<pcl::PointIndices> &clusters) {
        size_t largest_idx = 0;
        size_t largest_size = 0;
        for (size_t i = 0; i < clusters.size(); ++i) {
            if (clusters[i].indices.size() > largest_size) {
                largest_size = clusters[i].indices.size();
                largest_idx = i;
            }
        }
        return largest_idx;
    }

    // Find best cluster for exploration: prefer clusters deeper in cave (smaller X)
    // with bonus for larger clusters to avoid tiny outliers
    size_t findBestCluster(const std::vector<pcl::PointIndices> &clusters,
                       const PointCloudPtr &cloud) {
        if (clusters.empty()) return 0;

        Eigen::Vector3d drone_pos;
        {
            std::lock_guard<std::mutex> lock(drone_state_mutex_);
            drone_pos = {current_pose_.position.x,
                         current_pose_.position.y,
                         current_pose_.position.z};
        }

        // ── 1. Rohdaten sammeln ──────────────────────────────────────────────────
        struct ClusterFeatures {
            size_t  idx;
            Eigen::Vector3d centroid;
            double  distance;      // drone → centroid
            double  size;          // number of frontier points
            double  z_depth;       // -centroid.z()  (höher = weiter unten)
            double  continuity;    // cos-Winkel [-1, +1]
            double  entrance_dist; // Abstand zum Eingang
            bool    valid;
        };

        const double MIN_DIST          = 5.0;
        const double ENTRANCE_X        = -330.0;
        const double ENTRANCE_Y        = 10.0;
        const double ENTRANCE_Z        = 20.0;
        const double ENTRANCE_DISC_R   = 12.0;

        std::vector<ClusterFeatures> feats;
        feats.reserve(clusters.size());

        // Continuity-Richtung einmal berechnen
        Eigen::Vector3d continuity_dir = Eigen::Vector3d::Zero();
        bool has_continuity = false;
        if (accepted_frontier_positions_.size() >= 2) {
            continuity_dir =
                accepted_frontier_positions_.back() -
                accepted_frontier_positions_[accepted_frontier_positions_.size() - 2];
            has_continuity = continuity_dir.norm() > 0.1;
        } else if (accepted_frontier_positions_.size() == 1) {
            continuity_dir =
                drone_pos - accepted_frontier_positions_.back();
            has_continuity = continuity_dir.norm() > 0.1;
        }
        if (has_continuity) continuity_dir.normalize();

        for (size_t i = 0; i < clusters.size(); ++i) {
            ClusterFeatures f;
            f.idx      = i;
            f.centroid = computeClusterCentroid(cloud, clusters[i]);
            f.distance = (f.centroid - drone_pos).norm();
            f.size     = static_cast<double>(clusters[i].indices.size());
            f.z_depth  = -f.centroid.z();
            f.valid    = true;

            // Hard-Filter
            if (f.distance < MIN_DIST) {
                RCLCPP_INFO(get_logger(), "Cluster %zu SKIP: too close (%.2f m)", i, f.distance);
                f.valid = false;
            }
            if (!isSafePosition(f.centroid)) {
                RCLCPP_INFO(get_logger(), "Cluster %zu SKIP: unsafe position", i);
                f.valid = false;
            }
            if ((f.centroid - last_published_frontier_).norm() < 3.0) {
                RCLCPP_INFO(get_logger(), "Cluster %zu SKIP: near last published", i);
                f.valid = false;
            }

            // Continuity [-1, +1]
            if (has_continuity) {
                Eigen::Vector3d to_cluster = f.centroid - accepted_frontier_positions_.back();
                f.continuity = (to_cluster.norm() > 0.1)
                    ? continuity_dir.dot(to_cluster.normalized())
                    : 0.0;
            } else {
                f.continuity = 0.0;
            }

            // Eingangs-Abstand (Disc in YZ-Ebene, Strafe wenn nahe am Eingang)
            double dist_yz = std::hypot(f.centroid.y() - ENTRANCE_Y,
                                        f.centroid.z() - ENTRANCE_Z);
            f.entrance_dist = (f.centroid.x() > ENTRANCE_X + 2.0 && dist_yz < ENTRANCE_DISC_R)
                              ? 0.0   // nah am Eingang → schlechtester Wert
                              : 1.0;  // tief in Höhle  → bester Wert

            feats.push_back(f);
        }

        // Prüfe ob valide Cluster vorhanden
        bool any_valid = std::any_of(feats.begin(), feats.end(), [](const auto &f){ return f.valid; });
        if (!any_valid) {
            RCLCPP_WARN(get_logger(), "No valid clusters – fallback: closest cluster");
            // Fallback: nächsten Cluster nehmen (kein Hard-Filter)
            return std::min_element(feats.begin(), feats.end(),
                [](const auto &a, const auto &b){ return a.distance < b.distance; }) - feats.begin();
        }

        // ── 2. Min/Max über valide Cluster für Normierung ───────────────────────
        auto valid_range = [&](auto getter) -> std::pair<double,double> {
            double lo =  std::numeric_limits<double>::max();
            double hi = -std::numeric_limits<double>::max();
            for (const auto &f : feats) {
                if (!f.valid) continue;
                double v = getter(f);
                lo = std::min(lo, v);
                hi = std::max(hi, v);
            }
            return {lo, hi};
        };

        // Normierungshilfe: gibt [0,1] zurück; 0 wenn Range trivial
        auto norm = [](double val, double lo, double hi) -> double {
            return (hi - lo) > 1e-6 ? (val - lo) / (hi - lo) : 0.5;
        };

        auto [size_lo,     size_hi    ] = valid_range([](const auto &f){ return f.size;     });
        auto [dist_lo,     dist_hi    ] = valid_range([](const auto &f){ return f.distance; });
        auto [zdepth_lo,   zdepth_hi  ] = valid_range([](const auto &f){ return f.z_depth;  });
        // continuity ist schon in [-1,+1] → auf [0,1] schieben: (c+1)/2
        // entrance_dist ist binär {0,1}

        // ── 3. Gewichte ─────────────────────────────────────────────────────────
        // Summe = 1.0  →  Score ∈ [0, 1]
        const double W_SIZE        = 0.10;  // Größe des Clusters
        const double W_Z_DEPTH     = 0.10;  // Vertikale Tiefe
        const double W_DISTANCE    = 0.35;  // Nähe (invertiert)
        const double W_CONTINUITY  = 0.20;  // Richtungstreue
        const double W_ENTRANCE    = 0.25;  // Eingangs-Penalty
        // Σ = 1.00

        // ── 4. Scoring ──────────────────────────────────────────────────────────
        size_t best_idx   = 0;
        double best_score = -1.0;

        for (auto &f : feats) {
            if (!f.valid) continue;

            // Größe: log-skaliert damit riesige Cluster nicht dominieren
            double s_size      = norm(std::log1p(f.size), std::log1p(size_lo), std::log1p(size_hi));
            double s_z_depth   = norm(f.z_depth,  zdepth_lo,  zdepth_hi);
            double s_distance  = 1.0 - norm(f.distance, dist_lo, dist_hi); // kleiner Abstand = besser
            double s_continuity= (f.continuity + 1.0) / 2.0;               // [-1,1] → [0,1]
            double s_entrance  = f.entrance_dist;                           // binär {0,1}

            double score =
                W_SIZE       * s_size      +
                W_Z_DEPTH    * s_z_depth   +
                W_DISTANCE   * s_distance  +
                W_CONTINUITY * s_continuity+
                W_ENTRANCE   * s_entrance;

            RCLCPP_INFO(get_logger(),
                "Cluster %zu | size=%.2f z=%.2f dist=%.2f cont=%.2f entr=%.2f → score=%.4f",
                f.idx, s_size, s_z_depth, s_distance, s_continuity, s_entrance, score);

            if (score > best_score) {
                best_score = score;
                best_idx   = f.idx;
            }
        }

        RCLCPP_INFO(get_logger(), ">>> SELECTED cluster %zu (score=%.4f)", best_idx, best_score);
        return best_idx;
    }

    Eigen::Vector3d computeClusterCentroid(const PointCloudPtr &cloud, const pcl::PointIndices &indices) {
        Eigen::Vector3d centroid(0,0,0);
        if (indices.indices.empty()) return centroid;
        for (int idx : indices.indices) {
            const auto &pt = cloud->points[idx];
            centroid.x() += pt.x;
            centroid.y() += pt.y;
            centroid.z() += pt.z;
        }
        centroid /= static_cast<double>(indices.indices.size());
        return centroid;
    }

    // Check if a point is safe (at least safety_margin away from all occupied voxels)
    bool isSafePosition(const Eigen::Vector3d &pos) {
        const double check_radius = safety_margin_;
        const double res = 1.0; // Check resolution (1m steps)
        
        // Check sphere around position for obstacles
        for (double dx = -check_radius; dx <= check_radius; dx += res) {
            for (double dy = -check_radius; dy <= check_radius; dy += res) {
                for (double dz = -check_radius; dz <= check_radius; dz += res) {
                    double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
                    if (dist > check_radius) continue; // Outside sphere
                    
                    double check_x = pos.x() + dx;
                    double check_y = pos.y() + dy;
                    double check_z = pos.z() + dz;
                    
                    // Check if this voxel is occupied
                    bool occupied = false;
                    if (color_octree_) {
                        auto node = color_octree_->search(check_x, check_y, check_z);
                        occupied = (node && color_octree_->isNodeOccupied(node));
                    } else if (octree_) {
                        auto node = octree_->search(check_x, check_y, check_z);
                        occupied = (node && octree_->isNodeOccupied(node));
                    }
                    
                    if (occupied) {
                        RCLCPP_INFO(this->get_logger(), 
                            "isSafePosition: UNSAFE at (%.2f, %.2f, %.2f) - obstacle at (%.2f, %.2f, %.2f)",
                            pos.x(), pos.y(), pos.z(), check_x, check_y, check_z);
                        // Found obstacle within safety margin
                        return false;
                    }
                }
            }
        }
        return true; // No obstacles found within safety margin
    }

    void publishGoalFromCentroid(const Eigen::Vector3d &centroid, size_t cluster_size) {
        if (!frontier_goal_pub_) return;
        
        // Get current drone position
        Eigen::Vector3d drone_pos;
        {
            std::lock_guard<std::mutex> lock(drone_state_mutex_);
            drone_pos.x() = current_pose_.position.x;
            drone_pos.y() = current_pose_.position.y;
            drone_pos.z() = current_pose_.position.z;
        }
        
        // Calculate direction vector from drone to goal
        Eigen::Vector3d direction = centroid - drone_pos;
        
        // Calculate yaw angle to point camera towards goal
        // Yaw = atan2(dy, dx) rotates around Z axis to point in XY plane direction
        double yaw = std::atan2(direction.y(), direction.x());
        
        // Add 180° offset to match camera/body orientation
        yaw += M_PI;
        
        // Convert yaw to quaternion (rotation around Z axis)
        // q = [cos(yaw/2), 0, 0, sin(yaw/2)]
        double half_yaw = yaw / 2.0;
        
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = this->now();
        goal.header.frame_id = "map";
        goal.pose.position.x = centroid.x();
        goal.pose.position.y = centroid.y();
        goal.pose.position.z = centroid.z();
        
        // Set orientation to point camera towards goal
        goal.pose.orientation.x = 0.0;
        goal.pose.orientation.y = 0.0;
        goal.pose.orientation.z = std::sin(half_yaw);
        goal.pose.orientation.w = std::cos(half_yaw);
        
        frontier_goal_pub_->publish(goal);
        publishFrontierHistoryMarkers(goal.pose.position);

        // Track this as the last published frontier to avoid re-suggesting if it gets rejected
        last_published_frontier_ = centroid;

        RCLCPP_INFO(this->get_logger(),
            "Continuity publish debug: published frontier with accepted history size=%zu",
            accepted_frontier_positions_.size());
        
        RCLCPP_INFO(this->get_logger(), 
            "Published frontier goal at (%.2f, %.2f, %.2f) with yaw=%.1f° from cluster size %zu", 
            centroid.x(), centroid.y(), centroid.z(), yaw * 180.0 / M_PI, cluster_size);
    }

    void publishFrontierHistoryMarkers(const geometry_msgs::msg::Point &new_goal_position) {
        if (!frontier_goal_markers_pub_) {
            return;
        }

        frontier_goal_history_.push_back(new_goal_position);
        if (frontier_goal_history_.size() > max_frontier_history_) {
            frontier_goal_history_.pop_front();
        }

        visualization_msgs::msg::MarkerArray marker_array;

        for (size_t index = 0; index < frontier_goal_history_.size(); ++index) {
            visualization_msgs::msg::Marker marker;
            marker.header.stamp = this->now();
            marker.header.frame_id = "world";
            marker.ns = "frontier_goal_history";
            marker.id = static_cast<int>(index);
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position = frontier_goal_history_[index];
            marker.pose.orientation.w = 1.0;
            const bool is_latest_marker = (index + 1 == frontier_goal_history_.size());
            const double marker_size = is_latest_marker ? 0.5 : 0.35;
            marker.scale.x = marker_size;
            marker.scale.y = marker_size;
            marker.scale.z = marker_size;
            marker.color.r = is_latest_marker ? 1.0f : 0.0f;
            marker.color.g = is_latest_marker ? 0.2f : 0.4f;
            marker.color.b = is_latest_marker ? 0.2f : 1.0f;
            marker.color.a = 0.95f;
            marker.lifetime = rclcpp::Duration::from_seconds(0.0);
            marker_array.markers.push_back(marker);
        }

        for (size_t index = frontier_goal_history_.size(); index < max_frontier_history_; ++index) {
            visualization_msgs::msg::Marker delete_marker;
            delete_marker.header.stamp = this->now();
            delete_marker.header.frame_id = "world";
            delete_marker.ns = "frontier_goal_history";
            delete_marker.id = static_cast<int>(index);
            delete_marker.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(delete_marker);
        }

        frontier_goal_markers_pub_->publish(marker_array);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrontierExploration>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}