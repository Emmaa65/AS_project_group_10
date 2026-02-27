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

    FrontierExploration() : Node("frontier_exploration_node"), octree_(std::make_unique<octomap::OcTree>(0.1)) {
        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            "/octomap_full", 10, 
            std::bind(&FrontierExploration::octomapCallback, this, std::placeholders::_1)
        );

        current_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "current_state", 10, std::bind(&FrontierExploration::currentStateCallback, this, std::placeholders::_1));
        
            // Timer Callback for Periodic Exploration (2 Hz)
        exploration_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.5), std::bind(&FrontierExploration::explorationTimerCallback, this)
        );

        // Frontier Goal Publisher
        frontier_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/exploration/frontier_goal", 10
        );


    }
    
private:
    // Member variables
    std::unique_ptr<octomap::OcTree> octree_;
    std::unique_ptr<octomap::ColorOcTree> color_octree_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_state_sub_;
    rclcpp::TimerBase::SharedPtr exploration_timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr frontier_goal_pub_;
    std::mutex mutex_;
    // Drone state
    std::mutex drone_state_mutex_;
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Twist current_velocity_;
    // Clustering parameters
    double cluster_tolerance_ = 5; // meters
    int min_cluster_size_ = 10;
    int max_cluster_size_ = 10000;
    // Cave entrance filter (only consider frontiers inside cave)
    double max_frontier_x_ = -330.0; // Cave extends only in X < -330
    double min_frontier_z_ = -33.5; // Minimum safe height
    double safety_margin_ = 2.0; // Minimum distance from obstacles (meters)

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

    void explorationTimerCallback() {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!frontier_goal_pub_) return;

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
    
    // Adjust Z to be in the middle of the cave at this XY position
    Eigen::Vector3d adjusted_centroid = adjustZToMidHeight(centroid);
    
    RCLCPP_INFO(this->get_logger(), 
        "Selected cluster %zu (size=%zu, X=%.2f): centroid: (%.2f, %.2f, %.2f) -> adjusted: (%.2f, %.2f, %.2f)",
        best_idx, cluster_indices[best_idx].indices.size(), centroid.x(),
        centroid.x(), centroid.y(), centroid.z(),
        adjusted_centroid.x(), adjusted_centroid.y(), adjusted_centroid.z());
    
    publishGoalFromCentroid(adjusted_centroid, cluster_indices[best_idx].indices.size());
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
                    // Filter: Only accept frontiers inside cave (X < -330) and above floor
                    if (x < max_frontier_x_ && z >= min_frontier_z_) {
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
                    // Filter: Only accept frontiers inside cave (X < -330) and above floor
                    if (x < max_frontier_x_ && z >= min_frontier_z_) {
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
        
        // Get current drone position
        Eigen::Vector3d drone_pos;
        {
            std::lock_guard<std::mutex> lock(drone_state_mutex_);
            drone_pos.x() = current_pose_.position.x;
            drone_pos.y() = current_pose_.position.y;
            drone_pos.z() = current_pose_.position.z;
        }
        
        size_t best_idx = 0;
        double best_score = -std::numeric_limits<double>::infinity();
        const double min_distance = 5.0;  // Minimum distance from drone (meters) - reduced for deep exploration
        
        // First pass: try to find clusters far enough from drone
        bool found_distant_cluster = false;
        const double entrance_x = -350.0;  // Cave entrance region (penalize return to entrance)
        
        for (size_t i = 0; i < clusters.size(); ++i) {
            // Compute cluster centroid
            Eigen::Vector3d centroid = computeClusterCentroid(cloud, clusters[i]);
            
            // Check distance to avoid getting stuck at same position
            double distance = (centroid - drone_pos).norm();
            if (distance < min_distance) {
                RCLCPP_INFO(this->get_logger(),
                    "Cluster %zu: X=%.2f, size=%zu, distance=%.2f - SKIPPED (too close to drone)",
                    i, centroid.x(), clusters[i].indices.size(), distance);
                continue;
            }
            
            // CRITICAL: Check if position is safe (5m from all obstacles)
            if (!isSafePosition(centroid)) {
                RCLCPP_INFO(this->get_logger(),
                    "Cluster %zu: X=%.2f, size=%zu - SKIPPED (too close to obstacles, <%.1fm safety margin)",
                    i, centroid.x(), clusters[i].indices.size(), safety_margin_);
                continue;
            }
            
            found_distant_cluster = true;
            
            double size_score = static_cast<double>(clusters[i].indices.size()) * 8.0; // Bonus for larger clusters
            double depth_bonus = -centroid.x() * 0.05; // Prefer deeper clusters (smaller X) with a smaller weight
            double distance_score = std::min(distance, 200.0) * 0.5; // Bonus for being farther from drone, capped at 200m
            double entrance_penalty = 0.0; // Penalize clusters near the entrance (X > -350) to encourage deeper exploration
            if (centroid.x() > entrance_x) {
                entrance_penalty = (centroid.x() - entrance_x) * 100.0; // Strong penalty for clusters near entrance to avoid getting stuck there
            }
            double score = size_score + depth_bonus + distance_score - entrance_penalty; // Combine factors
            RCLCPP_INFO(this->get_logger(),
                "Cluster %zu: X=%.2f, size=%zu, dist=%.2f, size_score=%.1f, depth_bonus=%.1f, entrance_penalty=%.1f, total=%.1f",
                i, centroid.x(), clusters[i].indices.size(), distance,
                size_score, depth_bonus, entrance_penalty, score);

            if (score > best_score) {
                best_score = score;
                best_idx = i;
            }
        }
        
        // Fallback: if all clusters too close, choose deepest one anyway
        if (!found_distant_cluster) {
            RCLCPP_WARN(this->get_logger(), 
                "All %zu clusters <%.1fm away from drone at (%.2f, %.2f, %.2f), selecting deepest as last resort", 
                clusters.size(), min_distance, drone_pos.x(), drone_pos.y(), drone_pos.z());
            best_score = -std::numeric_limits<double>::infinity();
            
            for (size_t i = 0; i < clusters.size(); ++i) {
                Eigen::Vector3d centroid = computeClusterCentroid(cloud, clusters[i]);
                double distance = (centroid - drone_pos).norm();
                
                // In emergency fallback: ignore entrance penalty, just find deepest
                double depth_score = -centroid.x();
                double size_bonus = std::log(static_cast<double>(clusters[i].indices.size()) + 1.0) * 5.0;
                double score = depth_score + size_bonus;
                
                RCLCPP_WARN(this->get_logger(),
                    "Fallback cluster %zu: X=%.2f, size=%zu, dist=%.2f, score=%.1f",
                    i, centroid.x(), clusters[i].indices.size(), distance, score);
                
                if (score > best_score) {
                    best_score = score;
                    best_idx = i;
                }
            }
            
            // If still no valid cluster found (shouldn't happen), use first cluster
            if (best_score == -std::numeric_limits<double>::infinity()) {
                RCLCPP_ERROR(this->get_logger(), "No valid cluster found, using cluster 0 as emergency fallback");
                best_idx = 0;
            }
        }
        
        // Log selected cluster
        Eigen::Vector3d selected_centroid = computeClusterCentroid(cloud, clusters[best_idx]);
        RCLCPP_INFO(this->get_logger(), 
            ">>> SELECTED cluster %zu at X=%.2f (score=%.1f)", 
            best_idx, selected_centroid.x(), best_score);
        
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

    // Adjust Z coordinate to be in the middle of the cave at the given XY position
    // by finding the highest and lowest occupied voxels
    Eigen::Vector3d adjustZToMidHeight(const Eigen::Vector3d &centroid) {
        Eigen::Vector3d adjusted = centroid;
        
        // Search range in Z direction
        double search_z_min = -50.0;  // Minimum expected Z in cave
        double search_z_max = 100.0;  // Maximum expected Z in cave
        double z_step = 1.0;          // Step size for Z search (octree resolution)
        
        double z_min_occupied = search_z_max;
        double z_max_occupied = search_z_min;
        bool found_any = false;
        
        // Use the appropriate octree
        if (color_octree_) {
            auto &ct = *color_octree_;
            double res = ct.getResolution();
            
            // Search vertically at the centroid's XY position
            for (double z = search_z_min; z <= search_z_max; z += res) {
                auto node = ct.search(centroid.x(), centroid.y(), z);
                if (node && ct.isNodeOccupied(node)) {
                    found_any = true;
                    if (z < z_min_occupied) z_min_occupied = z;
                    if (z > z_max_occupied) z_max_occupied = z;
                }
            }
        } else if (octree_) {
            auto &ot = *octree_;
            double res = ot.getResolution();
            
            // Search vertically at the centroid's XY position
            for (double z = search_z_min; z <= search_z_max; z += res) {
                auto node = ot.search(centroid.x(), centroid.y(), z);
                if (node && ot.isNodeOccupied(node)) {
                    found_any = true;
                    if (z < z_min_occupied) z_min_occupied = z;
                    if (z > z_max_occupied) z_max_occupied = z;
                }
            }
        }
        
        // Set Z to safe flying height
        if (found_any) {
            double height = z_max_occupied - z_min_occupied;
            
            // Safety checks
            const double min_passage_height = 3.0;  // Minimum clearance needed
            const double max_flight_z = 35.0;        // Maximum safe altitude
            
            if (height < min_passage_height) {
                RCLCPP_WARN(this->get_logger(), 
                    "Passage too narrow at (%.2f, %.2f): height=%.2fm (floor=%.2f, ceiling=%.2f), using original Z=%.2f",
                    centroid.x(), centroid.y(), height, z_min_occupied, z_max_occupied, centroid.z());
                // Keep original Z if passage is too narrow
                return adjusted;
            }
            
            // Fly at 50% height between floor and ceiling (closer to floor for safety)
            //adjusted.z() = z_min_occupied + height * 0.5;
            adjusted.z() = (z_min_occupied + z_max_occupied) / 2.0;
            
            // Cap at maximum safe altitude
            if (adjusted.z() > max_flight_z) {
                RCLCPP_WARN(this->get_logger(),
                    "Adjusted Z=%.2f exceeds max safe altitude %.2f, capping",
                    adjusted.z(), max_flight_z);
                adjusted.z() = max_flight_z;
            }
            
            RCLCPP_INFO(this->get_logger(), 
                "Adjusted Z from %.2f to %.2f (floor=%.2f, ceiling=%.2f, height=%.2fm)",
                centroid.z(), adjusted.z(), z_min_occupied, z_max_occupied, height);
        } else {
            RCLCPP_WARN(this->get_logger(),
                "No occupied voxels found in Z-column at (%.2f, %.2f), using safe default Z=15.0",
                centroid.x(), centroid.y());
            // Use safe default height if no obstacles found
            adjusted.z() = 15.0;
        }
        
        return adjusted;
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
        
        RCLCPP_INFO(this->get_logger(), 
            "Published frontier goal at (%.2f, %.2f, %.2f) with yaw=%.1f° from cluster size %zu", 
            centroid.x(), centroid.y(), centroid.z(), yaw * 180.0 / M_PI, cluster_size);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrontierExploration>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}