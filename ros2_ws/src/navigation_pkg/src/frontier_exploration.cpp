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
        
            // Timer Callback for Periodic Exploration
        exploration_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0/1000.0), std::bind(&FrontierExploration::explorationTimerCallback, this)
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
    double cluster_tolerance_ = 1.5; // meters
    int min_cluster_size_ = 1;
    int max_cluster_size_ = 10000;

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

        // Select largest cluster
        size_t largest_idx = findLargestCluster(cluster_indices);

        // Compute centroid and publish
        Eigen::Vector3d centroid = computeClusterCentroid(frontier_cloud, cluster_indices[largest_idx]);
        // RCLCPP_INFO(this->get_logger(), "explorationTimerCallback: largest_idx=%zu centroid=(%.3f, %.3f, %.3f)", largest_idx, centroid.x(), centroid.y(), centroid.z());
        publishGoalFromCentroid(centroid, cluster_indices[largest_idx].indices.size());
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
                if (!ct.isNodeOccupied(*it)) continue;
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
                    if (!ct.isNodeOccupied(node)) { is_frontier = true; break; }
                }
                if (is_frontier) {
                    pcl::PointXYZ p;
                    p.x = static_cast<float>(x);
                    p.y = static_cast<float>(y);
                    p.z = static_cast<float>(z);
                    frontier_cloud->points.push_back(p);
                    ++found;
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
                if (!ot.isNodeOccupied(*it)) continue;
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
                    if (!ot.isNodeOccupied(node)) { is_frontier = true; break; }
                }
                if (is_frontier) {
                    pcl::PointXYZ p;
                    p.x = static_cast<float>(x);
                    p.y = static_cast<float>(y);
                    p.z = static_cast<float>(z);
                    frontier_cloud->points.push_back(p);
                    ++found;
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

    void publishGoalFromCentroid(const Eigen::Vector3d &centroid, size_t cluster_size) {
        if (!frontier_goal_pub_) return;
        geometry_msgs::msg::PoseStamped goal;
        goal.header.stamp = this->now();
        goal.header.frame_id = "map";
        goal.pose.position.x = centroid.x();
        goal.pose.position.y = centroid.y();
        goal.pose.position.z = centroid.z();
        goal.pose.orientation.w = 1.0;
        frontier_goal_pub_->publish(goal);
        RCLCPP_INFO(this->get_logger(), "Published frontier goal at (%.2f, %.2f, %.2f) from cluster size %zu", centroid.x(), centroid.y(), centroid.z(), cluster_size);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrontierExploration>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}