#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <chrono>

class PointCloudOutlierFilter : public rclcpp::Node
{
public:
    PointCloudOutlierFilter() : Node("pointcloud_outlier_filter")
    {
        // Parameters - Voxel Grid Downsampling
        voxel_leaf_size_ = this->declare_parameter("voxel_leaf_size", 0.1);  // 10cm voxels
        
        // Parameters - Statistical Outlier Removal
        mean_k_ = this->declare_parameter("mean_k", 10);  // Reduced from 20 for speed
        stddev_mul_thresh_ = this->declare_parameter("stddev_mul_thresh", 1.0);  // Relaxed threshold
        
        RCLCPP_INFO(this->get_logger(), 
            "Initialized with voxel_leaf_size=%.3f m, mean_k=%d, stddev_mul_thresh=%.2f", 
            voxel_leaf_size_, mean_k_, stddev_mul_thresh_);
        
        // Subscriber
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "cloud_in", 10,
            std::bind(&PointCloudOutlierFilter::cloudCallback, this, std::placeholders::_1));
        
        // Publisher
        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_out", 10);
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto start = std::chrono::high_resolution_clock::now();
        
        // Convert ROS message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        if (cloud->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
            return;
        }
        
        size_t original_size = cloud->points.size();
        
        // STEP 1: Voxel Grid Downsampling (fast coarse pass)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::VoxelGrid<pcl::PointXYZ> voxel;
        voxel.setInputCloud(cloud);
        voxel.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
        voxel.filter(*cloud_downsampled);
        
        auto after_voxel = std::chrono::high_resolution_clock::now();
        
        // STEP 2: Statistical Outlier Removal (on downsampled cloud - much faster)
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud_downsampled);
        sor.setMeanK(mean_k_);
        sor.setStddevMulThresh(stddev_mul_thresh_);
        sor.filter(*cloud_filtered);
        
        auto after_filter = std::chrono::high_resolution_clock::now();
        
        // Log filtering statistics
        double voxel_ms = std::chrono::duration<double, std::milli>(after_voxel - start).count();
        double filter_ms = std::chrono::duration<double, std::milli>(after_filter - after_voxel).count();
        size_t voxel_removed = original_size - cloud_downsampled->points.size();
        size_t outlier_removed = cloud_downsampled->points.size() - cloud_filtered->points.size();
        
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "Filter: %zu -> %zu (voxel: %.1fms) -> %zu (outlier: %.1fms) | %.1f%% removed",
            original_size, cloud_downsampled->points.size(), voxel_ms,
            cloud_filtered->points.size(), filter_ms,
            100.0 * (voxel_removed + outlier_removed) / original_size);
        
        // Convert back to ROS message
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header = msg->header;
        
        pub_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    double voxel_leaf_size_;
    int mean_k_;
    double stddev_mul_thresh_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudOutlierFilter>());
    rclcpp::shutdown();
    return 0;
}
