#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

class PointCloudOutlierFilter : public rclcpp::Node
{
public:
    PointCloudOutlierFilter() : Node("pointcloud_outlier_filter")
    {
        // Parameters
        mean_k_ = this->declare_parameter("mean_k", 20);
        stddev_mul_thresh_ = this->declare_parameter("stddev_mul_thresh", 1.5);
        
        RCLCPP_INFO(this->get_logger(), 
            "Initialized with mean_k=%d, stddev_mul_thresh=%.2f", 
            mean_k_, stddev_mul_thresh_);
        
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
        // Convert ROS message to PCL point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud);
        
        if (cloud->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
            return;
        }
        
        // Apply statistical outlier removal
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(cloud);
        sor.setMeanK(mean_k_);
        sor.setStddevMulThresh(stddev_mul_thresh_);
        sor.filter(*cloud_filtered);
        
        // Log filtering statistics
        size_t removed = cloud->points.size() - cloud_filtered->points.size();
        if (removed > 0) {
            RCLCPP_DEBUG(this->get_logger(), 
                "Filtered %zu/%zu outlier points (%.1f%%)", 
                removed, cloud->points.size(), 
                100.0 * removed / cloud->points.size());
        }
        
        // Convert back to ROS message
        sensor_msgs::msg::PointCloud2 output_msg;
        pcl::toROSMsg(*cloud_filtered, output_msg);
        output_msg.header = msg->header;
        
        pub_->publish(output_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
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
