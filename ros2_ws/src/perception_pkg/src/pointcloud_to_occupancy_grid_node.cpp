#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <cmath>
#include <vector>

class PointCloudToOccupancyGridNode : public rclcpp::Node {
public:
    PointCloudToOccupancyGridNode() : Node("pointcloud_to_occupancy_grid_node") {
        // Parameter declarations
        this->declare_parameter<std::string>("pointcloud_topic", "/camera/pointcloud");
        this->declare_parameter<std::string>("output_topic", "/occupancy_grid");
        this->declare_parameter<std::string>("global_frame", "world");
        this->declare_parameter<double>("resolution", 0.05);  // 5cm per cell
        this->declare_parameter<double>("grid_width", 20.0);   // 20m width
        this->declare_parameter<double>("grid_height", 20.0);  // 20m height
        this->declare_parameter<double>("min_obstacle_height", 0.1);  // obstacles above 10cm
        this->declare_parameter<double>("max_obstacle_height", 2.5);  // obstacles below 2.5m
        this->declare_parameter<double>("ground_height_tolerance", 0.1);  // ±10cm from ground
        
        // Get parameters
        pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
        output_topic_ = this->get_parameter("output_topic").as_string();
        global_frame_ = this->get_parameter("global_frame").as_string();
        resolution_ = this->get_parameter("resolution").as_double();
        grid_width_ = this->get_parameter("grid_width").as_double();
        grid_height_ = this->get_parameter("grid_height").as_double();
        min_obstacle_height_ = this->get_parameter("min_obstacle_height").as_double();
        max_obstacle_height_ = this->get_parameter("max_obstacle_height").as_double();
        ground_height_tolerance_ = this->get_parameter("ground_height_tolerance").as_double();

        // Calculate grid dimensions
        grid_width_cells_ = static_cast<int>(grid_width_ / resolution_);
        grid_height_cells_ = static_cast<int>(grid_height_ / resolution_);

        RCLCPP_INFO(this->get_logger(), "Starting PointCloud to Occupancy Grid Node");
        RCLCPP_INFO(this->get_logger(), "Grid: %dx%d cells, resolution: %.2fm", 
                    grid_width_cells_, grid_height_cells_, resolution_);
        
        // TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Subscriber
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic_, 10,
            std::bind(&PointCloudToOccupancyGridNode::pointcloudCallback, this, std::placeholders::_1));
        
        // Publisher
        occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic_, 10);
        
        RCLCPP_INFO(this->get_logger(), "Subscribed to: %s", pointcloud_topic_.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing occupancy grid to: %s", output_topic_.c_str());
    }

private:
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
        // Transform point cloud to global frame if necessary
        sensor_msgs::msg::PointCloud2 transformed_cloud;
        try {
            if (cloud_msg->header.frame_id != global_frame_) {
                // Wait for transform
                auto transform = tf_buffer_->lookupTransform(
                    global_frame_, cloud_msg->header.frame_id, 
                    cloud_msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
                
                // Transform point cloud
                tf2::doTransform(*cloud_msg, transformed_cloud, transform);
            } else {
                transformed_cloud = *cloud_msg;
            }
        } catch (tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                "Transform failed: %s. Using original frame.", ex.what());
            transformed_cloud = *cloud_msg;
        }

        // Create occupancy grid
        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp = cloud_msg->header.stamp;
        grid.header.frame_id = global_frame_;
        
        grid.info.resolution = resolution_;
        grid.info.width = grid_width_cells_;
        grid.info.height = grid_height_cells_;
        
        // Origin at center of grid
        grid.info.origin.position.x = -grid_width_ / 2.0;
        grid.info.origin.position.y = -grid_height_ / 2.0;
        grid.info.origin.position.z = 0.0;
        grid.info.origin.orientation.w = 1.0;
        
        // Initialize grid with unknown values (-1)
        grid.data.resize(grid_width_cells_ * grid_height_cells_, -1);
        
        // Create temporary grid for counting hits
        std::vector<int> hit_count(grid_width_cells_ * grid_height_cells_, 0);
        std::vector<int> obstacle_count(grid_width_cells_ * grid_height_cells_, 0);
        
        // Iterate through point cloud
        sensor_msgs::PointCloud2ConstIterator<float> iter_x(transformed_cloud, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(transformed_cloud, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(transformed_cloud, "z");
        
        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            float x = *iter_x;
            float y = *iter_y;
            float z = *iter_z;
            
            // Skip invalid points
            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
                continue;
            }
            
            // Convert to grid coordinates
            int grid_x = static_cast<int>((x - grid.info.origin.position.x) / resolution_);
            int grid_y = static_cast<int>((y - grid.info.origin.position.y) / resolution_);
            
            // Check bounds
            if (grid_x < 0 || grid_x >= grid_width_cells_ || 
                grid_y < 0 || grid_y >= grid_height_cells_) {
                continue;
            }
            
            int index = grid_y * grid_width_cells_ + grid_x;
            hit_count[index]++;
            
            // Check if point is an obstacle (between min and max height)
            if (z > min_obstacle_height_ && z < max_obstacle_height_) {
                obstacle_count[index]++;
            }
        }
        
        // Fill occupancy grid
        for (int i = 0; i < grid_width_cells_ * grid_height_cells_; ++i) {
            if (hit_count[i] == 0) {
                // No hits -> unknown
                grid.data[i] = -1;
            } else if (obstacle_count[i] > 0) {
                // Has obstacle points -> occupied
                grid.data[i] = 100;
            } else {
                // Has points but no obstacles -> free
                grid.data[i] = 0;
            }
        }
        
        // Publish occupancy grid
        occupancy_grid_pub_->publish(grid);
        
        RCLCPP_DEBUG(this->get_logger(), "Published occupancy grid with %d cells", 
                    grid_width_cells_ * grid_height_cells_);
    }

    // Subscriber & Publisher
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    
    // TF2
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Parameters
    std::string pointcloud_topic_;
    std::string output_topic_;
    std::string global_frame_;
    double resolution_;
    double grid_width_;
    double grid_height_;
    double min_obstacle_height_;
    double max_obstacle_height_;
    double ground_height_tolerance_;
    
    int grid_width_cells_;
    int grid_height_cells_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudToOccupancyGridNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}