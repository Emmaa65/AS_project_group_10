#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <message_filters/subscriber.h>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

//#include <image_geometry/pinhole_camera_model.hpp>
//#include <sensor_msgs/distortion_models.hpp> //mabye not needed

class ObjectToWorld : public rclcpp::Node
{
public:

    //constructor
    ObjectToWorld() : rclcpp::Node("object_to_world") {
        //subscriber stuff
        semantic_img_sub_.subscribe(this, "semantic_image");
        depth_img_sub_.subscribe(this, "depth_image");
        semantic_camera_info_sub_.subscribe(this, "semantic_camera_info");
        
        sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(
            MySyncPolicy(10), semantic_img_sub_, depth_img_sub_, semantic_camera_info_sub_);
        sync_->registerCallback(std::bind(&ObjectToWorld::callback, this,
            std::placeholders::_1, std::placeholders::_2, 
            std::placeholders::_3));

        //publisher  
        point_cloud2_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("object_point_cloud", 10); 

        //transformation
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_buffer_->setUsingDedicatedThread(true);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        //printing
        RCLCPP_INFO(this->get_logger(), "Subscribing to semantic image: %s", "semantic_image");
        RCLCPP_INFO(this->get_logger(), "Subscribing to depth image: %s", "depth_image");
        RCLCPP_INFO(this->get_logger(), "Subscribing to semantic camera info: %s", "semantic_camera_info");
        RCLCPP_INFO(this->get_logger(), "Publishing point cloud to: %s", point_cloud2_pub_->get_topic_name());
       
    }

    //callback for all 4 topics at once
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& semantic,
                  const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                  const sensor_msgs::msg::CameraInfo::ConstSharedPtr& semantic_info) {
        // Process all 4 messages together
        convertTo3D(semantic, depth, semantic_info);
    }



    //doing the math with all 4 messages from the same time
    void convertTo3D(const sensor_msgs::msg::Image::ConstSharedPtr& semantic,
                     const sensor_msgs::msg::Image::ConstSharedPtr& depth,
                     const sensor_msgs::msg::CameraInfo::ConstSharedPtr& semantic_info) {

        
        // Convert ROS images to cv::Mat
        cv::Mat semantic_cv = cv_bridge::toCvShare(semantic, "rgb8")->image;
        cv::Mat depth_cv = cv_bridge::toCvShare(depth, "16UC1")->image;
        
        // Get camera intrinsics of semantic camera
        double sem_fx = semantic_info->k[0];  // k is a 9-element array (3x3 K matrix)
        double sem_fy = semantic_info->k[4];
        double sem_cx = semantic_info->k[2];
        double sem_cy = semantic_info->k[5];

        //get camera intrinics of depth camera :not needed since it is not a pixel to unproject but already a real depth
      
        // Convert pixel (u, v) to 3D point
        // Iterate through semantic image and extract 3D points
        std::vector<cv::Point3d> points_3d;
        
        for (int v = 0; v < semantic_cv.rows; ++v) {
            for (int u = 0; u < semantic_cv.cols; ++u) {

                uint8_t semantic_value = semantic_cv.at<uint8_t>(v, u);
                if (semantic_value == 0) {
                    RCLCPP_INFO(this->get_logger(), "semantic_value at (u,v) = (%d,%d)", u,v);
                    continue;
                  }  // Skip background
                
                RCLCPP_INFO(this->get_logger(), "semantic_vale nonzero: %d", semantic_value);

                uint16_t z_mm = depth_cv.at<uint16_t>(v, u);
                if (z_mm == 0) continue;  // Skip invalid/zero depth
                
                double z = z_mm / 1000.0;  // Convert mm to meters if needed
                double x = (u - sem_cx) * z / sem_fx;
                double y = (v - sem_cy) * z / sem_fy;
                
                points_3d.push_back(cv::Point3d(x, y, z));
            }
        }

        if (!points_3d.empty()) {
        RCLCPP_INFO(this->get_logger(), "First point: x=%.3f, y=%.3f, z=%.3f", 
                points_3d[0].x, points_3d[0].y, points_3d[0].z);
        }

        //publish the cv::Point3d -> transform to message 
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = depth->header.stamp; 
        cloud_msg.header.frame_id = depth->header.frame_id; //should be /Quadrotor/Sensors/SemanticCamera
        // I need to convert to world frame!! 
        cloud_msg.height = 1;
        cloud_msg.width = points_3d.size();
        cloud_msg.is_dense = false;
        
        //populate cloud 
        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                                      "z", 1, sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(points_3d.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");

        for (const auto& pt : points_3d) {
            *iter_x = pt.x; ++iter_x;
            *iter_y = pt.y; ++iter_y;
            *iter_z = pt.z; ++iter_z;
        }

        //try w.o transformation first!
        point_cloud2_pub_->publish(cloud_msg);
        
  
    }

  
private: 

    // Subscriber & Publisher
    message_filters::Subscriber<sensor_msgs::msg::CameraInfo> semantic_camera_info_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> semantic_img_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_img_sub_;
 

    using MySyncPolicy = message_filters::sync_policies::ApproximateTime
                        <sensor_msgs::msg::Image,
                        sensor_msgs::msg::Image,
                        sensor_msgs::msg::CameraInfo>;

    std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

    //Publisher: publishes the 3d Points of object (can be then later converted to voxel)
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud2_pub_;

    //transformations
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
   
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectToWorld>());
  rclcpp::shutdown();
  return 0;
}
