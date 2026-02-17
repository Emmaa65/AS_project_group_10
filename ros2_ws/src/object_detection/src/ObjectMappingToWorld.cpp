#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
//#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cstdint>
#include <message_filters/subscriber.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <rclcpp/logging.hpp>
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
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "ObjectManager.hpp"

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
        //point_3d_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("object_point_3d", 10); 
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

        //laterns are (255,235,4)
        cv::Vec3b laterns(255, 235, 4);
        
        // Get camera intrinsics of semantic camera
        double sem_fx = semantic_info->k[0];  // k is a 9-element array (3x3 K matrix)
        double sem_fy = semantic_info->k[4];
        double sem_cx = semantic_info->k[2];
        double sem_cy = semantic_info->k[5];

        std::vector<cv::Point3d> points_3d;
        
        for (int v = 0; v < semantic_cv.rows; ++v) {
            for (int u = 0; u < semantic_cv.cols; ++u) {

                cv::Vec3b pixel = semantic_cv.at<cv::Vec3b>(v, u);
                if (pixel != laterns) {
                    continue;
                  }  // Skip background

                uint16_t z_mm = depth_cv.at<uint16_t>(v, u);
                if (z_mm == 0) continue;  // Skip invalid/zero depth
                
                double z = z_mm / 1000.0;  // Convert mm to meters
                double x = (u - sem_cx) * z / sem_fx;
                double y = (v - sem_cy) * z / sem_fy;

                points_3d.push_back(cv::Point3d(x, y, z));
                
            }
        }

        //transform to world:
        points_3d = this->camera_to_world(points_3d, depth->header.stamp);

        //update objectManager
        object_manager_.process_detection(points_3d, this->get_logger());
        object_manager_.print_all_objects(this->get_logger());

        //mean of points:
         cv::Point3d mean = this->compute_mean(points_3d);
        if(!(mean.x == 0 && mean.y == 0 && mean.z == 0)){
            RCLCPP_INFO(this->get_logger(), "Mean point: x=%.3f, y=%.3f, z=%.3f", mean.x, mean.y, mean.z); 
        }
        
        sensor_msgs::msg::PointCloud2 cloud_msg = this->transformToPointCloud(points_3d, semantic);
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

    //ObjectManager
    ObjectManager object_manager_{20.0};

    //--------------- helper functions --------------------------------------------

    //transform to point_cloud: 
    sensor_msgs::msg::PointCloud2 transformToPointCloud(std::vector<cv::Point3d> points_3d, const sensor_msgs::msg::Image::ConstSharedPtr& semantic){

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = semantic->header.stamp; 
        cloud_msg.header.frame_id = "world"; 
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
        return cloud_msg;
     }


    //compute mean of points
    cv::Point3d compute_mean(const std::vector<cv::Point3d>& points) {
        if (points.empty()) {
            return cv::Point3d(0, 0, 0);
        }
        
        double sum_x = 0, sum_y = 0, sum_z = 0;
        for (const auto& p : points) {
            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;
        }
        
        int n = points.size();
        return cv::Point3d(sum_x / n, sum_y / n, sum_z / n);
    }

    // Transform from camera to world
    std::vector<cv::Point3d> camera_to_world(const std::vector<cv::Point3d>& points_3d, const builtin_interfaces::msg::Time& semantic_stamp)
    {

        std::vector<cv::Point3d> points_world;

       try {
        // Get the transform ONCE before transforming all points
        geometry_msgs::msg::TransformStamped transform = 
            tf_buffer_->lookupTransform("world", "Quadrotor/Sensors/SemanticCamera_optical", 
                                       rclcpp::Time(semantic_stamp), tf2::durationFromSec(0.5));                           
        
            // Now transform all points using this single transform
            for (const auto& p : points_3d) {
                geometry_msgs::msg::PointStamped point_stamped;
                point_stamped.header.frame_id = "Quadrotor/Sensors/SemanticCamera_optical";
                point_stamped.header.stamp = semantic_stamp;
                point_stamped.point.x = p.x;
                point_stamped.point.y = p.y;
                point_stamped.point.z = p.z;
                
                // Apply the transform to each point
                geometry_msgs::msg::PointStamped point_world;
                tf2::doTransform(point_stamped, point_world, transform);
                
                points_world.push_back(cv::Point3d(
                    point_world.point.x,
                    point_world.point.y,
                    point_world.point.z)
                );

              
            }
 

        } catch (const tf2::TransformException& e) {
        RCLCPP_ERROR(this->get_logger(), 
            "TF2 Exception: %s", e.what());
        
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), 
                "Transform lookup failed: %s", e.what());
        }
        
        
        return points_world;
    }
   
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectToWorld>());
  rclcpp::shutdown();
  return 0;
}
