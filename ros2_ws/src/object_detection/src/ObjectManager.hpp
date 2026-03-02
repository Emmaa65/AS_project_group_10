#ifndef OBJECT_MANAGER_HPP
#define OBJECT_MANAGER_HPP

#include <vector>
#include <fstream>
#include <opencv2/core.hpp>
#include <rclcpp/rclcpp.hpp>

struct DetectedObject {
    int id;
    cv::Point3d position;
    std::vector<cv::Point3d> accumulated_points;
    
    void update_position() {
        if (accumulated_points.empty()) return;
        
        double sum_x = 0, sum_y = 0, sum_z = 0;
        for (const auto& p : accumulated_points) {
            sum_x += p.x;
            sum_y += p.y;
            sum_z += p.z;
        }
        int n = accumulated_points.size();
        position = cv::Point3d(sum_x / n, sum_y / n, sum_z / n);
    }
};

class ObjectManager {
public:
    ObjectManager(double max_distance = 0.5);
    
    void process_detection(const std::vector<cv::Point3d>& new_points, 
                          rclcpp::Logger logger);
    
    std::vector<DetectedObject> get_all_objects() const;
    DetectedObject* get_object(int id);
    
    void print_all_objects(rclcpp::Logger logger) const;
    void save_to_file(const std::string& filepath, rclcpp::Logger logger) const;
    
private:
    std::vector<DetectedObject> detected_objects_;
    int next_id_;
    double max_distance_;
    
    cv::Point3d compute_mean(const std::vector<cv::Point3d>& points) const;
};

#endif