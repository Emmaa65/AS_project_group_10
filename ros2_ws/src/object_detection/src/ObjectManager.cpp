#include "ObjectManager.hpp"

ObjectManager::ObjectManager(double max_distance) 
    : next_id_(0), max_distance_(max_distance) {}

void ObjectManager::process_detection(const std::vector<cv::Point3d>& new_points,
                                     rclcpp::Logger logger) {
    if (new_points.empty()) return;
    
    cv::Point3d new_mean = compute_mean(new_points);
    
    // Find closest existing object
    int closest_id = -1;
    double closest_distance = max_distance_;
    
    for (auto& obj : detected_objects_) {
        double dist = cv::norm(obj.position - new_mean);
        if (dist < closest_distance) {
            closest_distance = dist;
            closest_id = obj.id;
        }
    }
    
    if (closest_id != -1) {
        // Update existing object
        for (auto& obj : detected_objects_) {
            if (obj.id == closest_id) {
                for (const auto& p : new_points) {
                    obj.accumulated_points.push_back(p);
                }
                obj.update_position();
                break;
            }
        }
    } else {
        // Create new object
        DetectedObject new_obj;
        new_obj.id = next_id_++;
        new_obj.accumulated_points = new_points;
        new_obj.update_position();
        detected_objects_.push_back(new_obj);
        
        RCLCPP_INFO(logger,
            "New object detected: ID=%d, pos=(%.3f, %.3f, %.3f)",
            new_obj.id, new_obj.position.x, new_obj.position.y, new_obj.position.z);
    }
}

std::vector<DetectedObject> ObjectManager::get_all_objects() const {
    return detected_objects_;
}

DetectedObject* ObjectManager::get_object(int id) {
    for (auto& obj : detected_objects_) {
        if (obj.id == id) {
            return &obj;
        }
    }
    return nullptr;
}

void ObjectManager::print_all_objects(rclcpp::Logger logger) const {
    RCLCPP_INFO(logger, "Total objects detected: %zu", detected_objects_.size());
    for (const auto& obj : detected_objects_) {
        RCLCPP_INFO(logger,
            "  Object %d: pos=(%.3f, %.3f, %.3f), points=%zu",
            obj.id, obj.position.x, obj.position.y, obj.position.z,
            obj.accumulated_points.size());
    }
}

cv::Point3d ObjectManager::compute_mean(const std::vector<cv::Point3d>& points) const {
    if (points.empty()) return cv::Point3d(0, 0, 0);
    double sum_x = 0, sum_y = 0, sum_z = 0;
    for (const auto& p : points) {
        sum_x += p.x;
        sum_y += p.y;
        sum_z += p.z;
    }
    int n = points.size();
    return cv::Point3d(sum_x / n, sum_y / n, sum_z / n);
}