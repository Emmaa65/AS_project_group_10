#include <chrono>
#include <memory>
#include <string>
#include <random>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

class NavigationTest : public rclcpp::Node {
public:
  NavigationTest()
  : Node("navigation_test")
  {
    this->declare_parameter<std::string>("odom_topic", "current_state");
    this->declare_parameter<std::string>("goal_topic", "/exploration/frontier_goal");
    this->declare_parameter<double>("odom_x", -330.0);
    this->declare_parameter<double>("odom_y", -10.0);
    this->declare_parameter<double>("odom_z", 20.0);

    odom_topic_ = this->get_parameter("odom_topic").as_string();
    goal_topic_ = this->get_parameter("goal_topic").as_string();

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 10);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, 10);

    // timer for odom (5 s)
    odom_timer_ = this->create_wall_timer(5s, [this]() { // 100ms (10 Hz)
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = this->now();
      odom.header.frame_id = "map";
      odom.child_frame_id = "base_link";
      odom.pose.pose.position.x = -500.0 + (float)(rand()) / ((float)(RAND_MAX / (500.0 - (-500.0)))); // this->get_parameter("odom_x").as_double();
      odom.pose.pose.position.y = -100.0 + (float)(rand()) / ((float)(RAND_MAX / (100.0 - (-100.0)))); // this->get_parameter("odom_y").as_double();
      odom.pose.pose.position.z = 20; // this->get_parameter("odom_z").as_double();
      odom.pose.pose.orientation.w = 1.0;
      odom.twist.twist.linear.x = 0.0;
      odom.twist.twist.linear.y = 0.0;
      odom.twist.twist.linear.z = 0.0;
      odom_pub_->publish(odom);
      RCLCPP_INFO(this->get_logger(), "Current pose at (%.2f, %.2f, %.2f)",
                odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    });

    // timer for publishing a frontier goal (every 5s)
    goal_timer_ = this->create_wall_timer(5s, [this]() {
      geometry_msgs::msg::PoseStamped p;
      p.header.stamp = this->now();
      p.header.frame_id = "map";
      // random goal or simple fixed goal ahead of odom
      double ox = this->get_parameter("odom_x").as_double();
      double oy = this->get_parameter("odom_y").as_double();
      p.pose.position.x = -500.0 + (float)(rand()) / ((float)(RAND_MAX / (500.0 - (-500.0)))); // ox + 50.0;
      p.pose.position.y = -100.0 + (float)(rand()) / ((float)(RAND_MAX / (100.0 - (-100.0)))); // oy + 20.0;
      p.pose.position.z = 0.0;
      p.pose.orientation.w = 1.0;
      goal_pub_->publish(p);
      RCLCPP_INFO(this->get_logger(), "Published test frontier goal at (%.2f, %.2f)", p.pose.position.x, p.pose.position.y);
    });

    RCLCPP_INFO(this->get_logger(), "navigation_test ready: publishing odom on '%s' and goals on '%s'",
                odom_topic_.c_str(), goal_topic_.c_str());
  }

private:
  std::string odom_topic_;
  std::string goal_topic_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  rclcpp::TimerBase::SharedPtr goal_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigationTest>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
