#include <rclcpp/rclcpp.hpp>
#include <mav_planning_msgs/msg/polynomial_trajectory4_d.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation/ros_conversions.h>
#include <mav_msgs/eigen_mav_msgs.hpp>
#include <cmath>  // for atan2

class TrajectoryExecutor : public rclcpp::Node {
private:
  // ROS communication
  rclcpp::Subscription<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr traj_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr exploration_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr desired_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr trajectory_finished_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr landing_complete_pub_;
  rclcpp::TimerBase::SharedPtr executor_timer_;

  // Trajectory storage
  mav_trajectory_generation::Trajectory trajectory_;
  bool trajectory_received_ = false;
  rclcpp::Time trajectory_start_time_;
  double trajectory_duration_ = 0.0;
  bool trajectory_finished_ = false;

  // Landing mode state
  bool landing_mode_active_ = false;
  bool landing_complete_ = false;
  bool current_state_received_ = false;
  Eigen::Vector3d current_position_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d current_velocity_ = Eigen::Vector3d::Zero();
  Eigen::Vector2d landing_hold_xy_ = Eigen::Vector2d::Zero();
  double landing_commanded_z_ = 0.0;
  double landing_start_z_ = 0.0;
  rclcpp::Time landing_start_time_;
  rclcpp::Time last_landing_command_time_;
  rclcpp::Time low_motion_start_time_;

  // Landing parameters
  double landing_descent_rate_mps_ = 0.35;
  double landing_velocity_threshold_mps_ = 0.15;
  double landing_vertical_velocity_threshold_mps_ = 0.10;
  double landing_settle_time_s_ = 1.5;
  double landing_min_descent_m_ = 1.0;
  double landing_max_duration_s_ = 120.0;

  double hz_;  // Executor frequency

  void trajectoryCallback(
    const mav_planning_msgs::msg::PolynomialTrajectory4D::SharedPtr msg)
  {
    if (landing_mode_active_ || landing_complete_) {
      RCLCPP_WARN(this->get_logger(),
        "Ignoring incoming trajectory because landing mode is active/completed");
      return;
    }

    // Convert ROS message back to mav_trajectory_generation::Trajectory
    mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory(*msg, &trajectory_);
    
    trajectory_received_ = true;
    trajectory_finished_ = false;
    trajectory_start_time_ = this->now();
    
    // Get total trajectory duration
    trajectory_duration_ = trajectory_.getMaxTime();
    
    RCLCPP_INFO(this->get_logger(), 
      "Received new trajectory with duration: %.2f seconds", trajectory_duration_);
  }

  void explorationStateCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!msg || msg->data != "EXPLORATION_COMPLETE") {
      return;
    }

    if (landing_mode_active_ || landing_complete_) {
      return;
    }

    if (!current_state_received_) {
      RCLCPP_WARN(this->get_logger(),
        "EXPLORATION_COMPLETE received but no current_state_est yet");
      return;
    }

    landing_mode_active_ = true;
    trajectory_received_ = false;
    trajectory_finished_ = true;

    landing_hold_xy_ << current_position_.x(), current_position_.y();
    landing_commanded_z_ = current_position_.z();
    landing_start_z_ = current_position_.z();
    landing_start_time_ = this->now();
    last_landing_command_time_ = this->now();
    low_motion_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());

    RCLCPP_WARN(this->get_logger(),
      "Starting landing mode from z=%.2f m", landing_start_z_);
  }

  void currentStateCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    current_position_ <<
      msg->pose.pose.position.x,
      msg->pose.pose.position.y,
      msg->pose.pose.position.z;

    current_velocity_ <<
      msg->twist.twist.linear.x,
      msg->twist.twist.linear.y,
      msg->twist.twist.linear.z;

    current_state_received_ = true;
  }

  void publishLandingDesiredState()
  {
    const rclcpp::Time now = this->now();
    double dt = (now - last_landing_command_time_).seconds();
    if (dt < 0.0) dt = 0.0;
    if (dt > 0.2) dt = 0.2;
    last_landing_command_time_ = now;

    landing_commanded_z_ -= landing_descent_rate_mps_ * dt;

    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint desired_state;
    desired_state.transforms.resize(1);
    desired_state.transforms[0].translation.x = landing_hold_xy_.x();
    desired_state.transforms[0].translation.y = landing_hold_xy_.y();
    desired_state.transforms[0].translation.z = landing_commanded_z_;
    desired_state.transforms[0].rotation.x = 0.0;
    desired_state.transforms[0].rotation.y = 0.0;
    desired_state.transforms[0].rotation.z = 0.0;
    desired_state.transforms[0].rotation.w = 1.0;

    desired_state.velocities.resize(1);
    desired_state.velocities[0].linear.x = 0.0;
    desired_state.velocities[0].linear.y = 0.0;
    desired_state.velocities[0].linear.z = -landing_descent_rate_mps_;
    desired_state.velocities[0].angular.x = 0.0;
    desired_state.velocities[0].angular.y = 0.0;
    desired_state.velocities[0].angular.z = 0.0;

    desired_state.accelerations.resize(1);
    desired_state.accelerations[0].linear.x = 0.0;
    desired_state.accelerations[0].linear.y = 0.0;
    desired_state.accelerations[0].linear.z = 0.0;
    desired_state.accelerations[0].angular.x = 0.0;
    desired_state.accelerations[0].angular.y = 0.0;
    desired_state.accelerations[0].angular.z = 0.0;

    desired_state_pub_->publish(desired_state);
  }

  bool landingSettled() const
  {
    const double speed = current_velocity_.norm();
    const double vertical_speed = std::abs(current_velocity_.z());
    return speed <= landing_velocity_threshold_mps_ &&
           vertical_speed <= landing_vertical_velocity_threshold_mps_;
  }

  void checkLandingCompletion()
  {
    const rclcpp::Time now = this->now();
    const double elapsed_s = (now - landing_start_time_).seconds();
    const double descended_m = landing_start_z_ - current_position_.z();

    if (elapsed_s >= landing_max_duration_s_) {
      RCLCPP_ERROR(this->get_logger(),
        "Landing timeout after %.1f s - forcing completion", elapsed_s);
      landing_mode_active_ = false;
      landing_complete_ = true;
    } else if (descended_m >= landing_min_descent_m_ && landingSettled()) {
      if (low_motion_start_time_.nanoseconds() == 0) {
        low_motion_start_time_ = now;
      }

      const double settled_s = (now - low_motion_start_time_).seconds();
      if (settled_s >= landing_settle_time_s_) {
        landing_mode_active_ = false;
        landing_complete_ = true;
      }
    } else {
      low_motion_start_time_ = rclcpp::Time(0, 0, this->get_clock()->get_clock_type());
    }

    if (landing_complete_) {
      std_msgs::msg::Bool finish_msg;
      finish_msg.data = true;
      landing_complete_pub_->publish(finish_msg);
      RCLCPP_WARN(this->get_logger(),
        "Landing complete (descended %.2f m, z=%.2f). Published /mission/landing_complete.",
        descended_m, current_position_.z());
    }
  }

  void executorLoop()
  {
    if (landing_complete_) {
      return;
    }

    if (landing_mode_active_) {
      if (!current_state_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Landing mode active, waiting for current_state_est");
        return;
      }

      publishLandingDesiredState();
      checkLandingCompletion();
      return;
    }

    if (!trajectory_received_) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Executor loop running, waiting for trajectory...");
      return;
    }

    if (trajectory_finished_) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Trajectory execution completed, waiting for new trajectory...");
      return;
    }

    // Calculate elapsed time since trajectory start
    double elapsed_time = (this->now() - trajectory_start_time_).seconds();
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Executor loop active: t=%.3f / %.3f", elapsed_time, trajectory_duration_);

    // Check if trajectory is finished
    if (elapsed_time >= trajectory_duration_) {
      trajectory_finished_ = true;
      std_msgs::msg::Bool finish_msg;
      finish_msg.data = true;
      trajectory_finished_pub_->publish(finish_msg);
      RCLCPP_INFO(this->get_logger(), "Trajectory execution finished");
      return;
    }

    // Sample trajectory at current time
    mav_msgs::EigenTrajectoryPoint sampled_point;
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(trajectory_, elapsed_time, &sampled_point);
    if (!success) {
      RCLCPP_ERROR(this->get_logger(), 
        "Failed to sample trajectory at time %.3f", elapsed_time);
      return;
    }

    // Calculate orientation based on velocity direction (point camera towards movement)
    // This makes the camera look in the direction the drone is flying
    double velocity_magnitude = sampled_point.velocity_W.norm();
    Eigen::Quaterniond orientation_W_B;
    
    if (velocity_magnitude > 0.01) {  // Very low threshold to always orient towards movement
      // Calculate yaw from velocity in XY plane
      double yaw = std::atan2(sampled_point.velocity_W(1), sampled_point.velocity_W(0));
      
      // Add 180° offset because camera is mounted backwards relative to velocity direction
      // This makes the drone fly forward with camera pointing in flight direction
      yaw += M_PI;
      
      // Create quaternion for rotation around Z axis (yaw only, keep drone level)
      double half_yaw = yaw / 2.0;
      orientation_W_B = Eigen::Quaterniond(
        std::cos(half_yaw),  // w
        0.0,                  // x
        0.0,                  // y
        std::sin(half_yaw)    // z
      );
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Velocity-based orientation: vel=[%.2f, %.2f, %.2f], mag=%.2f, yaw=%.1f° (with 180° offset)", 
        sampled_point.velocity_W(0), sampled_point.velocity_W(1), sampled_point.velocity_W(2),
        velocity_magnitude, yaw * 180.0 / M_PI);
    } else {
      // At very low velocity, keep level but maintain last yaw
      // This prevents orientation flickering at trajectory start/end
      orientation_W_B = sampled_point.orientation_W_B;
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
        "Low velocity (%.3f m/s), using trajectory orientation", velocity_magnitude);
    }

    // Convert to MultiDOFJointTrajectoryPoint message
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint desired_state;

    // Position and orientation
    desired_state.transforms.resize(1);
    desired_state.transforms[0].translation.x = sampled_point.position_W(0);
    desired_state.transforms[0].translation.y = sampled_point.position_W(1);
    desired_state.transforms[0].translation.z = sampled_point.position_W(2);

    // Use velocity-based orientation instead of sampled orientation
    desired_state.transforms[0].rotation.x = orientation_W_B.x();
    desired_state.transforms[0].rotation.y = orientation_W_B.y();
    desired_state.transforms[0].rotation.z = orientation_W_B.z();
    desired_state.transforms[0].rotation.w = orientation_W_B.w();

    // Velocity
    desired_state.velocities.resize(1);
    desired_state.velocities[0].linear.x = sampled_point.velocity_W(0);
    desired_state.velocities[0].linear.y = sampled_point.velocity_W(1);
    desired_state.velocities[0].linear.z = sampled_point.velocity_W(2);
    desired_state.velocities[0].angular.x = 0.0;
    desired_state.velocities[0].angular.y = 0.0;
    desired_state.velocities[0].angular.z = 0.0;

    // Acceleration
    desired_state.accelerations.resize(1);
    desired_state.accelerations[0].linear.x = sampled_point.acceleration_W(0);
    desired_state.accelerations[0].linear.y = sampled_point.acceleration_W(1);
    desired_state.accelerations[0].linear.z = sampled_point.acceleration_W(2);
    desired_state.accelerations[0].angular.x = 0.0;
    desired_state.accelerations[0].angular.y = 0.0;
    desired_state.accelerations[0].angular.z = 0.0;

    // Publish desired state
    desired_state_pub_->publish(desired_state);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Publishing desired state at t=%.3f: pos=[%.2f, %.2f, %.2f]",
      elapsed_time, sampled_point.position_W(0), 
      sampled_point.position_W(1), sampled_point.position_W(2));
  }

public:
  TrajectoryExecutor()
  : rclcpp::Node("trajectory_executor"),
    hz_(100.0)
  {
    this->declare_parameter("landing_descent_rate_mps", 0.35);
    this->declare_parameter("landing_velocity_threshold_mps", 0.15);
    this->declare_parameter("landing_vertical_velocity_threshold_mps", 0.10);
    this->declare_parameter("landing_settle_time_s", 1.5);
    this->declare_parameter("landing_min_descent_m", 1.0);
    this->declare_parameter("landing_max_duration_s", 120.0);

    landing_descent_rate_mps_ = this->get_parameter("landing_descent_rate_mps").as_double();
    landing_velocity_threshold_mps_ = this->get_parameter("landing_velocity_threshold_mps").as_double();
    landing_vertical_velocity_threshold_mps_ = this->get_parameter("landing_vertical_velocity_threshold_mps").as_double();
    landing_settle_time_s_ = this->get_parameter("landing_settle_time_s").as_double();
    landing_min_descent_m_ = this->get_parameter("landing_min_descent_m").as_double();
    landing_max_duration_s_ = this->get_parameter("landing_max_duration_s").as_double();

    auto latched_qos = rclcpp::QoS(1).transient_local().reliable();

    // Create subscription for trajectory
    traj_sub_ = this->create_subscription<mav_planning_msgs::msg::PolynomialTrajectory4D>(
      "trajectory", 10,
      std::bind(&TrajectoryExecutor::trajectoryCallback, this, std::placeholders::_1));

    exploration_state_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/exploration_state", 10,
      std::bind(&TrajectoryExecutor::explorationStateCallback, this, std::placeholders::_1));

    current_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/current_state_est", 10,
      std::bind(&TrajectoryExecutor::currentStateCallback, this, std::placeholders::_1));

    // Create publisher for desired state
    desired_state_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
      "desired_state", 10);

    // Publish one-shot signal when static trajectory is completed
    trajectory_finished_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "trajectory_finished", latched_qos);

    landing_complete_pub_ = this->create_publisher<std_msgs::msg::Bool>(
      "/mission/landing_complete", latched_qos);

    // Create timer for execution loop
    executor_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / hz_),
      std::bind(&TrajectoryExecutor::executorLoop, this));

    RCLCPP_INFO(this->get_logger(), 
      "Trajectory executor started (hz=%.1f, landing_rate=%.2f m/s)", hz_, landing_descent_rate_mps_);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryExecutor>());
  rclcpp::shutdown();
  return 0;
}
