#include <rclcpp/rclcpp.hpp>
#include <mav_planning_msgs/msg/polynomial_trajectory4_d.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <mav_trajectory_generation/trajectory.h>
#include <mav_trajectory_generation/trajectory_sampling.h>
#include <mav_trajectory_generation/ros_conversions.h>
#include <mav_msgs/eigen_mav_msgs.hpp>
#include <cmath>  // for atan2

class TrajectoryExecutor : public rclcpp::Node {
private:
  // ROS communication
  rclcpp::Subscription<mav_planning_msgs::msg::PolynomialTrajectory4D>::SharedPtr traj_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr desired_state_pub_;
  rclcpp::TimerBase::SharedPtr executor_timer_;

  // Trajectory storage
  mav_trajectory_generation::Trajectory trajectory_;
  bool trajectory_received_ = false;
  rclcpp::Time trajectory_start_time_;
  double trajectory_duration_ = 0.0;
  bool trajectory_finished_ = false;

  double hz_;  // Executor frequency

  void trajectoryCallback(
    const mav_planning_msgs::msg::PolynomialTrajectory4D::SharedPtr msg)
  {
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

  void executorLoop()
  {
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
    // Create subscription for trajectory
    traj_sub_ = this->create_subscription<mav_planning_msgs::msg::PolynomialTrajectory4D>(
      "trajectory", 10,
      std::bind(&TrajectoryExecutor::trajectoryCallback, this, std::placeholders::_1));

    // Create publisher for desired state
    desired_state_pub_ = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
      "desired_state", 10);

    // Create timer for execution loop
    executor_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / hz_),
      std::bind(&TrajectoryExecutor::executorLoop, this));

    RCLCPP_INFO(this->get_logger(), 
      "Trajectory executor started (hz=%.1f)", hz_);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryExecutor>());
  rclcpp::shutdown();
  return 0;
}
