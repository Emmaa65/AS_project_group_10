/*
 * Simple example that shows a trajectory planner using
 * mav_trajectory_generation in ROS2.
 *
 * Build + run via your ROS2 launch setup, e.g.
 *   ros2 run basic_waypoint_pkg basic_waypoint_node
 */

 #include <memory>
 #include <iostream>
 
 #include "rclcpp/rclcpp.hpp"
 #include "Eigen/Dense"
 
 #include "planner.hpp"
 
 int main(int argc, char ** argv)
 {
   rclcpp::init(argc, argv);
 
   // Create node
   auto node = rclcpp::Node::make_shared("simple_planner");
 
   // Instantiate basic planner
   BasicPlanner planner(node);
 
   // Let things settle a bit (similar to ros::Duration(1.0).sleep())
   rclcpp::Rate init_rate(10.0);
   init_rate.sleep();
   
   // Define goal point
   Eigen::Vector3d goal_position, goal_velocity;
   goal_position << -0.1, -10.0, 5.0;//<< -330.0, 10.0, 20.0;
   goal_velocity << 0.0, 0.0, 0.0;

   // Process some callbacks to receive odometry and update current position
   // Wait until we actually receive valid odometry (not the initial zero position)
   RCLCPP_INFO(node->get_logger(), "Waiting for valid odometry message...");
   rclcpp::Rate spin_rate(50.0);
   int max_iterations = 500; // wait up to 10 seconds
   int iterations = 0;
   
   while (iterations < max_iterations && rclcpp::ok()) {
     rclcpp::spin_some(node);
     spin_rate.sleep();
     iterations++;
     
     // Check if we have received odometry and it's not the zero position
     if (planner.hasReceivedOdometry()) {
       auto pos = planner.getCurrentPosition();
       // Check if position is non-zero (not the initial [0,0,0])
       if (std::abs(pos[0]) > 0.1 || std::abs(pos[1]) > 0.1 || std::abs(pos[2]) > 0.1) {
         RCLCPP_INFO(node->get_logger(), "Valid odometry received. Current position: [%.2f, %.2f, %.2f]",
                     pos[0], pos[1], pos[2]);
         break;
       }
     }
   }
   
   if (iterations >= max_iterations) {
     RCLCPP_ERROR(node->get_logger(), "Failed to receive valid odometry! Cannot plan trajectory.");
     rclcpp::shutdown();
     return 1;
   }

   // Plan and publish trajectory
   mav_trajectory_generation::Trajectory trajectory;
   planner.planTrajectory(goal_position, goal_velocity, &trajectory);
   planner.publishTrajectory(trajectory);
 
   RCLCPP_WARN(node->get_logger(), "DONE. GOODBYE.");
 
   rclcpp::shutdown();
   return 0;
 }
 