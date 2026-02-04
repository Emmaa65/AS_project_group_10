#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include <mav_msgs/msg/actuators.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#include <Eigen/Dense>
#include <tf2/utils.h>

namespace tf2 {
  inline void fromMsg(const geometry_msgs::msg::Quaternion &msg, tf2::Quaternion &bt) {
    bt = tf2::Quaternion(msg.x, msg.y, msg.z, msg.w);
  }
}

#define PI M_PI

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
//  PART 0 |  Autonomous Systems - Fall 2025  - Lab 2 coding assignment
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//
//  In this code, we ask you to implement a geometric controller for a
//  simulated UAV, following the publication:
//
//  [1] Lee, Taeyoung, Melvin Leoky, N. Harris McClamroch. "Geometric tracking
//      control of a quadrotor UAV on SE (3)." Decision and Control (CDC),
//      49th IEEE Conference on. IEEE, 2010
//
//  We use variable names as close as possible to the conventions found in the
//  paper, however, we have slightly different conventions for the aerodynamic
//  coefficients of the propellers (refer to the lecture notes for these).
//  Additionally, watch out for the different conventions on reference frames
//  (see Lab Handout for more details).
//
//  Eigen is a C++ library for linear algebra that will help you significantly 
//  with the implementation. Check the reference page to learn the basics:
//
//  https://eigen.tuxfamily.org/dox/group__QuickRefPage.html
//
// ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
//                                 end part 0
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

class ControllerNode : public rclcpp::Node {
  // Flags to check if messages have been received
  bool desired_received_ = false;
  bool current_received_ = false;

  
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  //  PART 1 |  Declare ROS callback handlers
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //
  // In this section, you need to declare:
  //   1. two subscribers (for the desired and current UAVStates)
  //   2. one publisher (for the propeller speeds)
  //   3. a timer for your main control loop
  //
  // ~~~~ begin solution
  //
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr desired_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_state_sub_;
  rclcpp::Publisher<mav_msgs::msg::Actuators>::SharedPtr motor_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  //
  // ~~~~ end solution
  // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
  //                                 end part 1
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  // Controller parameters
  double kx, kv, kr, komega; // controller gains - [1] eq (15), (16)

  // Physical constants (we will set them below)
  double m;              // mass of the UAV
  double g;              // gravity acceleration
  double d;              // distance from the center of propellers to the c.o.m.
  double cf,             // Propeller lift coefficient
         cd;             // Propeller drag coefficient
  Eigen::Matrix3d J;     // Inertia Matrix
  Eigen::Vector3d e3;    // [0,0,1]
  Eigen::MatrixXd F2W;   // Wrench-rotor speeds map

  // Controller internals (you will have to set them below)
  // Current state
  Eigen::Vector3d x;     // current position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d v;     // current velocity of the UAV's c.o.m. in the world frame
  Eigen::Matrix3d R;     // current orientation of the UAV
  Eigen::Vector3d omega; // current angular velocity of the UAV's c.o.m. in the *body* frame

  // Desired state
  Eigen::Vector3d xd;    // desired position of the UAV's c.o.m. in the world frame
  Eigen::Vector3d vd;    // desired velocity of the UAV's c.o.m. in the world frame
  Eigen::Vector3d ad;    // desired acceleration of the UAV's c.o.m. in the world frame
  double yawd;           // desired yaw angle

  double hz;             // frequency of the main control loop

  static Eigen::Vector3d Vee(const Eigen::Matrix3d& in){
    Eigen::Vector3d out;
    out << in(2,1), in(0,2), in(1,0);
    return out;
  }

  static double signed_sqrt(double val){
    return val>0?sqrt(val):-sqrt(-val);
  }

public:
  ControllerNode()
  : rclcpp::Node("controller_node"),
    e3(0,0,1),
    F2W(4,4),
    hz(1000.0)
  {
    // Initialize states to safe defaults
    x.setZero();
    v.setZero();
    R.setIdentity();
    omega.setZero();
    xd.setZero();
    vd.setZero();
    ad.setZero();
    yawd = 0.0;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 2 |  Initialize ROS callback handlers
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //
    // In this section, you need to initialize your handlers from part 1.
    // Specifically:
    //  - bind controllerNode::onDesiredState() to the topic "desired_state"
    //  - bind controllerNode::onCurrentState() to the topic "current_state"
    //  - bind controllerNode::controlLoop() to the created timer, at frequency
    //    given by the "hz" variable
    //
    // Hints:
    //  - read the lab handout to find the message type
    //
    // ~~~~ begin solution
    //
    desired_state_sub_ = this->create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
        "desired_state", 10, std::bind(&ControllerNode::onDesiredState, this, std::placeholders::_1));
        
    current_state_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "current_state", 10, std::bind(&ControllerNode::onCurrentState, this, std::placeholders::_1));
        
    motor_pub_ = this->create_publisher<mav_msgs::msg::Actuators>("rotor_speed_cmds", 10);

    control_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0/hz), std::bind(&ControllerNode::controlLoop, this));
    //
    // ~~~~ end solution
    //
    // ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //                                 end part 2
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Declare parameters with default values (fallback if YAML not loaded)
    this->declare_parameter("kx", 2.0);
    this->declare_parameter("kv", 2.0);
    this->declare_parameter("kr", 2.1);
    this->declare_parameter("komega", 0.5);

    // Get values from parameters
    kx = this->get_parameter("kx").as_double();
    kv = this->get_parameter("kv").as_double();
    kr = this->get_parameter("kr").as_double();
    komega = this->get_parameter("komega").as_double();
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 6 [NOTE: save this for last] |  Tune your gains!
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    // Controller gains (move to parameters.yaml for convenience if you like)
    // kx = 1.0;
    // kv = 1.0;             //    **** FIDDLE WITH THESE! ***
    // kr = 1.0;
    // komega = 1.0;

    // Initialize constants
    m = 1.0;
    cd = 1e-5;
    cf = 1e-3;
    g = 9.81;
    d = 0.3;
    J << 1.0,0.0,0.0,
         0.0,1.0,0.0,
         0.0,0.0,1.0;

    // Based on equation (3.9) from lecture notes with 45° rotation
    double sqrt2_2 = sqrt(2.0) / 2.0;  // cos(45°) = sin(45°)

    F2W << cf,           cf,           cf,           cf,
          cf*d*sqrt2_2,  cf*d*sqrt2_2, -cf*d*sqrt2_2, -cf*d*sqrt2_2,
          -cf*d*sqrt2_2, cf*d*sqrt2_2, cf*d*sqrt2_2, -cf*d*sqrt2_2,
          cd,           -cd,           cd,           -cd;

    RCLCPP_INFO(this->get_logger(), 
              "Controller gains: kx=%.2f, kv=%.2f, kr=%.2f, komega=%.2f",
              kx, kv, kr, komega);
    RCLCPP_INFO(this->get_logger(), "controller_node ready (hz=%.1f)", hz);
  }

  void onDesiredState(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint::SharedPtr des_state_msg){
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 3 | Objective: fill in xd, vd, ad, yawd
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //
      // 3.1 Get the desired position, velocity and acceleration from the in-
      //     coming ROS message and fill in the class member variables xd, vd
      //     and ad accordingly. You can ignore the angular acceleration.
      //
      // Hint: use "v << vx, vy, vz;" to fill in a vector with Eigen.
      //
      // ~~~~ begin solution
      //
      xd << des_state_msg->transforms[0].translation.x,
            des_state_msg->transforms[0].translation.y,
            des_state_msg->transforms[0].translation.z;
      
      vd << des_state_msg->velocities[0].linear.x,
            des_state_msg->velocities[0].linear.y,
            des_state_msg->velocities[0].linear.z;
      
      ad << des_state_msg->accelerations[0].linear.x,
            des_state_msg->accelerations[0].linear.y,
            des_state_msg->accelerations[0].linear.z;
      //
      // ~~~~ end solution
      //
      // 3.2 Extract the yaw component from the quaternion in the incoming ROS
      //     message and store in the yawd class member variable
      //
      //  Hints:
      //    - use tf2::getYaw(des_state_msg->transforms[0].rotation)
      //
      // ~~~~ begin solution
      //
      yawd = tf2::getYaw(des_state_msg->transforms[0].rotation);
      // Rotate by 180° to align the drone's forward direction
      yawd = yawd + M_PI;
      while (yawd > M_PI) yawd -= 2.0*M_PI;
      while (yawd <= -M_PI) yawd += 2.0*M_PI;
      // ~~~~ end solution
      
      desired_received_ = true;

      RCLCPP_INFO_ONCE(this->get_logger(), 
          "First desired_state received: xd=[%.2f, %.2f, %.2f]", 
          xd(0), xd(1), xd(2));
  }

  void onCurrentState(const nav_msgs::msg::Odometry::SharedPtr cur_state_msg){
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //  PART 4 | Objective: fill in x, v, R and omega
      // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //
      // Get the current position and velocity from the incoming ROS message and
      // fill in the class member variables x, v, R and omega accordingly.
      //
      //  CAVEAT: cur_state.twist.twist.angular is in the world frame, while omega
      //          needs to be in the body frame!
      //
      // ~~~~ begin solution  
      // Current position
      x << cur_state_msg->pose.pose.position.x,
          cur_state_msg->pose.pose.position.y,
          cur_state_msg->pose.pose.position.z;
      
      // Current velocity
      v << cur_state_msg->twist.twist.linear.x,
          cur_state_msg->twist.twist.linear.y,
          cur_state_msg->twist.twist.linear.z;
      
      // Current orientation (quaternion to rotation matrix)
      Eigen::Quaterniond q(
          cur_state_msg->pose.pose.orientation.w,
          cur_state_msg->pose.pose.orientation.x,
          cur_state_msg->pose.pose.orientation.y,
          cur_state_msg->pose.pose.orientation.z
      );
      R = q.toRotationMatrix();
      
      // Current angular velocity (world frame -> body frame)
      Eigen::Vector3d omega_world;
      omega_world << cur_state_msg->twist.twist.angular.x,
                    cur_state_msg->twist.twist.angular.y,
                    cur_state_msg->twist.twist.angular.z;
      omega = R.transpose() * omega_world;
      //
      current_received_ = true;
      // ~~~~ end solution
      RCLCPP_INFO_ONCE(this->get_logger(), 
          "First current_state received: x=[%.2f, %.2f, %.2f]", 
          x(0), x(1), x(2));
  }

  void controlLoop(){
    if (!desired_received_ || !current_received_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Waiting for both current_state and desired_state...");
        return;
    }

    // *** Für Orientierungs-Tuning ***
    // Kommentiere diese Zeile ein, um nur Orientierung zu tunen:
    // x = xd;  // Setze current position = desired position
    // v = vd;  // Optional: auch Geschwindigkeit gleichsetzen

    Eigen::Vector3d ex, ev, er, eomega;

    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //  PART 5 | Objective: Implement the controller!
    // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    //
    // 5.1 Compute position and velocity errors. Objective: fill in ex, ev.
    //  Hint: [1], eq. (6), (7)
    //
    // ~~~~ begin solution
    ex = x - xd;      // equation (6)
    ev = v - vd;      // equation (7)
    //debug
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
    "ex: [%.2f, %.2f, %.2f], ev: [%.2f, %.2f, %.2f]",
    ex(0), ex(1), ex(2), ev(0), ev(1), ev(2));
    //
    // ~~~~ end solution

    // 5.2 Compute the Rd matrix.
    //
    //  Hint: break it down in 3 parts:
    //    - b3d vector = z-body axis of the quadrotor, [1] eq. (12)
    //    - check out [1] fig. (3) for the remaining axes [use cross product]
    //    - assemble the Rd matrix, eigen offers: "MATRIX << col1, col2, col3"
    //
    //  CAVEATS:
    //    - Compare the reference frames in the Lab handout with Fig. 1 in the
    //      paper. The z-axes are flipped, which affects the signs of:
    //         i) the gravity term and
    //        ii) the overall sign (in front of the fraction) in equation (12)
    //            of the paper
    //    - remember to normalize your axes!
    //
    // ~~~~ begin solution
    //    
    // Desired b3 axis (z-body axis) - equation (12) with sign corrections
    Eigen::Vector3d b3d;
    b3d = (-kx * ex - kv * ev + m * g * e3 + m * ad);
    b3d.normalize();  // Normalize to get unit vector
    
    // Desired b1 axis (x-body axis)
    Eigen::Vector3d b1d_temp;
    b1d_temp << cos(yawd), sin(yawd), 0;  // Direction based on desired yaw
    
    Eigen::Vector3d b2d = b3d.cross(b1d_temp);  // b2 = b3 x b1_temp
    b2d.normalize();
    
    Eigen::Vector3d b1d = b2d.cross(b3d);  // b1 = b2 x b3
    // b1d is already normalized (cross product of two orthonormal vectors)
    
    // Assemble Rd matrix
    Eigen::Matrix3d Rd;
    Rd << b1d, b2d, b3d;  // Column-wise: [b1d | b2d | b3d]
    //
    // ~~~~ end solution
    //
    // 5.3 Compute the orientation error (er) and the rotation-rate error (eomega)
    //  Hints:
    //     - [1] eq. (10) and (11)
    //     - you can use the Vee() static method implemented above
    //
    //  CAVEATS: feel free to ignore the second addend in eq (11), since it
    //        requires numerical differentiation of Rd and it has negligible
    //        effects on the closed-loop dynamics.
    //
    // ~~~~ begin solution
    //    
    // Orientation error - equation (10)
    Eigen::Matrix3d eR_matrix;
    eR_matrix = 0.5 * (Rd.transpose() * R - R.transpose() * Rd);
    er = Vee(eR_matrix);  // Extract vector from skew-symmetric matrix
    // Angular velocity error - equation (11), ignoring Omega_d terms
    eomega = omega;  // Simplified: we ignore the desired angular velocity term
    //
    // ~~~~ end solution
    //
    // 5.4 Compute the desired wrench (force + torques) to control the UAV.
    //  Hints:
    //     - [1] eq. (15), (16)
    //
    // CAVEATS:
    //    - Compare the reference frames in the Lab handout with Fig. 1 in the
    //      paper. The z-axes are flipped, which affects the signs of:
    //         i) the gravity term
    //        ii) the overall sign (in front of the bracket) in equation (15)
    //            of the paper
    //
    //    - feel free to ignore all the terms involving \Omega_d and its time
    //      derivative as they are of the second order and have negligible
    //      effects on the closed-loop dynamics.
    //
    // ~~~~ begin solution
    //  
    // Desired force in body z-direction - equation (15) with sign correction
    double f = (-kx * ex - kv * ev + m * g * e3 + m * ad).dot(R * e3);
    // Note: +m*g*e3 because our z-axis points up
    //
    // Desired torques - equation (16), ignoring Omega_d terms
    Eigen::Vector3d tau;
    tau = -kr * er - komega * eomega + omega.cross(J * omega);
    //
    // Assemble wrench vector [f, tau_x, tau_y, tau_z]
    Eigen::Vector4d wrench;
    wrench << f, tau(0), tau(1), tau(2);
    //
    // ~~~~ end solution
    //
    // 5.5 Recover the rotor speeds from the wrench computed above
    //
    //  Hints:
    //     - [1] eq. (1)
    //
    // CAVEATs:
    //     - we have different conventions for the arodynamic coefficients,
    //       Namely: C_{\tau f} = c_d / c_f
    //               (LHS paper [1], RHS our conventions [lecture notes])
    //
    //     - Compare the reference frames in the Lab handout with Fig. 1 in the
    //       paper. In the paper [1], the x-body axis [b1] is aligned with a
    //       quadrotor arm, whereas for us, it is 45° from it (i.e., "halfway"
    //       between b1 and b2). To resolve this, check out equation 6.9 in the
    //       lecture notes!
    //
    //     - The thrust forces are **in absolute value** proportional to the
    //       square of the propeller speeds. Negative propeller speeds - although
    //       uncommon - should be a possible outcome of the controller when
    //       appropriate. Note that this is the case in unity but not in real
    //       life, where propellers are aerodynamically optimized to spin in one
    //       direction!
    //
    // ~~~~ begin solution
    //
    Eigen::Vector4d rotor_forces_squared = F2W.inverse() * wrench;
    //

    if (!rotor_forces_squared.allFinite()) {
        RCLCPP_WARN(this->get_logger(), "rotor_forces_squared contains Inf/NaN, skipping control update");
        return;
    }
    // ~~~~ end solution
    //
    // 5.6 Populate and publish the control message
    //
    // Hint: do not forget that the propeller speeds are signed (maybe you want
    // to use signed_sqrt function).
    //
    // ~~~~ begin solution
    //
    mav_msgs::msg::Actuators cmd;
    cmd.angular_velocities.resize(4);

    // Convert squared forces to angular velocities (rad/s)
    // Note: forces can be negative, so we use signed_sqrt
    cmd.angular_velocities[0] = signed_sqrt(rotor_forces_squared(0));
    cmd.angular_velocities[1] = signed_sqrt(rotor_forces_squared(1));
    cmd.angular_velocities[2] = signed_sqrt(rotor_forces_squared(2));
    cmd.angular_velocities[3] = signed_sqrt(rotor_forces_squared(3));
    //
    //for(int i=0; i<4; i++){
    //    double w = signed_sqrt(rotor_forces_squared(i) / cf);
        // Clamp to reasonable max/min angular velocity
        //cmd.angular_velocities[i] = std::clamp(w, 0.0, 1000.0);
    //}


    motor_pub_->publish(cmd);
    //
    // ~~~~ end solution

    // Example publish skeleton (keep after you compute rotor speeds):
    // mav_msgs::msg::Actuators cmd;
    // cmd.angular_velocities.resize(4);
    // cmd.angular_velocities[0] = /* w1 */;
    // cmd.angular_velocities[1] = /* w2 */;
    // cmd.angular_velocities[2] = /* w3 */;
    // cmd.angular_velocities[3] = /* w4 */;
    // motor_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "ex: [%.2f, %.2f, %.2f], f: %.2f, w: [%.1f, %.1f, %.1f, %.1f]",
        ex(0), ex(1), ex(2), wrench(0),
        cmd.angular_velocities[0], cmd.angular_velocities[1],
        cmd.angular_velocities[2], cmd.angular_velocities[3]);
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
