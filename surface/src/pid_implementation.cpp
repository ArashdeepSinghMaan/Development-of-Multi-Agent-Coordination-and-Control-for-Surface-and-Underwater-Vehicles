//IMplementation of PID for AUV


#include <rclcpp/rclcpp.hpp>
#include<nav_msgs/msg/odometry.hpp>
#include<std_msgs/msg/float64.hpp>
#include<geometry_msgs/msg/quaternion.hpp>
#include<cmath>               
#include<vector>
#include<Eigen/Dense>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2/LinearMath/Matrix3x3.h>
#include<sensor_msgs/msg/laser_scan.hpp>
#include "/home/arash/ros2_ws/src/surface/include/surface/oobstacle_processing.hpp"
#include "/home/arash/ros2_ws/src/surface/include/surface/mathtool.hpp"
#include "/home/arash/ros2_ws/src/surface/include/surface/pid_controller.hpp"

#include "/home/arash/ros2_ws/install/message_type/include/message_type/message_type/msg/traj.hpp"



double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &quat){
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;

}
double get_pitch_from_quaternion(const geometry_msgs::msg::Quaternion &quat){
    tf2::Quaternion q(quat.x,quat.y,quat.z,quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch,yaw;
    m.getRPY(roll,pitch,yaw);
    return pitch;

}
class SimplePIDControlNode : public rclcpp::Node {
public:
    SimplePIDControlNode()
        : Node("simple_pid_control_node"),
          pid_x_(0.6, 0.1, 0.2),
          pid_y_(0.6, 0.1, 0.2),
          pid_z_(1.0, 0.1, 0.2)
          //current_waypoint_index_(0)
    {
        // Define waypoints
      //  waypoints_ = {
       //     {10.0, 0.0},
       //     {10.0, 10.0},
       //     {0.0, 10.0},
      //      {0.0, 0.0}
        

        // Subscribers
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/my_lrauv_modified/submarine/odometry", 10,
            std::bind(&SimplePIDControlNode::odom_callback, this, std::placeholders::_1));

        start_time_ = this->now();

        // Publishers
        pub_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/my_lrauv_modified/propeller/thrust", 10);
        pub_pitch_fin_ = this->create_publisher<std_msgs::msg::Float64>("/my_lrauv_modified/submarine/pitch/fin/pos", 10);
        pub_yaw_fin_ = this->create_publisher<std_msgs::msg::Float64>("/my_lrauv_modified/submarine/yaw/fin/pos", 10);
        pub_trajectory_desired_=this->create_publisher<message_type::msg::Traj>("controller/trajectory/desired",10);
        last_time_ = this->now();
    }

private:

    rclcpp::Time start_time_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_yaw_fin_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_pitch_fin_;
    rclcpp::Publisher<message_type::msg::Traj>::SharedPtr pub_trajectory_desired_;

    PIDController pid_x_;
    PIDController pid_y_;
    PIDController pid_z_;  // Not used here, but available if you extend for depth control

    double desired_submarine_x_;
    double desired_submarine_y_;
    double desired_submarine_z_;

    //std::vector<std::pair<double, double>> waypoints_;
    //int current_waypoint_index_;
    rclcpp::Time last_time_;
double simulation_duration_=120.0;
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        if (dt < 1e-6) return;
        last_time_ = now;
         double elapsed_time = (now - start_time_).seconds();


if (elapsed_time > simulation_duration_) {
        RCLCPP_INFO(this->get_logger(),
            "Trajectory tracking complete (%.2f s). Holding position.", elapsed_time);

        // Optionally: send zero thrust and neutral fins
       


        std_msgs::msg::Float64 thrust_msg;
        thrust_msg.data = 0;
        pub_thrust_->publish(thrust_msg);

        std_msgs::msg::Float64 yaw_fin_msg;
        yaw_fin_msg.data =0;  
        pub_yaw_fin_->publish(yaw_fin_msg);

        return; // exit callback early
    }

        // Simplified downward trajectory starting at (0, 0, -2)
double A = 50.0;      // Forward distance in X
double B = 15.0;       // Sway in Y
double C = 2.0;       // Descent depth
double ω = 0.1;      // Lower frequency for slower yaw/pitch

//double x_d=A*ω*elapsed_time;
//double y_d=B*std::sin(0.5 * ω * elapsed_time);
double x_d = A * std::sin(0.8*ω * elapsed_time);                         // Moves forward
double y_d = B * std::sin(  ω * elapsed_time);                   // Smooth gentle sway
//double z_d = -2.0 - C * std::sin(0.5 * ω * elapsed_time);            // Descend from -2.0

//double dx_dt=A*ω;
//double dy_dt=B * 0.5 * ω * std::cos(0.5 * ω * elapsed_time);
// First derivatives (for yaw/pitch calculation)
double dx_dt = A * 0.8* ω * std::cos(0.8* ω * elapsed_time);
double dy_dt = B * ω * std::cos( ω * elapsed_time);
//double dz_dt = -C * 0.5 * ω * std::cos(0.5 * ω * elapsed_time);

// Desired orientation from derivatives
double desired_yaw = std::atan2(dy_dt, dx_dt);
//double desired_pitch = std::atan2(-dz_dt, std::hypot(dx_dt, dy_dt));  // pitch downward

            // Prevent zero division

       

        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        double yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);

        // Select current target waypoint
       // double target_x = waypoints_[current_waypoint_index_].first;
       // double target_y = waypoints_[current_waypoint_index_].second;

        // Move to next waypoint if close enough
       // double distance = std::hypot(target_x - x, target_y - y);
       //// if (distance < 1.0) {
        //    current_waypoint_index_ = (current_waypoint_index_ + 1) % waypoints_.size();
       // }

        // Compute errors
        double error_x = x_d - x;
        double error_y = y_d - y;

        // PID output velocities (we'll just use x direction as thrust here)
        double v_x = pid_x_.compute(error_x, dt);
      //  double v_y = pid_y_.compute(error_y, dt);
        double vx_world = msg->twist.twist.linear.x;
double vy_world = msg->twist.twist.linear.y;

// Transform to body frame
double surge_velocity = vx_world * std::cos(yaw) + vy_world * std::sin(yaw);
        
        double error_vel= v_x-surge_velocity;
        double thrust=pid_y_.compute(error_vel,dt);
        // Use desired yaw angle from velocity vector
      //  double desired_yaw = std::atan2(v_y, v_x);
        double yaw_error = desired_yaw - yaw;

        // Normalize to [-pi, pi]
        while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

        // Compute simple PID yaw control
        double yaw_torque = pid_z_.compute(yaw_error, dt);
        double yaw_fin    = pid_z_.compute_yaw(yaw_torque, surge_velocity);

        // Clamp thrust (we use v_x only for surge)
        double thrust_final = std::clamp(thrust, -2.5, 2.5);

        // Publish commands
        message_type::msg::Traj traj_msg;
        traj_msg.time = elapsed_time;
        traj_msg.x_d=x_d;
        traj_msg.y_d=y_d;
        pub_trajectory_desired_->publish(traj_msg);

        std_msgs::msg::Float64 thrust_msg;
        thrust_msg.data = thrust_final;
        pub_thrust_->publish(thrust_msg);

        std_msgs::msg::Float64 yaw_fin_msg;
        yaw_fin_msg.data = yaw_fin;  // Assume inverse direction
        pub_yaw_fin_->publish(yaw_fin_msg);

        std_msgs::msg::Float64 pitch_fin_msg;
        pitch_fin_msg.data = 0.0;  // No pitch control in this basic PID node
        pub_pitch_fin_->publish(pitch_fin_msg);

        RCLCPP_INFO(this->get_logger(), "PID: Thrsut=%.2f,Thrust Published=%.2f, Yaw_published=%.2f,YawErr=%.2f,VelErr=%.2f",thrust, thrust_final, yaw_fin,yaw_error,error_vel);
    }

};
// ---------------- Main ----------------
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePIDControlNode>());
    rclcpp::shutdown();
    return 0;
}



