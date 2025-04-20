#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <cmath>
#include "std_msgs/msg/float64.hpp"

// PID Controller Class
class PIDController {
public:
    PIDController(double kp, double ki, double kd) 
        : kp_(kp), ki_(ki), kd_(kd), prev_error_(0.0), integral_(0.0) {}

    double compute(double error, double dt) {
        if (dt < 1e-6) return 0.0;  // Prevent division by zero
        integral_ += error * dt;
        double derivative = (error - prev_error_) / dt;
        prev_error_ = error;
        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

private:
    double kp_, ki_, kd_;
    double prev_error_, integral_;
};

// MultiSubscriberNode Class Definition
class MultiSubscriberNode : public rclcpp::Node {
public:
    MultiSubscriberNode() 
        : Node("multi_subscriber_node"),
          pid_submarine_x_(8.0, 0.4, 0.3),
          pid_submarine_y_(1.0, 0.0, 0.1),
          pid_submarine_z_(1.0, 0.0, 0.1),
          pid_wamv_x_(1.0, 0.0, 0.1),
          pid_wamv_y_(1.0, 0.0, 0.1),
          pid_wamv_orientation_(1.0, 0.0, 0.1) {
        
        // Initialize Subscribers
        sub_submarine_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/my_lrauv_modified/submarine/odometry", 10,
            std::bind(&MultiSubscriberNode::submarine_callback, this, std::placeholders::_1));

        sub_wamv_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/wamv/ground_truth/odometry", 10,
            std::bind(&MultiSubscriberNode::wamv_callback, this, std::placeholders::_1));

        // Initialize Publishers
        pub_submarine_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/my_lrauv_modified/propeller/thrust", 10);
        pub_wamv_left_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", 10);
        pub_wamv_right_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", 10);

        // Set Desired Positions & Orientations
        desired_submarine_x_ = 6.0;
        desired_wamv_x_ = 10.0;
        desired_wamv_y_ = 20.0;

        last_time_ = this->now();
    }

private:
    void submarine_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt < 1e-6) return;  // Prevent zero division

        // Extract Current Position
        double current_x = msg->pose.pose.position.x;

        // Compute Error
        double error_x = desired_submarine_x_ - current_x;
        RCLCPP_INFO(this->get_logger(), "Submarine Error: [%.2f]", error_x);

        // PID Control Output
        double thrust_x = pid_submarine_x_.compute(error_x, dt);

        // Apply thrust saturation
        thrust_x = std::clamp(thrust_x, -5.0, 5.0);

        if(error_x<0){
            thrust_x= -thrust_x;
        }
        else if (error_x>0){
            thrust_x= -thrust_x;
        }
        if (abs(error_x) <= 0.8 ){
            thrust_x=0;
        }


        // Publish Control Commands
        auto thrust_msg = std_msgs::msg::Float64();
        thrust_msg.data = thrust_x;
        pub_submarine_thrust_->publish(thrust_msg);

        RCLCPP_INFO(this->get_logger(), "Submarine Thrust: [%.2f]", thrust_x);
    }

    void wamv_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt < 1e-6) return;  // Prevent zero division

        // Extract Current Position
        double current_x = msg->pose.pose.position.x;

        // Compute Error
        double error_x = desired_wamv_x_ - current_x;
        RCLCPP_INFO(this->get_logger(), "WAMV Error: [%.2f]", error_x);

        // PID Control Output
        double thrust_x = pid_wamv_x_.compute(error_x, dt);

        // Apply thrust saturation
        thrust_x = std::clamp(thrust_x, -5.0, 5.0);

        // Publish Control Commands
        auto thrust_msg = std_msgs::msg::Float64();
        thrust_msg.data = thrust_x;
        pub_wamv_left_thrust_->publish(thrust_msg);
        pub_wamv_right_thrust_->publish(thrust_msg);

        RCLCPP_INFO(this->get_logger(), "WAMV Thrust: [%.2f]", thrust_x);
    }

    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_submarine_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_wamv_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_submarine_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_wamv_left_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_wamv_right_thrust_;

    // Desired Positions
    double desired_submarine_x_;
    double desired_wamv_x_, desired_wamv_y_;

    // PID Controllers
    PIDController pid_submarine_x_, pid_submarine_y_, pid_submarine_z_;
    PIDController pid_wamv_x_, pid_wamv_y_, pid_wamv_orientation_;

    rclcpp::Time last_time_;
};

// Main Function
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
