#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "surface/cascaded_control.hpp"  

// Extract yaw and pitch from quaternion
void getEulerFromQuaternion(const geometry_msgs::msg::Quaternion &quat,
                            double &roll, double &pitch, double &yaw) {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
}

class MultiSubscriberNode : public rclcpp::Node {
public:
    MultiSubscriberNode()
        : Node("multi_subscriber_node"),
          // Initialize publishers
          pub_submarine_thrust_(this->create_publisher<std_msgs::msg::Float64>(
              "/my_lrauv_modified/propeller/thrust", 10)),
          pub_submarine_fin_(this->create_publisher<std_msgs::msg::Float64>(
              "/my_lrauv_modified/submarine/vertical/fin/pos", 10)),
          // Initialize controller with tuned gains
          controller_(
              /* kp_pos */        1.0, /* ki_pos */ 0.01, /* kd_pos */ 0.1,
              /* kp_yawAngle */   2.0, /* ki_yawAngle */ 0.02, /* kd_yawAngle */ 0.2,
              /* kp_pitchAngle */ 2.0, /* ki_pitchAngle */ 0.02, /* kd_pitchAngle */ 0.2,
              /* kp_u */          5.0, /* ki_u */ 0.1, /* kd_u */ 2.0,
              /* kp_yaw */        10.0, /* ki_yaw */ 0.5, /* kd_yaw */ 3.0,
              /* kp_pitch */      10.0, /* ki_pitch */ 0.5, /* kd_pitch */ 3.0
          )
    {
        // Initialize subscriber
        sub_submarine_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/my_lrauv_modified/submarine/odometry", 10,
            std::bind(&MultiSubscriberNode::submarine_callback, this, std::placeholders::_1));

        last_time_ = this->now();
    }

private:
    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_submarine_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_submarine_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_submarine_fin_;

    // Controller
    auv::CascadedAUVController controller_;

    // Timing
    rclcpp::Time last_time_;

    void submarine_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Compute time step
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;
        if (dt < 1e-6) return;

        // Extract current pose
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;
        double current_z = msg->pose.pose.position.z;
        // Extract current orientation
        double roll, pitch, yaw;
        getEulerFromQuaternion(msg->pose.pose.orientation, roll, pitch, yaw);

        // Extract current velocities
        double u = msg->twist.twist.linear.x;   // surge velocity
        double r = msg->twist.twist.angular.z;  // yaw rate
        double q = msg->twist.twist.angular.y;  // pitch rate

        // Desired 3D position
double desired_x = 12.0;
double desired_y = 1.0;
double desired_z = -2.0;

// Compute deltas
double dx = desired_x - current_x;  // Assuming current_x[0] = x
double dy = desired_y - current_y;  // Assuming current_x[1] = y
double dz = desired_z - current_z;  // Assuming current_x[2] = z

// Desired surge distance (for controller, optional)
double desired_pos = std::sqrt(dx * dx + dy * dy );

// Yaw (heading in XY-plane)
double desired_yaw = std::atan2(dy, dx);

// Pitch (angle from horizontal)
double horizontal_dist = std::sqrt(dx * dx + dy * dy);
double desired_pitch = std::atan2(-dz, horizontal_dist);  // Negative since depth increases down


        // Compute control outputs
        double T, tau_yaw, tau_pitch;
        controller_.computeControl(
            desired_pos, desired_yaw, 
            current_x, yaw, 
            u, r,  dt,
            T, tau_yaw
        );

        // Map yaw torque to fin deflection
        double fin_deflection = auv::computeFinDeflection(tau_yaw, u);
        T = std::clamp(T, 1.0, 4.0); 
        // Publish thrust command
        std_msgs::msg::Float64 thrust_msg;
        thrust_msg.data = -T;
        pub_submarine_thrust_->publish(thrust_msg);
        RCLCPP_INFO(this->get_logger(), "Thrust command: %.2f", T);

        // Publish fin deflection
        std_msgs::msg::Float64 fin_msg;
        fin_msg.data = fin_deflection;
        pub_submarine_fin_->publish(fin_msg);
        RCLCPP_INFO(this->get_logger(), "Fin deflection: %.2f rad", fin_deflection);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
