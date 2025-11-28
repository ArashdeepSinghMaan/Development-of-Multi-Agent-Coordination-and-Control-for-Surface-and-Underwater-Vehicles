//IMplementation of Backstepping Control


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
#include "/home/arash/ros2_ws/src/surface/include/surface/controller.hpp"
#include "/home/arash/ros2_ws/src/surface/include/surface/backstepping_control.hpp"
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

class MultiSubscriberNode : public rclcpp::Node {
    public:
        MultiSubscriberNode()
            : Node("multi_subscriber_node")
            
         
    {
       
            // Subscribe to submarine odometry
            sub_submarine_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/my_lrauv_modified/submarine/odometry", 10,
                std::bind(&MultiSubscriberNode::submarine_callback, this, std::placeholders::_1));
    
           start_time_ = this->now();

            // Initialize Publishers
            pub_submarine_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/my_lrauv_modified/propeller/thrust", 10);
            pub_submarine_fin_pitch_ = this->create_publisher<std_msgs::msg::Float64>("/my_lrauv_modified/submarine/pitch/fin/pos", 10);
            pub_submarine_fin_yaw_=this->create_publisher<std_msgs::msg::Float64>("/my_lrauv_modified/submarine/yaw/fin/pos",10);
            pub_trajectory_desired_=this->create_publisher<message_type::msg::Traj>("controller/trajectory/desired",10);

            //User Desired Pose Topics
        
sub_submarine_desired_x_ = this->create_subscription<std_msgs::msg::Float64>(
    "/user/trajectory/x", 10,
    std::bind(&MultiSubscriberNode::desired_x_callback, this, std::placeholders::_1));

sub_submarine_desired_y_ = this->create_subscription<std_msgs::msg::Float64>(
    "/user/trajectory/y", 10,
    std::bind(&MultiSubscriberNode::desired_y_callback, this, std::placeholders::_1));

sub_submarine_desired_yaw_ = this->create_subscription<std_msgs::msg::Float64>(
    "/user/trajectory/yaw", 10,
    std::bind(&MultiSubscriberNode::desired_yaw_callback, this, std::placeholders::_1));

            last_time_ = this->now();
                 
        }
    
    private:

    rclcpp::Time start_time_;

    //std::vector<std::pair<double, double>> waypoints_;
    //    int current_waypoint_index_;
    
    // Desired positions for the submarine
    double desired_x_ = 0.0;
double desired_y_ = 0.0;
double desired_yaw_ = 0.0;


void desired_x_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    desired_x_ = msg->data;
}

void desired_y_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    desired_y_ = msg->data;
}

void desired_yaw_callback(const std_msgs::msg::Float64::SharedPtr msg) {
    desired_yaw_ = msg->data;
}


            
    //double null_matrix_ [3][3];
        rclcpp::Time last_time_;
       



        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_surface_;
    
        // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_submarine_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_submarine_desired_x_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_submarine_desired_y_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_submarine_desired_yaw_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_submarine_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_submarine_fin_pitch_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_submarine_fin_yaw_;
    rclcpp::Publisher<message_type::msg::Traj>::SharedPtr pub_trajectory_desired_;
    

      
      double simulation_duration_=120.0;
        void submarine_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            rclcpp::Time now = this->now();
    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (dt < 1e-6) return;

    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;
    double u = msg->twist.twist.linear.x;
    double v = msg->twist.twist.linear.y;
    double r = msg->twist.twist.angular.z;
    double yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);

    //  Now use the stored user inputs
    double x_d = desired_x_;
    double y_d = desired_y_;
    double yaw_d = desired_yaw_;

    OuterBack outer(0.6, 0.3);
    InnerBack inner(0.8, 0.4);

    auto ref = outer.compute(x_d, y_d, yaw_d, current_x, current_y, yaw, u, v, r);
    auto [thrust, fin] = inner.compute(u, v, r, ref);

    // Publish Actuation
    std_msgs::msg::Float64 thrust_msg;
    thrust_msg.data = std::clamp(thrust, -15.0, 15.0);
    pub_submarine_thrust_->publish(thrust_msg);

    std_msgs::msg::Float64 fin_yaw_msg;
    fin_yaw_msg.data = std::clamp(fin, -0.42, 0.42);
    pub_submarine_fin_yaw_->publish(fin_yaw_msg);

    RCLCPP_INFO(this->get_logger(),
        "Desired:(%.2f, %.2f, %.2f) Current:(%.2f, %.2f) Thrust:%.2f YawFin:%.2f",
        x_d, y_d, yaw_d, current_x, current_y,
        thrust_msg.data, fin_yaw_msg.data);

        }
        
        
 };

        

    
    //  Main Function
int main(int argc, char *argv[]) {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<MultiSubscriberNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    } 
    
    



