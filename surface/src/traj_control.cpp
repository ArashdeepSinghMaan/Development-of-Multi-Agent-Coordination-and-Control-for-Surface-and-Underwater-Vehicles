#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/msg/laser_scan.hpp>
//LEader Follower with Trajectory while Follower has DMM also


// Extract yaw from quaternion
double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &quat) {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

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
          pid_wamv_x_(1.0, 0.2, 0.3),  // PID for WAM-V forward motion
          pid_wamv_yaw_(1.0, 0.2, 0.3) ,// PID for WAM-V yaw control
          pid_submarine_x_(1.0, 0.2, 0.3),
          pid_submarine_y_(1.0, 0.2, 0.3),
          current_waypoint_index_(0)
    {

         waypoints_ = {
            {20.0, 0.0},
            {30.0, 20.0},
            {0.0, 20.0},
            {0.0, -20.0}
        };
        // Subscribe to submarine odometry
        sub_submarine_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/my_lrauv_modified/submarine/odometry", 10,
            std::bind(&MultiSubscriberNode::submarine_callback, this, std::placeholders::_1));

        // Subscribe to front LiDAR sensor
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/my_lrauv_modified/submarine/Laser_scan_front", 10,
            std::bind(&MultiSubscriberNode::lidar_callback, this, std::placeholders::_1));

        // Initialize Publishers
        pub_submarine_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/my_lrauv_modified/propeller/thrust", 10);
        pub_submarine_fin_ = this->create_publisher<std_msgs::msg::Float64>("/my_lrauv_modified/submarine/horizontal/fin/pos", 10);

        // Set Desired Positions
        desired_submarine_x_ = 0.0;
        desired_submarine_y_ = 0.0;

        
      
        // Subscribe to WAM-V odometry
        sub_surface_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/wamv/ground_truth/odometry", 10,
            std::bind(&MultiSubscriberNode::wamv_callback, this, std::placeholders::_1));

        // Initialize WAM-V Publishers
        pub_wamv_left_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/thrust", 10);
        pub_wamv_right_thrust_ = this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/thrust", 10);
        pub_wamv_left_angle_=this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/left/angle",10);
        pub_wamv_right_angle_=this->create_publisher<std_msgs::msg::Float64>("/wamv/thrusters/right/angle",10);
        // Set Desired Position for WAM-V
        desired_wamv_x_ = 0.0;
        desired_wamv_y_ = 0.0;
        desired_wamv_yaw_ = 0.0;

        last_obstacle_dist_ = std::numeric_limits<double>::infinity();  // Start with a large value
        last_obstacle_normal_ = Eigen::Vector3d(0.0, 0.0, 0.0);  // Assume obstacle in front initially


        last_time_ = this->now();
         last_obstacle_pos_=Eigen::Vector3d(0.0, 0.0, 0.0);
    }

private:

    std::vector<std::pair<double, double>> waypoints_;
    int current_waypoint_index_;
    // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_surface_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_wamv_left_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_wamv_right_thrust_;
     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_wamv_left_angle_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_wamv_right_angle_;
    // Subscribers
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_submarine_;
rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_;

// Publishers
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_submarine_thrust_;
rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_submarine_fin_;

// Desired positions for the submarine
double desired_submarine_x_;
double desired_submarine_y_;

// Obstacle avoidance parameters
double last_obstacle_dist_;
Eigen::Vector3d last_obstacle_normal_;
double safe_distance_ = 3.0;  // Example safe distance
Eigen::Vector3d last_obstacle_pos_;



    // PID Controllers for WAM-V
    PIDController pid_wamv_x_;
    PIDController pid_wamv_yaw_;
    PIDController pid_submarine_x_;
    PIDController pid_submarine_y_;

    // Desired Position & Yaw for WAM-V
    double desired_wamv_x_;
    double desired_wamv_y_;
    double desired_wamv_yaw_;

    rclcpp::Time last_time_;

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        double min_distance = std::numeric_limits<double>::max();
        Eigen::Vector3d obstacle_normal(0, 0, 0);
        Eigen::Vector3d obstacle_position(0, 0, 0);

        for (size_t i = 0; i < msg->ranges.size(); i++) {
            double range = msg->ranges[i];
            if (range < min_distance && range > msg->range_min) {
                min_distance = range;
                double angle = msg->angle_min + i * msg->angle_increment;
                obstacle_position.x() = range * cos(angle);
                obstacle_position.y() = range * sin(angle);
                obstacle_normal = obstacle_position.normalized();
             //   RCLCPP_INFO(this->get_logger(), "Obstacle Detected: [%.2f]", min_distance);
              //  RCLCPP_INFO(this->get_logger(), "Obstacle Normal: [%.2f, %.2f, %.2f]", 
          //  obstacle_normal.x(), obstacle_normal.y(), obstacle_normal.z());
//;;
            }
        }

        last_obstacle_pos_ = obstacle_position;
        last_obstacle_normal_ = obstacle_normal;
        last_obstacle_dist_ = min_distance;
    }


    // WAM-V Callback - Controls WAM-V Movement
    void wamv_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt < 1e-6) return;  // Prevent zero division

        // Extract Current Position & Yaw of WAM-V
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;
        double current_yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);
       
        desired_submarine_x_ =current_x-4;
        desired_submarine_y_ =current_y-4;
        
        desired_wamv_x_=waypoints_[current_waypoint_index_].first;
        desired_wamv_y_=waypoints_[current_waypoint_index_].second;
        // Compute Forward Motion Error
        double error_x = sqrt((desired_wamv_y_ - current_y)*(desired_wamv_y_ - current_y) +(desired_wamv_x_ - current_x)*(desired_wamv_x_ - current_x));
        double error_in_x=desired_wamv_x_-current_x;
        double error_in_y=desired_wamv_y_-current_y;
        
        double thrust_x = pid_wamv_x_.compute(error_x, dt);
          // Limit thrust


       desired_wamv_yaw_=atan2(desired_wamv_y_,desired_wamv_x_);

       RCLCPP_INFO(this->get_logger(),"desired wamv x,[%.2f],desired wamv y,[%.2f],desired wamv yaw ,[%.2f]",desired_wamv_x_,desired_wamv_y_,desired_wamv_yaw_);
    
    double thrust_yaw_1{0};
    double thrust_yaw_2{0};
    
       double yaw_error = desired_wamv_yaw_ - current_yaw;
       RCLCPP_INFO(this->get_logger(),"current yaw:,[%.2f],yaw_error,[%.2f]",current_yaw,yaw_error);

    // 2 is with left
    //1 is with right
    double velocity_for_yaw_control=1.5*pid_wamv_yaw_.compute(yaw_error,dt);
      
      if(yaw_error>0){
         if (yaw_error<0.70){
        thrust_yaw_1 =2.5*velocity_for_yaw_control;
         thrust_yaw_2= -2.5*velocity_for_yaw_control;}
         else if (yaw_error>0.70){
        thrust_yaw_1 =3.9*velocity_for_yaw_control;
         thrust_yaw_2= -3.9*velocity_for_yaw_control;}
      }
      else if(yaw_error<0){
         if (yaw_error > -0.70){
        thrust_yaw_1 =-2.5*velocity_for_yaw_control;
         thrust_yaw_2= 2.5*velocity_for_yaw_control;}
         else if (yaw_error< -0.70){
        thrust_yaw_2 =3.9*velocity_for_yaw_control;
         thrust_yaw_1= -3.9*velocity_for_yaw_control;}
         
      }

        if( yaw_error<=0.005 && yaw_error>=-0.005){
            thrust_yaw_1=0;
            thrust_yaw_2=0;
        }
      
        double left_thrust = thrust_x+thrust_yaw_2;
        double right_thrust = thrust_x+thrust_yaw_1 ;
        if(abs(error_in_x)<=0.7 && abs(error_in_y)<=0.7 ){
            left_thrust=left_thrust/2;
            right_thrust=right_thrust/2;
            current_waypoint_index_ = (current_waypoint_index_ + 1) % waypoints_.size();
            RCLCPP_INFO(this->get_logger(), "Reached waypoint, moving to next: [%d]", current_waypoint_index_);
        }
        thrust_x = std::clamp(thrust_x, -12.0, 12.0);
        thrust_yaw_2=std::clamp(thrust_yaw_2, -5.0, 5.0);
        thrust_yaw_1=std::clamp(thrust_yaw_1, -5.0, 5.0);

        // Publish Control Commands

        auto thrust_msg = std_msgs::msg::Float64();
        thrust_msg.data = left_thrust;
        pub_wamv_left_thrust_->publish(thrust_msg);

        auto thrust_msg_1 = std_msgs::msg::Float64();
        thrust_msg_1.data = right_thrust;
        pub_wamv_right_thrust_->publish(thrust_msg_1);
       
       
        
        //RCLCPP_INFO(this->get_logger(), "WAMV Position: [%.2f], Yaw: [%.2f]", current_x, current_yaw);
        RCLCPP_INFO(this->get_logger(), "WAMV Thrusters = [%.2f], left_yaw:[%.2f],right_yaw:[%.2f]", thrust_x,  thrust_yaw_2, thrust_yaw_1);
    }

 

     void submarine_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        rclcpp::Time now = this->now();
        double dt = (now - last_time_).seconds();
        last_time_ = now;

        if (dt < 1e-6) return;  // Prevent zero division

        // Extract Current Position
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;

        double submarine_error_x =desired_submarine_x_ - current_x;
        double submarine_error_y =desired_submarine_y_ - current_y;
        double v_x=8*pid_submarine_x_.compute(submarine_error_x,dt);
        double v_y=8*pid_submarine_y_.compute(submarine_error_y,dt);

        // Compute Desired Velocity Before Modulation
        Eigen::Vector3d v_d(v_x,
                            v_y, 
                            0);
        v_d.normalize();
      //  v_d *= 8.0;  // Desired speed (tune this value)

        // --- Apply Dynamic Modulation Matrix (DMM) ---
        double lambda = 1.0;
        if (last_obstacle_dist_ < safe_distance_) {
            lambda = 1.0 - (last_obstacle_dist_ * last_obstacle_dist_) / (safe_distance_ * safe_distance_);
        }

        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d nnT = last_obstacle_normal_ * last_obstacle_normal_.transpose();
        Eigen::Matrix3d M = I - (1.0 - lambda) * nnT;

        Eigen::Vector3d v_mod = M * v_d;  // Apply DMM to desired velocity
     ///RCLCPP_INFO(this->get_logger(), "Modulated Velocity Vector: [%.2f, %.2f, %.2f]", 
        //   v_mod.x(), v_mod.y(), v_mod.z());
;
        double current_yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);
    
        double thrust_x= -v_mod.x();
        thrust_x = std::clamp(thrust_x, -12.0, 12.0);
/*
        if (std::abs(v_mod.x()) <= 0.8) {
            thrust_x = 0;
        }
*/
        // Publish Modified Thrust
        std_msgs::msg::Float64 thrust_msg;
        thrust_msg.data = thrust_x;
        pub_submarine_thrust_->publish(thrust_msg);

      //  RCLCPP_INFO(this->get_logger(), "Modulated Thrust X: [%.2f]", thrust_x);


       double fin_position=v_mod.y()/10;
        fin_position = std::clamp(fin_position, -6.28, 6.28);

        // Publish Fin Control
        std_msgs::msg::Float64 fin_msg;
        fin_msg.data = -fin_position;
        pub_submarine_fin_->publish(fin_msg);

      // RCLCPP_INFO(this->get_logger(), " Fin Position: [%.2f]",  fin_position);
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
