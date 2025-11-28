//IMplementation of Cascaded Feedback Linearization with PID


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
#include "/home/arash/ros2_ws/src/surface/include/surface/cascaded_control.hpp"
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
            : Node("multi_subscriber_node"),
             cascaded_controller_(
              0.6, 0.1, 0.2,   // Outer surge PID
              2.0, 0.2, 0.4,   // Outer yaw PID
              0.0, 0.0, 0.0,   // Outer pitch PID
              0.6, 0.1, 0.4,   // Inner surge PID
              2.0, 0.2, 0.4,   // Inner yaw PID
              0.0, 0.0, 0.0    // Inner pitch PID
          )
         // current_waypoint_index_(0)
    {
       // waypoints_ = {
        //    {10.0, 0.0},
         //   {10.0, 10.0},
         //   {0.0, 10.0},
         //   {0.0, 0.0}
       // };
          //    pid_submarine_x_(0.7, 0.4, 0.3),
          //    pid_submarine_y_(0.7, 0.4, 0.3),
            //  current_waypoint_index_(0)
           //   null_matrix_{0}
        
    
           //  waypoints_ = {
              //  {100.0, 0.0},
             //   {30.0, 20.0},
             //   {0.0, 20.0},
             //   {0.0, -20.0}
           // };
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
            last_time_ = this->now();
                 
        }
    
    private:

    rclcpp::Time start_time_;

    //std::vector<std::pair<double, double>> waypoints_;
    //    int current_waypoint_index_;
    
    // Desired positions for the submarine
    double desired_submarine_x_;
    double desired_submarine_y_;
    double desired_submarine_z_;
    
            
    //double null_matrix_ [3][3];
        rclcpp::Time last_time_;
       



        // Subscribers
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_surface_;
    
        // Subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_submarine_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_lidar_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_submarine_thrust_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_submarine_fin_pitch_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_submarine_fin_yaw_;
    rclcpp::Publisher<message_type::msg::Traj>::SharedPtr pub_trajectory_desired_;
    

      auv::CascadedAUVController cascaded_controller_;
      double simulation_duration_=120.0;
        void submarine_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            rclcpp::Time now = this->now();
            double dt = (now - last_time_).seconds();
            last_time_ = now;
    double elapsed_time = (now - start_time_).seconds();

if (elapsed_time > simulation_duration_) {
        RCLCPP_INFO(this->get_logger(),
            "Trajectory tracking complete (%.2f s). Holding position.", elapsed_time);

        // Optionally: send zero thrust and neutral fins
        std_msgs::msg::Float64 thrust_msg;
        thrust_msg.data = 0.0;
        pub_submarine_thrust_->publish(thrust_msg);

        std_msgs::msg::Float64 yaw_fin_msg;
        yaw_fin_msg.data = 0.0;
        pub_submarine_fin_yaw_->publish(yaw_fin_msg);

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

            if (dt < 1e-6) return;  // Prevent zero division
    
            // Extract Current Position
            double current_x = msg->pose.pose.position.x;
            double current_y = msg->pose.pose.position.y;
           // double current_z= msg->pose.pose.position.z;
             double u = msg->twist.twist.linear.x;  // Surge Velocity Current
        double r = msg->twist.twist.angular.z;   // Yaw Rate Current
     //   double q = msg->twist.twist.angular.y;

        double yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);
      //  double pitch = get_pitch_from_quaternion(msg->pose.pose.orientation);

        // Select current waypoint
       // desired_submarine_x_ = waypoints_[current_waypoint_index_].first;
       // desired_submarine_y_ = waypoints_[current_waypoint_index_].second;

        // Update to next waypoint if close
       // double distance_to_waypoint = std::hypot(current_x - desired_submarine_x_, current_y - desired_submarine_y_);
       // if (distance_to_waypoint < 1.0) {
      //      current_waypoint_index_ = (current_waypoint_index_ + 1) % waypoints_.size();
       // }

        // Compute control inputs
        double T = 0.0, tau_yaw = 0.0, tau_pitch = 0.0;

       cascaded_controller_.computeControl(
    x_d,              // Desired surge position from trajectory
    desired_yaw,      // From atan2(dy/dt, dx/dt)
  //  desired_pitch,    // From atan2(-dz/dt, sqrt(dx^2 + dy^2))
    current_x, yaw, //pitch,
    u, r, //q,
    dt,
    T, tau_yaw //tau_pitch
);

        // Clamp thrust
        std_msgs::msg::Float64 thrust_msg;
        thrust_msg.data =  std::clamp(T, -2.5, 2.5);
        double published_thrust=thrust_msg.data;
        pub_submarine_thrust_->publish(thrust_msg);

        message_type::msg::Traj traj_msg;
        traj_msg.time = elapsed_time;
        traj_msg.x_d=x_d;
        traj_msg.y_d=y_d;
        pub_trajectory_desired_->publish(traj_msg);

        // Map torques to fin angles
        //double V = std::abs(u);
          double V= u;
        double fin_yaw = auv::computeFinDeflection(tau_yaw, V);
      //  double fin_pitch = auv::computeFinDeflection(tau_pitch, V);

        std_msgs::msg::Float64 fin_yaw_msg;
        fin_yaw_msg.data = fin_yaw;
        double published_yaw=fin_yaw_msg.data;
      //  fin_pitch_msg.data = -std::clamp(fin_pitch, -0.52, 0.52);

        pub_submarine_fin_yaw_->publish(fin_yaw_msg);
      //  pub_submarine_fin_pitch_->publish(fin_pitch_msg);

      RCLCPP_INFO(this->get_logger(),"Calculated Torques From Controller:(%.2f,%.2f),Converted Control Input:(%.2f,%.2f)",T,tau_yaw,T,fin_yaw);

        RCLCPP_INFO(this->get_logger(), "Traj Pos: (%.2f, %.2f), Current: (%.2f, %.2f), Thrust: %.2f, Angle_yaw: %.2f,Desired_yaw:%.2f,Current_yaw:%.2f",
            x_d, y_d, current_x, current_y, published_thrust, published_yaw,desired_yaw,yaw);

    /*
            double submarine_error_x = desired_submarine_x_ -current_x ;
            double submarine_error_y =desired_submarine_y_ -current_y ;
            double submarine_error_z=desired_submarine_z_-current_z;
            
            double v_x=pid_submarine_x_.compute(submarine_error_x,dt);
            double v_y=pid_submarine_y_.compute(submarine_error_y,dt);
            double v_z=pid_submarine_z_.compute(submarine_error_z,dt);
            RCLCPP_INFO(this->get_logger(), "Velocity in X from PID: [%.2f]", v_x);

            
            RCLCPP_INFO(this->get_logger(), "PID Working");
            double yaw_angle= atan2(v_y,v_x);
            double pitch_angle=atan2(v_y,v_x);

            double v_max =10.0;
   
           
             RCLCPP_INFO(this->get_logger(), "Velocity in X: [%.2f]", v_x);
            double thrust_sub_x = - v_x;
            thrust_sub_x=std::clamp(thrust_sub_x,-5.0,5.0);

            std_msgs::msg::Float64 thrust_msg;
            thrust_msg.data = thrust_sub_x;
            pub_submarine_thrust_->publish(thrust_msg);
    
          // RCLCPP_INFO(this->get_logger(), "Modulated Thrust X: [%.2f]", thrust_sub_x);
    
    
           double fin_position=yaw_angle;
            fin_position = std::clamp(fin_position, -3.57, 3.57);
    
           //  Publish Yaw Fin Control
            std_msgs::msg::Float64 fin_msg;
            fin_msg.data = -fin_position;
            pub_submarine_fin_pitch_->publish(fin_msg);
    
         //  RCLCPP_INFO(this->get_logger(), " Fin Position: [%.2f]",  fin_position);

            double ver_fin_position=pitch_angle;
            ver_fin_position=std::clamp(ver_fin_position,-3.57,3.57);

            std_msgs::msg::Float64 ver_fin_msg;
            ver_fin_msg.data= -ver_fin_position;
            pub_submarine_fin_yaw_ ->publish(ver_fin_msg);  
            */


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
    
    



