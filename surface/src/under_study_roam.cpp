//IMplementation of LInear ROAM


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


double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &quat){
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;

}

class MultiSubscriberNode : public rclcpp::Node {
    public:
        MultiSubscriberNode()
            : Node("multi_subscriber_node"),
              pid_wamv_x_(0.9, 0.5, 0.3),  // PID for WAM-V forward motion
              pid_wamv_yaw_(0.9, 0.5, 0.3) ,// PID for WAM-V yaw control
              pid_submarine_x_(1.0, 0.4, 0.3),
              pid_submarine_y_(1.0, 0.0, 0.1),
         
              current_waypoint_index_(0),
              null_matrix_{0}
        {
    
             waypoints_ = {
                {100.0, 0.0},
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
            std::vector <double> last_obstacle_normal_ = {0};  // Assume obstacle in front initially
    
    
            last_time_ = this->now();
            std::vector <double> last_obstacle_pos_={0};
            
        }
    
    private:

    std::vector<Obstacle> obstacles;  // Member variable to store obstacles
    std::vector <std::pair <double,double>> velocity_sum_;
    std::vector <double> weigh;
   
        double magn_original;
    
        std::vector<std::pair<double, double>> waypoints_;
        double null_matrix_ [3][3];
        rclcpp::Time last_time_;
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
    std::vector <double> last_obstacle_normal_;
    double safe_distance_ = 4.0;  // Example safe distance
    std::vector <double> last_obstacle_pos_={0};
    
    
    
        // PID Controllers for WAM-V
        PIDController pid_wamv_x_;
        PIDController pid_wamv_yaw_;
        PIDController pid_submarine_x_;
        PIDController pid_submarine_y_;
    
        // Desired Position & Yaw for WAM-V
        double desired_wamv_x_;
        double desired_wamv_y_;
        double desired_wamv_yaw_;
    
        void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            double threshold = 2.5; // Define obstacle separation threshold
            std::vector<double> lidar_data(msg->ranges.begin(), msg->ranges.end());
            
            obstacles = extract_obstacles(lidar_data, threshold, msg->angle_min, msg->angle_increment);
        
            std::cout << "N" << std::endl;
            for (const auto& obs : obstacles) {
             std::cout << " " << "\n";
                std::cout << ""  << " "  << "\n";
            }
        }
      //  std::cout<<"Size of vector Holding Obstacle Information"<<obstacles.size()<<std::endl;
       // RCLCPP_INFO(this->get_logger(),"Obstacle Distance [%.2f]",obstacles.distance);
     

        //obstacles.size()
        void wamv_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            rclcpp::Time now = this->now();
            double dt = (now - last_time_).seconds();
            last_time_ = now;
    
            if (dt < 1e-6) return;  // Prevent zero division
    
            // Extract Current Position & Yaw of WAM-V
            double current_x = msg->pose.pose.position.x;
            double current_y = msg->pose.pose.position.y;
            double current_yaw = get_yaw_from_quaternion(msg->pose.pose.orientation);
           /* For Leader Follower
            desired_submarine_x_ =current_x-4;
            desired_submarine_y_ =current_y-4;
            */
           // For individual
           desired_submarine_x_=40;
           desired_submarine_y_=0;
            desired_wamv_x_=waypoints_[current_waypoint_index_].first;
            desired_wamv_y_=waypoints_[current_waypoint_index_].second;
            // Compute Forward Motion Error
            double error_x = sqrt((desired_wamv_y_ - current_y)*(desired_wamv_y_ - current_y) +(desired_wamv_x_ - current_x)*(desired_wamv_x_ - current_x));
            double error_in_x=desired_wamv_x_-current_x;
            double error_in_y=desired_wamv_y_-current_y;
            
            double thrust_x = pid_wamv_x_.compute(error_x, dt);
              // Limit thrust
    
    
           desired_wamv_yaw_=atan2(desired_wamv_y_- current_y,desired_wamv_x_- current_x);
    
          // RCLCPP_INFO(this->get_logger(),"desired wamv x,[%.2f],desired wamv y,[%.2f],desired wamv yaw ,[%.2f]",desired_wamv_x_,desired_wamv_y_,desired_wamv_yaw_);
        
        double thrust_yaw_1{0};
        double thrust_yaw_2{0};
        double prev_left_thrust{0};
        double prev_right_thrust{0};
        
           double yaw_error = desired_wamv_yaw_ - current_yaw;
         //  RCLCPP_INFO(this->get_logger(),"current yaw:,[%.2f],yaw_error,[%.2f]",current_yaw,yaw_error);
    
        // 2 is with left
        //1 is with right
        double velocity_for_yaw_control=pid_wamv_yaw_.compute(yaw_error,dt);
          
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
              //  RCLCPP_INFO(this->get_logger(), "Reached waypoint, moving to next: [%d]", current_waypoint_index_);
            }
            thrust_x = std::clamp(thrust_x, -12.0, 12.0);
            thrust_yaw_2=std::clamp(thrust_yaw_2, -5.0, 5.0);
            thrust_yaw_1=std::clamp(thrust_yaw_1, -5.0, 5.0);

            //Smoothening the thrust

            double max_delta = 2.0; // Maximum change per cycle
            left_thrust = std::clamp(left_thrust, prev_left_thrust - max_delta, prev_left_thrust + max_delta);
            right_thrust = std::clamp(right_thrust, prev_right_thrust - max_delta, prev_right_thrust + max_delta);

            prev_left_thrust = left_thrust;
            prev_right_thrust = right_thrust;

    
            // Publish Control Commands
    
            auto thrust_msg = std_msgs::msg::Float64();
            thrust_msg.data = left_thrust;
            pub_wamv_left_thrust_->publish(thrust_msg);
    
            auto thrust_msg_1 = std_msgs::msg::Float64();
            thrust_msg_1.data = right_thrust;
            pub_wamv_right_thrust_->publish(thrust_msg_1);
           
           
            
            //RCLCPP_INFO(this->get_logger(), "WAMV Position: [%.2f], Yaw: [%.2f]", current_x, current_yaw);
          //  RCLCPP_INFO(this->get_logger(), "WAMV Thrusters = [%.2f], left_yaw:[%.2f],right_yaw:[%.2f]", thrust_x,  thrust_yaw_2, thrust_yaw_1);
        }
        
        std::vector <double> direction_space_per_obstacle (std::vector<double> veloci,std::vector <double> last_obstacle_normal){
            
        double magnitude=sqrt(magni_tude(last_obstacle_normal));
        if (magnitude>0){
        RCLCPP_INFO(this->get_logger(),"DIrectional Space Caluclation Working");
            //Find Unit Vector
            //Vector by It's Magnitude
            
            
            for (size_t i = 0; i < last_obstacle_normal.size(); ++i) {
                last_obstacle_normal[i] /= magnitude;
            }
                      std::vector <double> unit_vector= last_obstacle_normal;
        
                           for (size_t i=0;i<3;i++){
            null_matrix_[i][0]=unit_vector[i];
           }
        
           std::vector<double> secondi ={}; 
           if (unit_vector[0]<1.2 &&unit_vector[0] >0.8){
            secondi ={0,1,0}; 
           }
           else{
           secondi ={1,0,0}; 
           }
        //Find a vector which is perpendicular to first column vector.
           std::vector<double> second=cross_product(unit_vector,secondi);
         
        //Find a vector which is perpendicular to first and second column vector.
           std::vector<double> third=cross_product(unit_vector,second);
           if(magni_tude(second)>1e-6){
            second=normalize(second);}
          
            if(magni_tude(third)>1e-6){
           third=normalize(third);
            }
           for (size_t i=0;i<3;i++){
            null_matrix_[i][1]=second[i];
           }
        
           for (size_t i=0;i<3;i++){
            null_matrix_[i][2]=third[i];
           }
        
        //Trnaspose of Null Matrix
           double transpose_null_matrix[3][3]={{null_matrix_[0][0],null_matrix_[1][0],null_matrix_[2][0]},{null_matrix_[0][1],null_matrix_[1][1],null_matrix_[2][1]},{null_matrix_[0][2],null_matrix_[1][2],null_matrix_[2][2]}};
        
        
        double nmm=sqrt(magni_tude(veloci));
        if (nmm > 1e-6) {
        for(size_t i=0;i<3;i++){
            veloci[i]=veloci[i]/nmm;
        };
        }
        
            //Dot Product with Global Vector
        

            std::vector <double> directional_space= matrixVectorMultiply(transpose_null_matrix,veloci);
           directional_space=normalize(directional_space);
         //  RCLCPP_INFO(this->get_logger(),"directional space vector  components [%.2f]--[%.2f]--[%.2f]",directional_space[0],directional_space[1],directional_space[2]);
           double a_1 = acos(std::clamp(directional_space[0], -1.0, 1.0));

          //  double a_1=acos(directional_space[0]);
          //  RCLCPP_INFO(this->get_logger(),"directional space first element [%.2f]",directional_space[0]);
            std::vector<double> sub_vector={directional_space[1], directional_space[2]};
            double a_2_1= magni_tude(sub_vector);
          //  double a_3_1=magni_tude(directional_space[2]);
          RCLCPP_INFO(this->get_logger(),"Magnitude of Sub vector [%.2f]",a_2_1);
            double b_2{0};
            double b_3{0};
          if (std::abs(a_2_1) > 1e-6) {
             b_2 = directional_space[1] / a_2_1;
             b_3 = directional_space[2] / a_2_1;
        } else {
             b_2 = 0;
             b_3 = 0;
        }
        
            std::vector<double> fn={b_2 , b_3};

           std::vector <double> dir_spc =scale(a_1,fn);
                
        return dir_spc;
        
        }
        else{
        std::vector <double> no={0};
        return no;
        }
        }


        std::vector <double> inverse_direction_space_per_obstacle (std::vector<double> veloci,std::vector <double> last_obstacle_normal){
            std::vector < double> last_ =last_obstacle_normal;
            double sum=0;
        
            for(double  com: last_){
                com=com*com;
                sum=sum+com;
               
            }
            
        double magnitude=sqrt(sum);
        if (magnitude>0){

            double b_2 = last_obstacle_normal[0] / magnitude;
            double b_3 = last_obstacle_normal[1] / magnitude;

            double v1 = acos(std::clamp(magnitude, -1.0, 1.0));
            double v2 = b_2 * magnitude;
            double v3 = b_3 * magnitude;
            std::vector <double> recovered ={v1,v2,v3};
        
            //Find Unit Vector
            //Vector by It's Magnitude
            for (size_t i = 0; i < recovered.size(); ++i) {
                recovered[i] /= magnitude;
            }
          
            std::vector <double> unit_vector= recovered;
        
            // This will be first column of null_matrix)which is ortthogonal
          // double null_matrix[3][3]  ={0};
        
        
           for (size_t i=0;i<3;i++){
            null_matrix_[i][0]=unit_vector[i];
           }
        
        
        
           std::vector<double> secondi ={}; 
           if (unit_vector[0]<1.2 &&unit_vector[0] >0.8){
            secondi ={0,1,0}; 
           }
           else{
           secondi ={1,0,0}; 
           }
        //Find a vector which is perpendicular to first column vector.
           std::vector<double> second=cross_product(unit_vector,secondi);
          
         
        //Find a vector which is perpendicular to first and second column vector.
           std::vector<double> third=cross_product(unit_vector,second);
           if(magni_tude(second)>0){
            second=normalize(second);}
          
            if(magni_tude(third)>0){
           third=normalize(third);
            }
           for (size_t i=0;i<3;i++){
            null_matrix_[i][1]=second[i];
           }
        
           for (size_t i=0;i<3;i++){
            null_matrix_[i][2]=third[i];
           }
        /*
           for(size_t i=0;i<3;i++){
            for(size_t j=0;j<3;j++){
                cout<<"Index "<<i<<j<<"  "<<null_matrix[i][j]<<endl;
            }
           }
        */
        //Trnaspose of Null Matrix
       //    double transpose_null_matrix[3][3]={{null_matrix_[0][0],null_matrix_[1][0],null_matrix_[2][0]},{null_matrix_[0][1],null_matrix_[1][1],null_matrix_[2][1]},{null_matrix_[0][2],null_matrix_[1][2],null_matrix_[2][2]}};
        
        /*
            for(size_t i=0;i<3;i++){
                for(size_t j=0;j<3;j++){
                    cout<<"Index "<<i<<j<<"  "<<transpose_null_matrix[i][j]<<endl;
                }
            };
        */
        
        double nmm=sqrt(magni_tude(veloci));
        for(size_t i=0;i<3;i++){
            veloci[i]=veloci[i]/nmm;
        };
        
            //Dot Product with Global Vector
            //To get Directional Space

             magn_original =   magni_tude(veloci);
            // RCLCPP_INFO(this->get_logger(),"Orginal magnitude [%.2f]",magn_original);
            std::vector <double> directional_space= matrixVectorMultiply(null_matrix_,veloci);
            //if(magni_tude(directional_space)>0){
            //directional_space=(directional_space);}
            
        directional_space=normalize(directional_space);
        
         //   std::vector <double> conv_vector={0}; 
          //  if(magni_tude(v_d)>0){
           //     conv_vector =normalize(second);}
        
           
          //  double a_3_1=magni_tude(directional_space[2]);

           
                
        return directional_space;
        
        }
        else{
        std::vector <double> no={0};
        return no;
        }
        }
        
        std::vector <double> weighted (std::vector <double> vec,double weigh){
            for (size_t i=0;i<3;i++){
                vec[i]=vec[i]*weigh;
            }
        return vec;
        }

        std::vector<double> computePseudoTangent(
            std::vector <double>  n, // normal vector
            std::vector <double>  c, // convergence 
            std::vector <double>  r, // reference 
            double Re
        ) {
            std::vector <double> k_neg_n_c = direction_space_per_obstacle(n,c);
         //   RCLCPP_INFO(this->get_logger(),"direcrional space neg_normal_covergence [%.2f]--[%.2f]",k_neg_n_c[0],k_neg_n_c[1]);
            double norm_k_neg_n_c = magni_tude(k_neg_n_c);
        
            if (norm_k_neg_n_c >= Re) {
                return c; // No rotation needed, e = c
            }
        
            std::vector <double> k_neg_n_r = direction_space_per_obstacle(n,r);

         //    RCLCPP_INFO(this->get_logger(),"direcrional space neg_normal_reference [%.2f]--[%.2f]",k_neg_n_r[0],k_neg_n_r[1]);
            std::vector<double> k_neg_n_e(k_neg_n_c.size());
            double b = 0.0; // Solve for b such that || k(-n, e) || = Re
        
            double norm_k_neg_n_r = magni_tude(k_neg_n_r);

            if (std::abs(norm_k_neg_n_r - norm_k_neg_n_c) < 1e-6) {
                b = 0.5;
            } 
            else if((norm_k_neg_n_c - norm_k_neg_n_r)==0){
                b=0.5;
            }
            
            else {
                b = (Re - norm_k_neg_n_r) / (norm_k_neg_n_c - norm_k_neg_n_r);
            }
            
            /*
        
            if (norm_k_neg_n_r != norm_k_neg_n_c) {
                b = (Re - norm_k_neg_n_r) / (norm_k_neg_n_c - norm_k_neg_n_r);
            } else {
                b = 0.5; // Arbitrary choice if both norms are equal
            }
                */
        
            // Compute k(-n, e)
            for (size_t i = 0; i < k_neg_n_c.size(); i++) {
                k_neg_n_e[i] = (1 - b) * k_neg_n_r[i] + b * k_neg_n_c[i];
            }
        
            // Normalize to ensure ||k(-n, e)|| = Re
            double norm_k_neg_n_e = magni_tude(k_neg_n_e);
            for (size_t i = 0; i < k_neg_n_e.size(); i++) {
                if(norm_k_neg_n_e==0){
                    k_neg_n_e[i]=0;
                }
                else{
                k_neg_n_e[i] *= (Re / norm_k_neg_n_e);}
            }
        
            return k_neg_n_e;
        }
        
        std::vector<double> rotateConvergence(
            const std::vector<double>& k_c_f,  // k(c, f) -> initial dynamics
            const std::vector<double>& k_c_e,  // k(c, e) -> pseudo-tangent
            double gamma,                      // Distance-based scaling factor Γ(ξ)
            double R_e,                         // Defined rotation radius (between π/2 and π)
            const std::vector<double>& k_n_r,   // k(-n, r)
            const std::vector<double>& k_n_c ,
            const std::vector <double>& c  // k(-n, c)
                 ) {
            // Compute λ(ξ)
            double R_r = std::min(R_e - norm(k_n_r), M_PI / 2);
            double delta_k_c = norm(k_n_r) - norm(k_n_c);
            double q = std::max(1.0, std::pow(R_r / delta_k_c, 2.0)); // s is assumed 2.0
            double lambda = std::pow(1.0 / gamma, q);

            // Ensure λ is within [0,1]
            lambda = std::max(0.0, std::min(1.0, lambda));

            // Compute the rotated dynamics
            std::vector<double> rotated_k = weightedSum(k_c_f, k_c_e, lambda);
//What about inverse directional space mapping
            // Normalize the scale direction
            std::vector <double>  final_rotated =   inverse_direction_space_per_obstacle(c,rotated_k);

            return final_rotated;
        }
        double speed_scaling (double Re, double gamma, const std::vector<double>& k_n_r,   // k(-n, r)
            const std::vector<double>& k_n_c ){
                double R_r = std::min(Re - norm(k_n_r), M_PI / 2);
                double delta_k_c = norm(k_n_r) - norm(k_n_c);
                double lambda = std::pow(1.0 / gamma, 2);

                double fir =std::pow((delta_k_c/R_r),2);
                double sec= std::pow((1-(1/lambda)),2);
                double thi =fir+sec;
                double four =std::min(1.0,thi);

                return four;
            }
        void submarine_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            rclcpp::Time now = this->now();
            double dt = (now - last_time_).seconds();
            last_time_ = now;
    
            if (dt < 1e-6) return;  // Prevent zero division
    
            // Extract Current Position
            double current_x = msg->pose.pose.position.x;
            double current_y = msg->pose.pose.position.y;

            desired_submarine_x_=40;
            desired_submarine_y_=0;
            double submarine_error_x = desired_submarine_x_ -current_x ;
            double submarine_error_y =desired_submarine_y_ -current_y ;
            RCLCPP_INFO(this->get_logger(),"Desired Position [%.2f],[%.2f]",desired_submarine_x_,desired_submarine_y_);
            RCLCPP_INFO(this->get_logger(),"Error of submarine [%.2f],[%.2f]",submarine_error_x,submarine_error_y);
            double v_x=pid_submarine_x_.compute(submarine_error_x,dt);
            double v_y=pid_submarine_y_.compute(submarine_error_y,dt);
            RCLCPP_INFO(this->get_logger(), "Velocity in X from PID: [%.2f]", v_x);

            std::vector<double> global_velocity ={v_x,v_y,0};
            std::vector<double> convergence_vector ={-submarine_error_x,-submarine_error_y,0};
           // RCLCPP_INFO(this->get_logger(),"Convergence Vector[%.2f]--[%.2f]-",convergence_vector[0],convergence_vector[1]);
            

          //  for (const auto& obs : obstacles) {
         //       RCLCPP_INFO(this->get_logger(), "Obstacle Distance [%.2f]", obs.distance);
         //   };
          //  RCLCPP_INFO(this->get_logger(),"Size of vector of obstacles [%.2f]",obstacles[].size());
        //  if (!obstacles.empty()) {
            //RCLCPP_INFO(this->get_logger(), "Obstacle Distance: [%.2f]", obstacles[0].distance);
            
                      //  RCLCPP_INFO(this->get_logger(), "No . of Obstacles", obstacles.size());
          //              std::cout << "Number of obstacles: " << obstacles.size() << std::endl;
           //  for (const auto& obs : obstacles) {
             //       std::cout << "Obstacle at distance: " << obs.distance << "\n";
              //        std::cout << "Obstacle normal: [" << obs.normal[0] << ", " << obs.normal[1] << "]\n";
               // }
                        

            // Implemntation of ROAM per obstacle
            // Obstacle Normal(obstacles[0].normal) and  Vector
            //Compute pseudoTangent
             // normal vector
             // convergence 
             // reference 
          // Re

        if(obstacles.size() >=1){
            RCLCPP_INFO(this->get_logger(), "Linear ROAM");
          std::vector <double> neg_normal={0,0,0};

          for(size_t i = 0;i<2;i++){
            neg_normal[i] = -obstacles[0].normal[i];
          };

        //  RCLCPP_INFO(this->get_logger(),"NEgative Normal [%.2f]-- [%.2f]",neg_normal[0],neg_normal[1]);

          double Re=M_PI/2;
          double gamma=2;

          for(size_t i=0;i<obstacles.size();i++){
          std::vector<double> reference_vector = {obstacles[i].center_x - current_x, obstacles[i].center_y - current_y, 0};
         // RCLCPP_INFO(this->get_logger(),"Reference Vector [%.2f]--[%.2f]",reference_vector[0],reference_vector[1]);
          std::vector<double> pseudo_tangent=computePseudoTangent(neg_normal,convergence_vector,reference_vector,Re);
        //  RCLCPP_INFO(this->get_logger(),"Pseudo Tangent [%.2f]--[%.2f]--[%.2f]", pseudo_tangent[0],pseudo_tangent[1] );
        // Rotating Towards Tangent Direction

       std::vector <double> rotated= rotateConvergence(direction_space_per_obstacle(convergence_vector,convergence_vector),direction_space_per_obstacle(convergence_vector,pseudo_tangent),
        gamma,Re,direction_space_per_obstacle(neg_normal,reference_vector),direction_space_per_obstacle(neg_normal,convergence_vector),convergence_vector);
       // RCLCPP_INFO(this->get_logger(),"Rotated Towards Tangent Direction [%.2f]--[%.2f]--[%.2f]",rotated[0],rotated[1],rotated[2]);
            //Evaluation of Speed

        double slk=speed_scaling(Re,gamma,direction_space_per_obstacle(neg_normal,reference_vector),direction_space_per_obstacle(neg_normal,convergence_vector));
        //    RCLCPP_INFO(this->get_logger(),"Speed Scaling",slk);
        global_velocity=scale(slk,rotated);

       weigh={0};
       weigh.resize(obstacles.size());

           weigh[i] =1/obstacles[i].distance;
           velocity_sum_.resize(obstacles.size());

           velocity_sum_[i]={global_velocity[0],global_velocity[1]};
           };
          
        
        
          //Weighted Average
          double x_sum{0};
          double y_sum{0};
          double w_sum{0};

          for(size_t i=0 ;i<obstacles.size();i++){
           
           x_sum  = x_sum +(weigh[i]*velocity_sum_[i].first);
           y_sum = y_sum +(weigh[i]*velocity_sum_[i].second);
           w_sum =w_sum+weigh[i];
           if(w_sum!=0){
           global_velocity[0] =x_sum/w_sum;
           global_velocity[1]=y_sum/w_sum;
           }
           else{
            global_velocity[0] =x_sum;
            global_velocity[1]=y_sum;
           }


        

          };

          RCLCPP_INFO(this->get_logger(),"PID Velocity Vector [%.2f]--[%.2f]",v_x,v_y);
          //RCLCPP_INFO(this->get_logger()."Convergence Vector [%.2f]--[%.2f]",convergence_vector[0],convergence_vector[1]);
          
        
         // RCLCPP_INFO(this->get_logger(), "Weighted Rotated Velocity Vector : [%.2f]--[%.2f]--[%.2f]", global_velocity[0],global_velocity[1],global_velocity[2]);
                    
            double yaw_angle = 0.0;

            if (std::isfinite(global_velocity[0]) && std::isfinite(global_velocity[1])) {
                if (global_velocity[0] != 0.0 || global_velocity[1] != 0.0) {
                    yaw_angle = atan2(global_velocity[1], global_velocity[0]);
                } else {
                    yaw_angle = 0.0; // Default when both are zero
                }
            }

            double v_max = 10.0;
            double velocity_magnitude = sqrt(global_velocity[0] * global_velocity[0] + 
                                            global_velocity[1] * global_velocity[1]);

            // Protect against division by zero and NaN cases
            if (velocity_magnitude > 1e-6) { // Small threshold to avoid instability
                double ss = v_max / velocity_magnitude;

                if ((std::isfinite(ss)) && (velocity_magnitude> v_max)) {
                    global_velocity[0] *= ss;
                    global_velocity[1] *= ss;
                    
            }


       //   RCLCPP_INFO(this->get_logger(),"Final Data Being Published after [%.2f]--[%.2f]",global_velocity[0],yaw_angle);

    //PUBLISHINING
             double thrust_sub_x = - global_velocity[0];
            thrust_sub_x=std::clamp(thrust_sub_x,-5.0,5.0);
            // Publish Modified Thrust
            std_msgs::msg::Float64 thrust_msg;
            thrust_msg.data = thrust_sub_x;
            pub_submarine_thrust_->publish(thrust_msg);
    
           RCLCPP_INFO(this->get_logger(), "ROAM 's Result -- Modulated Thrust X: [%.2f]", thrust_sub_x);
    
    
           double fin_position=yaw_angle;
            fin_position = std::clamp(fin_position, -3.57, 3.57);
    
            // Publish Fin Control
            std_msgs::msg::Float64 fin_msg;
            fin_msg.data = -fin_position;
            pub_submarine_fin_->publish(fin_msg);
    
           RCLCPP_INFO(this->get_logger(), "ROAM 's Result  Fin Position: [%.2f]",  fin_position);
        
        }
    }
        
        else{

            RCLCPP_INFO(this->get_logger(), "PID Working");
            double yaw_angle= atan2(v_y,v_x);

            double v_max =10.0;
   
           // double velocity_magnitude=sqrt((v_x*v_x)+(v_y*v_y));
           // if (velocity_magnitude > v_max) {
            // double ss= v_max / velocity_magnitude;
            // v_x =v_x*ss;
            // yaw_angle = yaw_angle;}
             RCLCPP_INFO(this->get_logger(), "Velocity in X: [%.2f]", v_x);
            double thrust_sub_x = - v_x;
            thrust_sub_x=std::clamp(thrust_sub_x,-5.0,5.0);

            std_msgs::msg::Float64 thrust_msg;
            thrust_msg.data = thrust_sub_x;
            pub_submarine_thrust_->publish(thrust_msg);
    
          // RCLCPP_INFO(this->get_logger(), "Modulated Thrust X: [%.2f]", thrust_sub_x);
    
    
           double fin_position=yaw_angle;
            fin_position = std::clamp(fin_position, -3.57, 3.57);
    
           //  Publish Fin Control
            std_msgs::msg::Float64 fin_msg;
            fin_msg.data = -fin_position;
            pub_submarine_fin_->publish(fin_msg);
    
         //  RCLCPP_INFO(this->get_logger(), " Fin Position: [%.2f]",  fin_position);
               
        }
        
        
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
    
    



