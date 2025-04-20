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


double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &quat){
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;

}

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
    
    public:
        double kp_, ki_, kd_;
        double prev_error_, integral_;
    };
    
    // MultiSubscriberNode Class Definition
    class MultiSubscriberNode : public rclcpp::Node {
    public:
        MultiSubscriberNode()
            : Node("multi_subscriber_node"),
              pid_wamv_x_(0.9, 0.5, 0.3),  // PID for WAM-V forward motion
              pid_wamv_yaw_(0.9, 0.5, 0.3) ,// PID for WAM-V yaw control
              pid_submarine_x_(0.7, 0.4, 0.3),
              pid_submarine_y_(0.7, 0.4, 0.3),
              current_waypoint_index_(0),
              null_matrix_({{{0}}})
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
        double magn_original;
        std::vector<std::pair<double, double>> waypoints_;
        double null_matrix_ [3][3];
       
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
    double safe_distance_ = 3.0;  // Example safe distance
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
    
        rclcpp::Time last_time_;
        std::vector<double> normalizzze(const std::vector<double>& vec) {
            double magnitude = std::sqrt(std::inner_product(vec.begin(), vec.end(), vec.begin(), 0.0));
            std::vector<double> normalized_vec(vec.size());
            if (magnitude > 0) {
                std::transform(vec.begin(), vec.end(), normalized_vec.begin(),
                               [magnitude](double v) { return v / magnitude; });
            }
            return normalized_vec;
        }

        void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            double threshold = 0.5; // Define obstacle separation threshold
            std::vector<double> lidar_data(msg->ranges.begin(), msg->ranges.end());
            std::vector<Obstacle> obstacles = extract_obstacles(lidar_data, threshold,msg->angle_min, msg->angle_increment);

            for (const auto& obs : obstacles) {
                // Process each obstacle
                std::cout << "Obstacle at distance: " << obs.distance << "\n";
            }
        }

        /*
    
        void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            double min_distance = std::numeric_limits<double>::max();
            std::vector <double> obstacle_normal ={0};
            std::vector <double> obstacle_position={0};
    
            for (size_t i = 0; i < msg->ranges.size(); i++) {
                double range = msg->ranges[i];
                if (range < min_distance && range > msg->range_min) {
                    min_distance = range;
                    double angle = msg->angle_min + i * msg->angle_increment;
                    obstacle_position[0] = range * cos(angle);
                    obstacle_position[1] = range * sin(angle);
                    obstacle_normal = normalizzze(obstacle_position);
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
    
    */
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
           /* For Leader Follower
            desired_submarine_x_ =current_x-4;
            desired_submarine_y_ =current_y-4;
            */
           // For individual
           desired_submarine_x_=20;
           desired_submarine_y_=10;
            desired_wamv_x_=waypoints_[current_waypoint_index_].first;
            desired_wamv_y_=waypoints_[current_waypoint_index_].second;
            // Compute Forward Motion Error
            double error_x = sqrt((desired_wamv_y_ - current_y)*(desired_wamv_y_ - current_y) +(desired_wamv_x_ - current_x)*(desired_wamv_x_ - current_x));
            double error_in_x=desired_wamv_x_-current_x;
            double error_in_y=desired_wamv_y_-current_y;
            
            double thrust_x = pid_wamv_x_.compute(error_x, dt);
              // Limit thrust
    
    
           desired_wamv_yaw_=atan2(desired_wamv_y_- current_y,desired_wamv_x_- current_x);
    
           RCLCPP_INFO(this->get_logger(),"desired wamv x,[%.2f],desired wamv y,[%.2f],desired wamv yaw ,[%.2f]",desired_wamv_x_,desired_wamv_y_,desired_wamv_yaw_);
        
        double thrust_yaw_1{0};
        double thrust_yaw_2{0};
        double prev_left_thrust{0};
        double prev_right_thrust{0};
        
           double yaw_error = desired_wamv_yaw_ - current_yaw;
           RCLCPP_INFO(this->get_logger(),"current yaw:,[%.2f],yaw_error,[%.2f]",current_yaw,yaw_error);
    
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
                RCLCPP_INFO(this->get_logger(), "Reached waypoint, moving to next: [%d]", current_waypoint_index_);
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
            RCLCPP_INFO(this->get_logger(), "WAMV Thrusters = [%.2f], left_yaw:[%.2f],right_yaw:[%.2f]", thrust_x,  thrust_yaw_2, thrust_yaw_1);
        }
        std::vector <double> cross_product(std::vector <double>  first, std::vector <double> doja){

            return{
                first[1]*doja[2] -first[2]*doja[1],
                first[2]*doja[0] -first[0]*doja[2],
                first[0] *doja[1] -first[1]*doja[0]
            };
        }
        double magni_tude(std::vector <double> first){
            
            double val=sqrt(first[0]*first[0] + first[1]*first[1] + first[2]*first[2]);
            return val;
        }
        
        std::vector<double> normalize(std::vector<double> any){
            double sum{0};
            double mag{0};
        
            std::vector<double> cop=any;
        
            for(double & com: cop){
                com=com*com;
                sum=sum+com;
                
        
            }
            mag =sqrt(sum);
            
            for(double & co :any){
                co=co/mag;
            }
            return any;  
        }
        
        std::vector<double> matrixVectorMultiply(const double matrix[3][3], const std::vector<double> &vec) {
           
            // Resultant vector (m x 1)
           std::vector<double> result={0};
        
            // Matrix-Vector Multiplication
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    result[i] += matrix[i][j] * vec[j];
                }
            }
        
            return result;
        }
        std::vector <double> direction_space_per_obstacle (std::vector <double> last_obstacle_normal){
            std::vector < double> last_ =last_obstacle_normal;
            double sum=0;
        
            for(double & com: last_){
                com=com*com;
                sum=sum+com;
               
            }
            
        double magnitude=sqrt(sum);
        if (magnitude>0){
        
            //Find Unit Vector
            //Vector by It's Magnitude
            
            for(double & ve:last_obstacle_normal){
                ve=ve /magnitude;
            }
          
            std::vector <double> unit_vector= last_obstacle_normal;
        
            // This will be first column of null_matrix)which is ortthogonal
          // double null_matrix[3][3]  ={0};
        
        
           for (size_t i=0;i<3;i++){
            null_matrix_[i][0]=unit_vector[i];
           }
        
        
        
           std::vector<double> secondi ={}; 
           if (unit_vector[0]<1.1 &&unit_vector[0] <0.9){
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
           double transpose_null_matrix[3][3]={{null_matrix_[0][0],null_matrix_[1][0],null_matrix_[2][0]},{null_matrix_[0][1],null_matrix_[1][1],null_matrix_[2][1]},{null_matrix_[0][2],null_matrix_[1][2],null_matrix_[2][2]}};
        
        /*
            for(size_t i=0;i<3;i++){
                for(size_t j=0;j<3;j++){
                    cout<<"Index "<<i<<j<<"  "<<transpose_null_matrix[i][j]<<endl;
                }
            };
        */
            double v_x{1};
            double v_y{2};
            double v_z{3};
        
            //Dot Product with Global Vector
            //To get Directional Space
            std::vector <double> v_d={v_x,
                v_y, 
                v_z};
             magn_original =   magni_tude(v_d);
             RCLCPP_INFO(this->get_logger(),"Orginal magnitude [%.2f]",magn_original);
            std::vector <double> directional_space= matrixVectorMultiply(transpose_null_matrix,v_d);
            if(magni_tude(directional_space)>0){
            directional_space=(directional_space);}
            
        
        
            //Define a convergence radius
            //double conv_radius{2};
        
            // We taking convergence dynamics from initial velocity vector from 
            //First Controller
        
            std::vector <double> conv_vector={0}; 
            if(magni_tude(v_d)>0){
                conv_vector =normalize(second);}
        
            
            // To get back global vector from tangent space
        
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
            const std::vector<double>& n, 
            const std::vector<double>& c, 
            const std::vector<double>& r, 
            double Re
        ) {
            std::vector<double> k_neg_n_c = direction_space_per_obstacle(c);
            double norm_k_neg_n_c = magni_tude(k_neg_n_c);
        
            if (norm_k_neg_n_c >= Re) {
                return c; // No rotation needed, e = c
            }
        
            std::vector<double> k_neg_n_r = direction_space_per_obstacle(r);
            
            std::vector<double> k_neg_n_e(k_neg_n_c.size());
            double b = 0.0; // Solve for b such that || k(-n, e) || = Re
        
            double norm_k_neg_n_r = magni_tude(k_neg_n_r);
        
            if (norm_k_neg_n_r != norm_k_neg_n_c) {
                b = (Re - norm_k_neg_n_r) / (norm_k_neg_n_c - norm_k_neg_n_r);
            } else {
                b = 0.5; // Arbitrary choice if both norms are equal
            }
        
            // Compute k(-n, e)
            for (size_t i = 0; i < k_neg_n_c.size(); i++) {
                k_neg_n_e[i] = (1 - b) * k_neg_n_r[i] + b * k_neg_n_c[i];
            }
        
            // Normalize to ensure ||k(-n, e)|| = Re
            double norm_k_neg_n_e = magni_tude(k_neg_n_e);
            for (size_t i = 0; i < k_neg_n_e.size(); i++) {
                k_neg_n_e[i] *= (Re / norm_k_neg_n_e);
            }
        
            return k_neg_n_e;
        }


// Function to rotate convergence dynamics towards the pseudo-tangent direction

        std::vector<double> weightedSum(const std::vector<double>& v1, const std::vector<double>& v2, double weight) {
            std::vector<double> result(v1.size());
            for (size_t i = 0; i < v1.size(); i++) {
                result[i] = (1 - weight) * v1[i] + weight * v2[i];
            }
            return result;
        }
        double norm(const std::vector<double>& vec) {
            double sum = 0.0;
            for (double val : vec) sum += val * val;
            return std::sqrt(sum);
        }
        std::vector<double> rotateConvergence(
            const std::vector<double>& k_c_f,  // k(c, f) -> initial dynamics
            const std::vector<double>& k_c_e,  // k(c, e) -> pseudo-tangent
            double gamma,                      // Distance-based scaling factor Γ(ξ)
            double R_e,                         // Defined rotation radius (between π/2 and π)
            const std::vector<double>& k_n_r,   // k(-n, r)
            const std::vector<double>& k_n_c    // k(-n, c)
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

            // Normalize the final direction
            return normalize(rotated_k);
        }

        std::vector <double> final (double nn,std::vector <double> vdf){
            for (size_t i=0;i<3;i++){
                vdf[i]=nn*vdf[i];
            }
            return vdf;
        }
        std::vector<double> vector_difference (std::vector <double> one,std::vector <double> two,std::vector <double> neew){
            
            for(size_t i=0;i<3;i++){
                neew[i]=one[i]-two[i];
            };
            return neew;
        }

     
    
         void submarine_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
            rclcpp::Time now = this->now();
            double dt = (now - last_time_).seconds();
            last_time_ = now;
    
            if (dt < 1e-6) return;  // Prevent zero division
    
            // Extract Current Position
            double current_x = msg->pose.pose.position.x;
            double current_y = msg->pose.pose.position.y;
    
            double submarine_error_x = desired_submarine_x_ -current_x ;
            double submarine_error_y =desired_submarine_y_ -current_y ;
            double v_x=pid_submarine_x_.compute(submarine_error_x,dt);
            double v_y=pid_submarine_y_.compute(submarine_error_y,dt);
            

            if(abs(last_obstacle_dist_) <4){
             RCLCPP_INFO(this->get_logger(), "ROAM Starting ");
            std::vector<double> v_d ={v_x,v_y,0};
           // std::vector <double> second_obstacle_normal={0, 0,0};
           // std::vector <double> third_obstacle_normal={0,0,0};
            std::vector <double> k_1= direction_space_per_obstacle(last_obstacle_normal_);
            double safe_value = std::max(-1.0, std::min(1.0, k_1[0])); 
            double angle_orginal_reference=acos(safe_value);  // Assuming k_1[0] corresponds to v̂[1]
            ;
            RCLCPP_INFO(this->get_logger(),"Angle between Reference and Original Vcector  [%.2f]",angle_orginal_reference);
            //k_1[0]=0;
           // std::vector <double> k_2=direction_space_per_obstacle(second_obstacle_normal);
           // std::vector <double> k_3= direction_space_per_obstacle(third_obstacle_normal);
            // Mean will be evaluated as a function of the weight wi for all vectors
           // std::vector <double> weights={0.2,0.3,0.5};

            //Call a function to multiply weights with direction space vectors

            //std::vector <double> fi=weighted(k_1,weights[0]);
            //std::vector <double> fo=weighted(k_2,weights[1]);
            //std::vector<double> fu=weighted(k_3,weights[2]);


            //std::vector <double> summed={fi[0]+fo[0]+fu[0],fi[1]+fo[1]+fu[1],fi[2]+fo[2]+fu[2]};

          //  for(size_t i=0;i<3;i++){
           //     summed[i]=summed[i]/3;
            //}

            //summed[0]=0;
                double Re=M_PI;
                std::vector<double> v = {0.0, 1.0, 0.0}; 
                const std::vector<double> r=cross_product( last_obstacle_normal_,v);

            std::vector <double> pseudo_tangent=computePseudoTangent(
                last_obstacle_normal_, 
                v_d,  
                r,
                 Re
            );
          

            RCLCPP_INFO(this->get_logger(),"Pseudo Tangent Direction  [%.2f]--[%.2f]--[%.2f]",pseudo_tangent[0],pseudo_tangent[1],pseudo_tangent[2]);
            // Rotation TOwards Tangent Direction

            std::vector<double> k_neg_n_r = direction_space_per_obstacle(r);
            std::vector<double> k_neg_n_c = direction_space_per_obstacle(v_d);
            std::vector<double> k_c_f=direction_space_per_obstacle(v_d);
            std::vector<double> k_c_e=direction_space_per_obstacle(pseudo_tangent);
            double gamma =1;
            std::vector <double> Rotated_directional=rotateConvergence(
                k_c_f,  // k(c, f) -> initial dynamics
                k_c_e,  // k(c, e) -> pseudo-tangent
                gamma,                      // Distance-based scaling factor Γ(ξ)
                Re,                          // Defined rotation radius (between π/2 and π)            
                k_neg_n_r,          // k(-n, r)        
                k_neg_n_c                         // k(-n, c)
          
                     );

             
            
            RCLCPP_INFO(this->get_logger(),"Rotated Towards Tngent Space [%.2f]--[%.2f]--[%.2f]",Rotated_directional[0],Rotated_directional[1],Rotated_directional[2]);
            

            double Rr= std::min(Re-magni_tude(k_neg_n_r),M_PI/2);
            std::vector <double> nnw ={0};
            double delta_k_c=magni_tude (vector_difference(k_neg_n_r , k_neg_n_c,nnw));

            double stretching_factor = std::min(1.0,((delta_k_c) / Rr)* ((delta_k_c) / Rr) +  (1-gamma)*(1-gamma));



            std::vector <double> global=final(stretching_factor,v_d);
          // global= matrixVectorMultiply(null_matrix_,k_1);

         //  RCLCPP_INFO(this->get_logger(),"null_matrix [%.2f]",null_matrix_[0][0]);

         //  RCLCPP_INFO(this->get_logger(),"Outside Magnitude[%.2f] ",magn_original);
           

           v_x=global[0];
           v_y=global[1];
         
           RCLCPP_INFO(this->get_logger(),"Transformed Global Vector[%.2f][%.2f]",v_x,v_y);

    //PUBLISHINING
             double thrust_sub_x = - v_x;
            thrust_sub_x=std::clamp(thrust_sub_x,-15.0,15.0);
            // Publish Modified Thrust
            std_msgs::msg::Float64 thrust_msg;
            thrust_msg.data = thrust_sub_x;
            pub_submarine_thrust_->publish(thrust_msg);
    
           RCLCPP_INFO(this->get_logger(), "Modulated Thrust X: [%.2f]", v_x);
    
    
           double fin_position=v_y/10;
            fin_position = std::clamp(fin_position, -6.28, 6.28);
    
            // Publish Fin Control
            std_msgs::msg::Float64 fin_msg;
            fin_msg.data = -fin_position;
            pub_submarine_fin_->publish(fin_msg);
    
           RCLCPP_INFO(this->get_logger(), " Fin Position: [%.2f]",  fin_position);
          
        }
        else{
            double thrust_sub_x = - v_x;
            thrust_sub_x=std::clamp(thrust_sub_x,-15.0,15.0);

            std_msgs::msg::Float64 thrust_msg;
            thrust_msg.data = thrust_sub_x;
            pub_submarine_thrust_->publish(thrust_msg);
    
           RCLCPP_INFO(this->get_logger(), "Modulated Thrust X: [%.2f]", thrust_sub_x);
    
    
           double fin_position=v_y;
            fin_position = std::clamp(fin_position, -6.28, 6.28);
    
            // Publish Fin Control
            std_msgs::msg::Float64 fin_msg;
            fin_msg.data = -fin_position;
            pub_submarine_fin_->publish(fin_msg);
    
           RCLCPP_INFO(this->get_logger(), " Fin Position: [%.2f]",  fin_position);
          

        }
        
        
    };

};
    
    //  Main Function
    int main(int argc, char *argv[]) {
        rclcpp::init(argc, argv);
        auto node = std::make_shared<MultiSubscriberNode>();
        rclcpp::spin(node);
        rclcpp::shutdown();
        return 0;
    }
    
    