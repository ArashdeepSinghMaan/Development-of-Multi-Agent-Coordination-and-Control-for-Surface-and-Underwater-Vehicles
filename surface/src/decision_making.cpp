#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <cmath>
#include <vector>
#include <unordered_map>
#include <limits>
#include <string>
#include <chrono>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "message_type/msg/list.hpp"
#include "message_type/msg/triple.hpp"
#include "message_type/msg/allocatedtask.hpp"
#include "message_type/msg/taskallocation.hpp"
#include "message_type/msg/combined_taskallocation.hpp"
#include "message_type/msg/task_score.hpp"

using namespace std::chrono_literals;

double get_yaw_from_quaternion(const geometry_msgs::msg::Quaternion &quat) {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

class Decision_Making : public rclcpp::Node {
public:
    Decision_Making()
    : Node("Decision_Making")
    {
        this->declare_parameter<std::string>("robot_name", "my_lrauv_1");
        this->get_parameter("robot_name", robot_name_);

        // Subscriptions
        sub_task_details_ = this->create_subscription<message_type::msg::List>(
            "/Task_Details", 10,
            std::bind(&Decision_Making::task_details_callback, this, std::placeholders::_1));

        std::string topic_odom = "/" + robot_name_ + "/submarine/odometry";
        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            topic_odom, 10,
            std::bind(&Decision_Making::odom_callback, this, std::placeholders::_1));

        sub_combined_ = this->create_subscription<message_type::msg::CombinedTaskallocation>(
            "/global/aggregated_task_allocations", 10,
            std::bind(&Decision_Making::combined_callback, this, std::placeholders::_1));

        sub_final_allocated_ = this->create_subscription<message_type::msg::Allocatedtask>(
            "allocated_tasks", 10,
            std::bind(&Decision_Making::allocated_task_callback, this, std::placeholders::_1));

        // Publishers
        pub_task_allocation_ = this->create_publisher<message_type::msg::Taskallocation>(
            "task_allocation_discussion", 10);
        pub_allocated_task_ = this->create_publisher<message_type::msg::Allocatedtask>(
            "allocated_tasks", 10);

        // Timer to periodically check if I am the best agent
        timer_ = this->create_wall_timer(500ms, std::bind(&Decision_Making::check_allocation_decision, this));
    }

private:
    std::string robot_name_;
    nav_msgs::msg::Odometry::SharedPtr latest_odom_;
    message_type::msg::List::SharedPtr latest_tasks_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<message_type::msg::List>::SharedPtr sub_task_details_;
    rclcpp::Subscription<message_type::msg::Taskallocation>::SharedPtr sub_allocation_;
    rclcpp::Subscription<message_type::msg::Allocatedtask>::SharedPtr sub_final_allocated_;
    rclcpp::Subscription<message_type::msg::CombinedTaskallocation>::SharedPtr sub_combined_;

    rclcpp::Publisher<message_type::msg::Taskallocation>::SharedPtr pub_task_allocation_;
    rclcpp::Publisher<message_type::msg::Allocatedtask>::SharedPtr pub_allocated_task_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unordered_map<int, std::vector<message_type::msg::Taskallocation>> all_allocations_;
    std::unordered_map<int, std::string> allocated_tasks_;

    void task_details_callback(const message_type::msg::List::SharedPtr msg) {
        latest_tasks_ = msg;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        latest_odom_ = msg;
        compute_and_publish_task_score();  // Triggered every odom update
    }

    void compute_and_publish_task_score() {
        if (!latest_odom_ || !latest_tasks_) return;
    
        double agent_x = latest_odom_->pose.pose.position.x;
        double agent_y = latest_odom_->pose.pose.position.y;
        double agent_yaw = get_yaw_from_quaternion(latest_odom_->pose.pose.orientation);
    
        for (const auto &task : latest_tasks_->values) {
            if (allocated_tasks_.find(task.task_id) != allocated_tasks_.end()) continue;
    
            double task_x = task.x;
            double task_y = task.y;
           
            double distance = std::sqrt(std::pow(task_x - agent_x, 2) + std::pow(task_y - agent_y, 2));
            double desired_yaw = std::atan2(task_y - agent_y, task_x - agent_x);
            double yaw_difference = std::fabs(desired_yaw - agent_yaw);
            if (yaw_difference > M_PI) yaw_difference = 2 * M_PI - yaw_difference;
    
            double turn_distance = (yaw_difference * 180.0 / M_PI) * 0.5;
            double final_distance = distance + turn_distance;
    
            // Prepare TaskScore for the task
            message_type::msg::TaskScore score_msg;
            score_msg.task_id = task.task_id;
            score_msg.final_distance = final_distance;
    
            // Prepare Taskallocation for this agent
            message_type::msg::Taskallocation allocation_msg;
            allocation_msg.agent_name = robot_name_;
            allocation_msg.scores.push_back(score_msg);  // Add the score to the list of scores
            allocation_msg.round_number = 1;  // Set the round number (can adjust based on your logic)
    
            // Publish the task allocation
            pub_task_allocation_->publish(allocation_msg);
            RCLCPP_INFO(this->get_logger(), "[%s] Publishing allocation for task %d with cost %.2f",
                        robot_name_.c_str(), task.task_id, final_distance);
        }
    }
    
    void combined_callback(const message_type::msg::CombinedTaskallocation::SharedPtr msg) {
        // Reset for this round
        all_allocations_.clear();
        allocated_tasks_.clear();
      
        // Unpack everybody’s scores
        for (auto &alloc : msg->allocations) {
            for (const auto &score : alloc.scores) {
                all_allocations_[score.task_id].push_back(alloc);  // Associate task_id with allocation
            }
        }
      
        // Now immediately decide & claim
        check_allocation_decision();
    }
    

    void allocated_task_callback(const message_type::msg::Allocatedtask::SharedPtr msg) {
        allocated_tasks_[msg->task_id] = msg->agent_name;
    }

    void check_allocation_decision() {
        for (const auto &[task_id, alloc_list] : all_allocations_) {
            if (allocated_tasks_.find(task_id) != allocated_tasks_.end()) continue;
    
            double min_cost = std::numeric_limits<double>::max();
            std::string best_agent;
    
            for (const auto &alloc : alloc_list) {
                for (const auto &score : alloc.scores) {
                    if (score.final_distance < min_cost) {
                        min_cost = score.final_distance;
                        best_agent = alloc.agent_name;
                    }
                }
            }
    
            if (best_agent == robot_name_) {
                RCLCPP_INFO(this->get_logger(), "Agent %s is claiming task %d", robot_name_.c_str(), task_id);
                message_type::msg::Allocatedtask allocated_msg;
                allocated_msg.task_id = task_id;
                allocated_msg.agent_name = robot_name_;
                pub_allocated_task_->publish(allocated_msg);
                allocated_tasks_[task_id] = robot_name_;
                RCLCPP_INFO(this->get_logger(), "[%s] FINAL CLAIM for task %d", robot_name_.c_str(), task_id);
            }
        }
    
        all_allocations_.clear();  // Reset for next round
    }
    

};

// Main Function
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Decision_Making>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
