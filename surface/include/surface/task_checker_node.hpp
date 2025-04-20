#pragma once

#include <string>
#include <unordered_map>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "message_type/msg/allocatedtask.hpp"
#include "message_type/msg/list.hpp"

/// Header‑only helper that subscribes to two topics and extracts
/// the desired goal position for a given agent name.
class TaskCheckerNode {
public:
  /// @param agent_name  the agent_name to match against msg->agent_name
  /// @param node        a rclcpp::Node::SharedPtr that this helper will use
  TaskCheckerNode(
    const std::string & agent_name,
    rclcpp::Node::SharedPtr node)
  : node_(std::move(node))
  , agent_name_(agent_name)
  , assigned_task_id_(-1)
  , task_ready_(false)
  {
    allocated_sub_ = node_->create_subscription<message_type::msg::Allocatedtask>(
      "allocated_tasks", 10,
      std::bind(&TaskCheckerNode::allocatedCallback, this, std::placeholders::_1)
    );

    task_details_sub_ = node_->create_subscription<message_type::msg::List>(
      "task_details", 10,
      std::bind(&TaskCheckerNode::taskDetailsCallback, this, std::placeholders::_1)
    );
  }

  bool isTaskReady() const { return task_ready_; }
  geometry_msgs::msg::Point getDesiredPosition() const { return desired_position_; }

private:
  void allocatedCallback(const message_type::msg::Allocatedtask::SharedPtr msg) {
    if (msg->agent_name == agent_name_) {
      assigned_task_id_ = msg->task_id;
      auto it = task_positions_.find(assigned_task_id_);
      if (it != task_positions_.end()) {
        desired_position_ = it->second;
        task_ready_ = true;
        RCLCPP_INFO(node_->get_logger(),
          "Agent '%s' → task %d @ (%.2f,%.2f,%.2f)",
          agent_name_.c_str(),
          assigned_task_id_,
          desired_position_.x,
          desired_position_.y,
          desired_position_.z);
      } else {
        RCLCPP_WARN(node_->get_logger(),
          "Task %d allocated to '%s' but no position yet",
          assigned_task_id_, agent_name_.c_str());
      }
    }
  }

  void taskDetailsCallback(const message_type::msg::List::SharedPtr msg) {
    for (const auto & t : msg->values) {
      geometry_msgs::msg::Point p;
      p.x = t.x; p.y = t.y; p.z = t.z;
      task_positions_[t.task_id] = p;
    }
  }

  // configuration
  std::string agent_name_;

  // state
  int assigned_task_id_;
  bool task_ready_;
  geometry_msgs::msg::Point desired_position_;
  std::unordered_map<int, geometry_msgs::msg::Point> task_positions_;

  // ROS
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<message_type::msg::Allocatedtask>::SharedPtr allocated_sub_;
  rclcpp::Subscription<message_type::msg::List>::SharedPtr          task_details_sub_;
};
