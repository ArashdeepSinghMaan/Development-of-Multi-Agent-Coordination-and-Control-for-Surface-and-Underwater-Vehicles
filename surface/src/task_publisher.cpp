#include "rclcpp/rclcpp.hpp"
#include "message_type/msg/list.hpp"
#include "message_type/msg/triple.hpp"
#include "message_type/msg/allocatedtask.hpp"
#include "message_type/msg/taskallocation.hpp"
class TaskPublisher : public rclcpp::Node
{
public:
  TaskPublisher() : Node("simple_publisher")
  {
    publisher_ = this->create_publisher<message_type::msg::List>("Task_Details", 10);
    pub_task_allocation_ = this->create_publisher<message_type::msg::Taskallocation>(
      "/global/task_allocation_discussion", 10);
  pub_allocated_task_ = this->create_publisher<message_type::msg::Allocatedtask>(
      "/global/allocated_tasks", 10);
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TaskPublisher::publish_message, this));
  }

private:
  void publish_message()
  {
    message_type::msg::List msg;
    float task_details[4][3] = {
      {20, 10, -6},
      {2, 105, -9},
      {100, 20, -20},
      {25, 30, -8}
    };

    for (int i = 0; i < 4; ++i) {
      message_type::msg::Triple t;
      t.task_id = i + 1; 
      t.x = task_details[i][0];
      t.y = task_details[i][1];
      t.z = task_details[i][2];
      msg.values.push_back(t);
    }
    
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published task details message with %zu elements", msg.values.size());
  }

  rclcpp::Publisher<message_type::msg::List>::SharedPtr publisher_;
  rclcpp::Publisher<message_type::msg::Taskallocation>::SharedPtr pub_task_allocation_ ;
  rclcpp::Publisher<message_type::msg::Allocatedtask>::SharedPtr pub_allocated_task_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TaskPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
