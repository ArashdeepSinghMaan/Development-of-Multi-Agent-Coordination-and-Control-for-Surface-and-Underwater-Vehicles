#include <rclcpp/rclcpp.hpp>
#include "message_type/msg/taskallocation.hpp"
#include "message_type/msg/combined_taskallocation.hpp"

using namespace std::chrono_literals;

class Aggregator : public rclcpp::Node {
public:
  Aggregator()
  : Node("aggregator")
  {
    sub_ = create_subscription<message_type::msg::Taskallocation>(
      "/global/task_allocation_discussion", 10,
      std::bind(&Aggregator::alloc_cb, this, std::placeholders::_1));

    pub_ = create_publisher<message_type::msg::CombinedTaskallocation>(
      "/global/aggregated_task_allocations", 10);

    // Every second, publish the combined buffer and clear it
    timer_ = create_wall_timer(1s, std::bind(&Aggregator::publish_combined, this));
  }

private:
  void alloc_cb(const message_type::msg::Taskallocation::SharedPtr msg) {
    buffer_.push_back(*msg);
  }

  void publish_combined() {
    if (buffer_.empty()) return;
    auto out = message_type::msg::CombinedTaskallocation();
    out.stamp = now();
    out.allocations = buffer_;
    pub_->publish(out);
    buffer_.clear();
  }

  rclcpp::Subscription<message_type::msg::Taskallocation>::SharedPtr sub_;
  rclcpp::Publisher<message_type::msg::CombinedTaskallocation>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<message_type::msg::Taskallocation> buffer_;
};

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Aggregator>());
  rclcpp::shutdown();
  return 0;
}
