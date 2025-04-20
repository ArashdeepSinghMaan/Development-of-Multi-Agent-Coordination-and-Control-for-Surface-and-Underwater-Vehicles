#include"rclcpp/rclcpp.hpp"
#include"std_msgs/msg/float64_multi_array.hpp"


class Sub_Pub :public rclcpp::Node
{
public:
    Sub_Pub():Node("Sub_Pub_1"){

    subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "Task_Details", 10, std::bind(&Sub_Pub::topic_callback, this, std::placeholders::_1));

    }
private:
void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
    }
rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;


};

int main(int argc, char * argv[]){

    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<Sub_Pub>());
    rclcpp::shutdown();

    return 0;
}



// Extract Task Details
// Find TAsk1,Task2,Task3.
//Check feasibility
//Find Distance FRom These
//Publish Task and score 
// Subscribe data of Task and score
//Chose The task for which score is highest
//Publish the chosen Task