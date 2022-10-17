#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Echoreplyer : public rclcpp::Node
{
public:
  Echoreplyer() : Node("mros2_echoreply_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("to_stm", 10);
    subscriber_ = this->create_subscription<std_msgs::msg::String>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Echoreplyer::topic_callback, this, _1));
  }
  
private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "\r\nSubscribed msg: '%s'", msg->data.c_str());
    RCLCPP_INFO(this->get_logger(), "\r\nPublishing msg: '%s'", msg->data.c_str());
    publisher_->publish(*msg);
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Echoreplyer>());
  rclcpp::shutdown();
  return 0;
}
