#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"


using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber() : Node("mros2_sub")
  {
    subscriber_ = this->create_subscription<std_msgs::msg::String>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr message) const
  {
    auto reconverted_message = geometry_msgs::msg::Twist();
    reconverted_message.linear.x = std::stod(message->data.substr(0,8));
    reconverted_message.linear.y = std::stod(message->data.substr(8,8));
    reconverted_message.linear.z = std::stod(message->data.substr(16,8));
    reconverted_message.angular.x = std::stod(message->data.substr(24,8));
    reconverted_message.angular.y = std::stod(message->data.substr(32,8));
    reconverted_message.angular.z = std::stod(message->data.substr(40,8));
    
    RCLCPP_INFO(this->get_logger(), "Subscribed msg: { linear: { x: %f, y: %f, z: %f }, angular: { x: %f, y: %f, z: %f } }", reconverted_message.linear.x, reconverted_message.linear.y, reconverted_message.linear.z, reconverted_message.angular.x, reconverted_message.angular.y, reconverted_message.angular.z);
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}