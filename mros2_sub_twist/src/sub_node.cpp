#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber() : Node("sub_twist")
  {
    subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr message) const
  {
    RCLCPP_INFO(this->get_logger(), "Subscribed msg: { linear: { x: %f, y: %f, z: %f }, angular: { x: %f, y: %f, z: %f } }", message->linear.x, message->linear.y, message->linear.z, message->angular.x, message->angular.y, message->angular.z);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}