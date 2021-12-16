#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "health_msgs/msg/health.hpp"

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber() : Node("mros2_sub")
  {
    subscriber_ = this->create_subscription<health_msgs::msg::Health>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const health_msgs::msg::Health::SharedPtr message) const
  {
    RCLCPP_INFO(this->get_logger(), "Subscribed msg: { name: '%s', height: %d cm, weight: %f kg }", message->name.c_str(), message->height, message->weight);
  }
  rclcpp::Subscription<health_msgs::msg::Health>::SharedPtr subscriber_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}