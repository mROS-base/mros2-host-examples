#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mix_msgs/msg/mix.hpp"

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber() : Node("mros2_sub")
  {
    subscriber_ = this->create_subscription<mix_msgs::msg::Mix>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const mix_msgs::msg::Mix::SharedPtr message) const
  {
    RCLCPP_INFO(this->get_logger(), "Subscribed msg: { name: '%s', height: %d cm, weight: %f kg, array: {%u,%u,%u} }", message->name.c_str(), message->height, message->weight, message->array[0], message->array[1], message->array[2]);
  }
  rclcpp::Subscription<mix_msgs::msg::Mix>::SharedPtr subscriber_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}