#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "mix_msgs/msg/mix.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Publisher : public rclcpp::Node
{
public:
  Publisher()
    : Node("pub_mros2"), count_(0)
  {
    publisher_ = this->create_publisher<mix_msgs::msg::Mix>("to_stm", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&Publisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = mix_msgs::msg::Mix();
    message.name = std::to_string(count_++);
    message.height = 170;
    message.weight = 63.5;
    message.array.push_back(count_);
    message.array.push_back(count_+1);
    message.array.push_back(count_+2);
    RCLCPP_INFO(this->get_logger(), "Publishing msg: { name: '%s', height: %u cm, weight: %f kg, array: {%u,%u,%u} }", message.name.c_str(), message.height, message.weight, message.array[0], message.array[1], message.array[2]);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<mix_msgs::msg::Mix>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
