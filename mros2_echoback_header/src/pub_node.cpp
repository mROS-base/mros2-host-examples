#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Publisher : public rclcpp::Node
{
public:
  Publisher()
    : Node("pub_mros2"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Header>("to_stm", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&Publisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Header();
    message.frame_id = "Hello, world! " + std::to_string(count_++);
    message.stamp = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "Publishing msg: { frame_id: '%s', sec: %d, nanosec: %u } ", message.frame_id.c_str(), message.stamp.sec, message.stamp.nanosec );
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
