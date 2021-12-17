#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/w_strings.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Publisher : public rclcpp::Node
{
public:
  Publisher()
    : Node("pub_mros2"), count_(0)
  {
    publisher_ = this->create_publisher<test_msgs::msg::WStrings>("to_stm", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&Publisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = test_msgs::msg::WStrings();
    message.wstring_value = u"Hello, world! ";
    RCLCPP_INFO(this->get_logger(), "Publishing msg: '%ls'", message.wstring_value);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<test_msgs::msg::WStrings>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
