#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "float_location_msgs/msg/float_location.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Publisher : public rclcpp::Node
{
public:
  Publisher()
      : Node("pub_mros2"), count_(0)
  {
    publisher_ = this->create_publisher<location_msgs::msg::FloatLocation>("to_stm", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&Publisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = location_msgs::msg::FloatLocation();
    message.x = count_++/10.0;
    message.y = 2*count_++/10.0;
    message.z = 3*count_++/10.0;
    RCLCPP_INFO(this->get_logger(), "Publishing msg: { x: %f, y: %f, z: %f }", message.x,message.y, message.z);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<location_msgs::msg::FloatLocation>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
