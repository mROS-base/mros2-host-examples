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
    message.vec3.vec3.x = 1;
    message.vec3.vec3.y = 1;
    message.vec3.vec3.z = 1;
    message.height = 170;
    message.weight = 63.5;
    for (int i=0;i<8;i++){
      message.array[i]=i/100.0;
    }
    RCLCPP_INFO(this->get_logger(), "Publishing msg: { name: '%s', x: %u, y: %u, z: %u,height: %u cm, weight: %f kg, array: {%f,%f,%f} }", message.name.c_str(), message.vec3.vec3.x, message.vec3.vec3.y, message.vec3.vec3.z, message.height, message.weight, message.array[0], message.array[1], message.array[2]);
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
