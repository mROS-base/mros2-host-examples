#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Publisher : public rclcpp::Node
{
public:
  Publisher()
      : Node("pub_mros2"), count_(0)
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("to_stm", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&Publisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = sensor_msgs::msg::JointState();
    message.header.frame_id = "0";
    message.header.stamp = this->get_clock()->now();
    message.name.resize(2);
    message.name[0] = "joint1";
    message.name[1] = "joint2";
    message.position.resize(2);
    message.position[0] = -1.0*(count_/40.0);
    message.position[1] = 2.0*(count_/40.0);
    message.velocity.resize(2);
    message.velocity[0] = -1.0*(count_/40.0);
    message.velocity[1] = 2.0*(count_/40.0);
    message.effort.resize(1);
    message.effort[0] = -1.0*(count_/40.0);
    //message.effort[1] = 2.0*(count_/40.0);
    count_++;
    RCLCPP_INFO(this->get_logger(), "Publishing msg: { frame_id: '%s', sec: %d, nanosec: %u, name: { %s, %s }, position: { x: %f, y: %f }, velocity: { x: %f, y: %f }, effort: { x: %f, y: %f } }", message.header.frame_id.c_str(), message.header.stamp.sec, message.header.stamp.nanosec, message.name[0].c_str(), message.name[1].c_str(), message.position[0], message.position[1], message.velocity[0], message.velocity[1], message.effort[0], message.effort[1] );
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}