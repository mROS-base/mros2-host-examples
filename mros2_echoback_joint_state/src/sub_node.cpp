#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber() : Node("mros2_sub")
  {
    subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::JointState::SharedPtr message) const
  {
    RCLCPP_INFO(this->get_logger(), "Subscribed msg: { frame_id: '%s', sec: %d, nanosec: %u, name: { %s, %s, %s }, position: { x: %f, y: %f, z: %f }, velocity: { x: %f, y: %f, z: %f }, effort: { x: %f, y: %f, z: %f } }", message->header.frame_id.c_str(), message->header.stamp.sec, message->header.stamp.nanosec, message->name[0].c_str(), message->name[1].c_str(), message->name[2].c_str(), message->position[0], message->position[1], message->position[2], message->velocity[0], message->velocity[1], message->velocity[2], message->effort[0], message->effort[1], message->effort[2] );
  }
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriber_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}