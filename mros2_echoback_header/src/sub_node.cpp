#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber() : Node("mros2_sub")
  {
    subscriber_ = this->create_subscription<std_msgs::msg::Header>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Header::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Subscribed msg: { frame_id: '%s',sec: %d, nanosec: %u }", msg->frame_id.c_str(), msg->stamp.sec, msg->stamp.nanosec );
  }
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr subscriber_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}