#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "test_msgs/msg/w_strings.hpp"

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber() : Node("mros2_sub")
  {
    subscriber_ = this->create_subscription<test_msgs::msg::WStrings>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const test_msgs::msg::WStrings::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Subscribed msg: '%ls'", msg->data.c_str());
  }
  rclcpp::Subscription<test_msgs::msg::WStrings>::SharedPtr subscriber_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}