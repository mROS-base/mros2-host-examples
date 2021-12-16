#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "location_msgs/msg/location.hpp"

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber() : Node("mros2_sub")
  {
    subscriber_ = this->create_subscription<location_msgs::msg::Location>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const location_msgs::msg::Location::SharedPtr message) const
  {
    RCLCPP_INFO(this->get_logger(), "Subscribed msg: { x: %u, y: %u, z: %u }", message->x,message->y, message->z);
  }
  rclcpp::Subscription<location_msgs::msg::Location>::SharedPtr subscriber_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}