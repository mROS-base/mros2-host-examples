#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "float_location_msgs/msg/float_location.hpp"

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber() : Node("mros2_sub")
  {
    subscriber_ = this->create_subscription<float_location_msgs::msg::FloatLocation>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const float_location_msgs::msg::FloatLocation::SharedPtr message) const
  {
    RCLCPP_INFO(this->get_logger(), "Subscribed msg: { x: %f, y: %f, z: %f }", message->x,message->y, message->z);
  }
  rclcpp::Subscription<float_location_msgs::msg::FloatLocation>::SharedPtr subscriber_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}