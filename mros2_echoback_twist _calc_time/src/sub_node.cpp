#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber() : Node("mros2_sub")
  {
    subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr message)
  {
    auto subscribed_message = geometry_msgs::msg::Twist();
    subscribed_message = *message;

    rclcpp::Time now = this->get_clock()->now();
    std::ofstream writing_file;
    std::string filename = "twist_sublog.txt";
    writing_file.open(filename, std::ios::app);
    const std::string writing_text = std::to_string(now.nanoseconds());
    writing_file << writing_text << std::endl;
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}