#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"


using std::placeholders::_1;


class Subscriber : public rclcpp::Node
{
public:
  Subscriber() : Node("mros2_sub")
  {
    subscriber_ = this->create_subscription<std_msgs::msg::String>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  std::array<rclcpp::Time, 200> sublogs;
  void topic_callback(const std_msgs::msg::String::SharedPtr message)
  {
    auto reconverted_message = geometry_msgs::msg::Twist();
    reconverted_message.linear.x = std::stod(message->data.substr(0,8));
    reconverted_message.linear.y = std::stod(message->data.substr(8,8));
    reconverted_message.linear.z = std::stod(message->data.substr(16,8));
    reconverted_message.angular.x = std::stod(message->data.substr(24,8));
    reconverted_message.angular.y = std::stod(message->data.substr(32,8));
    reconverted_message.angular.z = std::stod(message->data.substr(40,8));

    rclcpp::Time now = this->get_clock()->now();
    std::ofstream writing_file;
    std::string filename = "twist_string_sublog.txt";
    writing_file.open(filename, std::ios::app);
    const std::string writing_text = std::to_string(now.nanoseconds());
    writing_file << writing_text << std::endl;
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}