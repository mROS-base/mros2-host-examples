#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Publisher : public rclcpp::Node
{
public:
  Publisher()
      : Node("pub_mros2"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("to_stm", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&Publisher::timer_callback, this));
  }

private:
  std::array<rclcpp::Time, 201> publogs;
  void timer_callback()
  {
    if (count_ < 201){
      auto message = geometry_msgs::msg::Twist();
      auto converted_message = std_msgs::msg::String();
      publogs[count_] = this->get_clock()->now();      
      message.linear.x = count_++/10.0;
      message.linear.y = count_/10.0;
      message.linear.z = count_/10.0;
      message.angular.x = count_/50.0;
      message.angular.y = count_/50.0;
      message.angular.z = count_/50.0;
      converted_message.data = std::to_string(message.linear.x) + std::to_string(message.linear.y) + std::to_string(message.linear.z) + std::to_string(message.angular.x) + std::to_string(message.angular.y) + std::to_string(message.angular.z);
      publisher_->publish(converted_message);
    } else if (count_ == 201) {
      std::ofstream writing_file;
      std::string filename = "twist_string_publog.txt";
      writing_file.open(filename, std::ios::out);
      for (int i=0; i<200; i++){
        const std::string writing_text = std::to_string(publogs[i].nanoseconds());
        writing_file << writing_text << std::endl;
      }
    }
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}