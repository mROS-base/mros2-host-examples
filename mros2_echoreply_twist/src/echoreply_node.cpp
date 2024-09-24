#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Echoreplyer : public rclcpp::Node
{
public:
  Echoreplyer() : Node("mros2_echoreply_node")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("to_stm", 10);
    subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Echoreplyer::topic_callback, this, _1));
  }
  
private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "{ linear: {x: %f, y: %f, z: %f }, angular: {x: %f, y: %f, z: %f } }",
      msg->linear.x, msg->linear.y, msg->linear.z, msg->angular.x, msg->angular.y, msg->angular.z);

    geometry_msgs::msg::Vector3 linear;
    geometry_msgs::msg::Vector3 angular;
    geometry_msgs::msg::Twist twist;

    linear.x = msg->linear.x;
    linear.y = msg->linear.y;
    linear.z = msg->linear.z;
    angular.x = msg->angular.x;
    angular.y = msg->angular.y;
    angular.z = msg->angular.z;
    twist.linear = linear;
    twist.angular = angular;

    RCLCPP_INFO(this->get_logger(), "{ linear: {x: %f, y: %f, z: %f }, angular: {x: %f, y: %f, z: %f } }",
      linear.x, linear.y, linear.z, angular.x, angular.y, angular.z);
    publisher_->publish(*msg);
  }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Echoreplyer>());
  rclcpp::shutdown();
  return 0;
}
