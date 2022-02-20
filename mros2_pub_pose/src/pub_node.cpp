#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Publisher : public rclcpp::Node
{
public:
  Publisher()
    : Node("pub_pose"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Pose>("cmd_vel", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&Publisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto point = geometry_msgs::msg::Point();
    auto quaternion = geometry_msgs::msg::Quaternion();
    auto pose = geometry_msgs::msg::Pose();
    point.x = count_/1.0;
    point.y = count_/1.0;
    point.z = count_/1.0;
    quaternion.x = count_/1.0;
    quaternion.y = count_/1.0;
    quaternion.z = count_/1.0;
    quaternion.w = count_/1.0;
    pose.position = point;
    pose.orientation = quaternion;
    RCLCPP_INFO(this->get_logger(), "Publishing msg: { position: {x: %f, y: %f, z: %f }, orientation: {x: %f, y: %f, z: %f, w: %f } }" , pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w );
    publisher_->publish(pose);
    count_++;
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr publisher_;
  uint16_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
