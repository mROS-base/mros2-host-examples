#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber() : Node("mros2_sub")
  {
    subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const nav_msgs::msg::Odometry::SharedPtr message) const
  {
    RCLCPP_INFO(this->get_logger(), "Subscribed msg: { sec:%d, nanosec:%u, frame_id:%s, child_frame_id:%s, pos_x:%f, ori_x:%f, pos_cov:%f, lin:%f, ang:%f, twi_cov:%f }", message->header.stamp.sec, message->header.stamp.nanosec, message->header.frame_id.c_str(), message->child_frame_id.c_str(), message->pose.pose.position.x, message->pose.pose.orientation.x,  message->pose.covariance[18], message->twist.twist.linear.x, message->twist.twist.angular.x, message->twist.covariance[18] );
  }
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}