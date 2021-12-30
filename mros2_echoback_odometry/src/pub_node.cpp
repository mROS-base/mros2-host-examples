#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Publisher : public rclcpp::Node
{
public:
  Publisher()
      : Node("pub_mros2"), count_(0)
  {
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("to_stm", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&Publisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = nav_msgs::msg::Odometry();
    message.header.stamp = this->get_clock()->now();
    message.header.frame_id = "odometry";
    message.child_frame_id = "child_odometry";
    for (int i=0;i<36;i++){
      message.pose.covariance[i]=(i/100.0);
    }
    message.pose.pose.position.x=count_++/10.0;
    message.pose.pose.position.y=count_++/10.0;
    message.pose.pose.position.z=count_++/10.0;
    message.pose.pose.orientation.x=count_++/20.0;
    message.pose.pose.orientation.y=count_++/20.0;
    message.pose.pose.orientation.z=count_++/20.0;
    message.pose.pose.orientation.w=count_++/20.0;

    for (int i=0;i<36;i++){
      message.twist.covariance[i]=(i/100.0);
    }
    message.twist.twist.linear.x = count_++/50.0;
    message.twist.twist.linear.y = count_++/50.0;
    message.twist.twist.linear.z = count_++/50.0;
    message.twist.twist.angular.x = count_++/100.0;
    message.twist.twist.angular.y = count_++/100.0;
    message.twist.twist.angular.z = count_++/100.0;
    
    RCLCPP_INFO(this->get_logger(), "Publishing msg: { sec:%d, nanosec:%u, frame_id:%s, child_frame_id:%s, pos_x:%f, ori_x:%f, pos_cov:%f, lin:%f, ang:%f, twi_cov:%f }", message.header.stamp.sec, message.header.stamp.nanosec, message.header.frame_id.c_str(), message.child_frame_id.c_str(), message.pose.pose.position.x, message.pose.pose.orientation.x,  message.pose.covariance[18], message.twist.twist.linear.x, message.twist.twist.angular.x, message.twist.covariance[18] );
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}