#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/mesh.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class Publisher : public rclcpp::Node
{
public:
  Publisher()
      : Node("pub_mros2"), count_(0)
  {
    publisher_ = this->create_publisher<shape_msgs::msg::Mesh>("to_stm", 10);
    timer_ = this->create_wall_timer(1000ms, std::bind(&Publisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = shape_msgs::msg::Mesh();
    message.triangles.reserve(2);
    message.vertices.reserve(2);
    for (int i=0;i<2;i++){
      for (int j=0;j<3;j++){
        message.triangles[i].vertex_indices[j]=count_++;
      }
    }
    for (int i=0;i<2;i++){
      for (int j=0;j<3;j++){
        message.vertices[i].x=count_++;
        message.vertices[i].y=count_++;
        message.vertices[i].z=count_++;
      }
    }
    RCLCPP_INFO(this->get_logger(), "Publishing msg: { triangles: {{ %lu, %lu, %lu }, { %lu, %lu, %lu }}, vertices: {{ %f, %f, %f }, { %f, %f, %f }}}", message.triangles[0].vertex_indices[0], message.triangles[0].vertex_indices[1], message.triangles[0].vertex_indices[2], message.triangles[1].vertex_indices[0], message.triangles[1].vertex_indices[1], message.triangles[1].vertex_indices[2], message.vertices[0].x, message.vertices[0].y, message.vertices[0].z, message.vertices[1].x, message.vertices[1].y, message.vertices[1].z);
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<shape_msgs::msg::Mesh>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}