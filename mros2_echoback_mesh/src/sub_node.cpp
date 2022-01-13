#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "shape_msgs/msg/mesh.hpp"

using std::placeholders::_1;

class Subscriber : public rclcpp::Node
{
public:
  Subscriber() : Node("mros2_sub")
  {
    subscriber_ = this->create_subscription<shape_msgs::msg::Mesh>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Subscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const shape_msgs::msg::Mesh::SharedPtr message) const
  {
    RCLCPP_INFO(this->get_logger(), "Subscribed msg: { triangles: {{ %lu, %lu, %lu }, { %lu, %lu, %lu }}, vertices: {{ %f, %f, %f }, { %f, %f, %f }}}", message->triangles[0].vertex_indices[0], message->triangles[0].vertex_indices[1], message->triangles[0].vertex_indices[2], message->triangles[1].vertex_indices[0], message->triangles[1].vertex_indices[1], message->triangles[1].vertex_indices[2], message->vertices[0].x, message->vertices[0].y, message->vertices[0].z, message->vertices[1].x, message->vertices[1].y, message->vertices[1].z);
  }
  rclcpp::Subscription<shape_msgs::msg::Mesh>::SharedPtr subscriber_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}