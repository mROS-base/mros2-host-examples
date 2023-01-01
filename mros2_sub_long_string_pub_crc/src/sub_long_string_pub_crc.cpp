#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

using std::placeholders::_1;

// imported from
// https://github.com/aeldidi/crc32/blob/master/src/crc32.c
uint32_t
crc32_for_byte(uint32_t byte)
{
	const uint32_t polynomial = 0xEDB88320L;
	uint32_t       result     = byte;
	size_t         i          = 0;

	for (; i < 8; i++) {
		result = (result >> 1) ^ (result & 1) * polynomial;
	}
	return result;
}

uint32_t
crc32(const void *input, size_t size)
{
  const uint8_t *current = static_cast<const uint8_t *>(input);
	uint32_t       result  = 0xFFFFFFFF;
	size_t         i       = 0;

	for (; i < size; i++) {
		result ^= current[i];
		result = crc32_for_byte(result);
	}

	return ~result;
}

class Crc32generator : public rclcpp::Node
{
public:
  Crc32generator() : Node("recv_long_string_send_crc_node")
  {
    publisher_ = this->create_publisher<std_msgs::msg::UInt32>("to_stm", 10);
    subscriber_ = this->create_subscription<std_msgs::msg::String>("to_linux", rclcpp::QoS(10).best_effort(), std::bind(&Crc32generator::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    auto pub_msg = std_msgs::msg::UInt32();
    uint32_t tmp = crc32((char *)&msg->data[0], msg->data.size());
    pub_msg.data = tmp;
    RCLCPP_INFO(this->get_logger(), "\r\nSubscribed msg: '%s'", msg->data.c_str());
    RCLCPP_INFO(this->get_logger(), "\r\n");
    RCLCPP_INFO(this->get_logger(), "========================");
    RCLCPP_INFO(this->get_logger(), "CRC: 0x%0x", tmp);
    RCLCPP_INFO(this->get_logger(), "========================");
    publisher_->publish(pub_msg);
  }
  rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Crc32generator>());
  rclcpp::shutdown();
  return 0;
}
