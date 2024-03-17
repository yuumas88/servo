#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <can_plugins2/msg/frame.hpp>
#include "can_utils.hpp"
#include <cmath>

using namespace can_utils;

can_plugins2::msg::Frame generate_servo_mode(const uint16_t id, const uint8_t mode)
{
  const int byte_size = 1;  // Mode is 1 byte.
  
  can_plugins2::msg::Frame frame;
  frame.id = id;
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;

  frame.dlc = byte_size;

  frame.data[0] = mode;

  return frame;
}

can_plugins2::msg::Frame generate_servo_target(const uint16_t id, const float data)
{
  const int float_size = 4;  // float is 4 bytes.
  
  can_plugins2::msg::Frame frame;
  frame.id = id;
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;

  frame.dlc = float_size;

  can_pack<uint16_t>(frame.data, data);

  return frame;
}

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("servo_node")
  {
    publisher_ = this->create_publisher<can_plugins2::msg::Frame>("can_tx", 10);

    joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&MinimalPublisher::joy_callback, this, std::placeholders::_1)
    );
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    if (joy_msg->buttons[0] == 1){      
      publisher_->publish(generate_servo_mode(0x300, 1));
    }

    if (joy_msg->buttons[1] == 1){      
      publisher_->publish(generate_servo_mode(0x300, 0));
    }

    if (joy_msg->buttons[2] == 1){      
      publisher_->publish(generate_servo_target(0x301, 12000 + 58000*40/270));
    }

    if (joy_msg->buttons[3] == 1){      
      publisher_->publish(generate_servo_target(0x301, 12000 + 58000));
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
  rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MinimalPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
