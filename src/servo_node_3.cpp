#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <robomas_plugins/msg/frame.hpp>
#include "can_utils.hpp"
#include <cmath>

using namespace can_utils;

robomas_plugins::msg::Frame generate_servo_mode(const uint16_t id, const uint8_t mode)
{
  const int byte_size = 1;  // Mode is 1 byte.
  
  robomas_plugins::msg::Frame frame;
  frame.id = id;
  frame.is_rtr = false;
  frame.is_extended = false;
  frame.is_error = false;

  frame.dlc = byte_size;

  frame.data[0] = mode;

  return frame;
}

class ServoPublisher : public rclcpp::Node
{
  public:
    ServoPublisher()
    : Node("servo_node")
    {
      publisher_ = this->create_publisher<robomas_plugins::msg::Frame>("robomas_can_tx2", 10);

      joy_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&ServoPublisher::joy_callback, this, std::placeholders::_1)
      );
    }

    void set_angle(uint8_t channel,uint16_t angle){
      
      robomas_plugins::msg::Frame frame;
      frame.id = 0x301;
      frame.is_rtr = false;
      frame.is_extended = false;
      frame.is_error = false;

      frame.dlc = 8;
      frame.data.fill(0);

      std::memcpy(&frame.data[2*(channel-1)], &angle, 2);

      publisher_->publish(frame);
    }

    // std::unique_ptr<robomas_plugins::msg::Frame> set_angle2(uint8_t channel,uint16_t angle){//悪い実装
    //   uint8_t data[8] = {};
    //   std::memcpy(&data[2*(channel-1)], &angle, 2);
    //   return can_utils::generate_frame(0x301,data);//ポインターの先がいつまで生きているのか
    // }

  private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
    {
      if (joy_msg->buttons[7] == 1){      
        publisher_->publish(generate_servo_mode(0x300, 1));
      }

      if (joy_msg->buttons[6] == 1){      
        publisher_->publish(generate_servo_mode(0x300, 0));
      }

      if (joy_msg->buttons[1] == 1){      
        set_angle(2, 11000 + 48000*40/270);
        set_angle(3, 12000 + 48000*40/270);
      }

      if (joy_msg->buttons[3] == 1){      
        set_angle(2, 12000 + 48000*130/270);
        set_angle(3, 11000 + 48000*130/270);
      }
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;
    rclcpp::Publisher<robomas_plugins::msg::Frame>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ServoPublisher>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
