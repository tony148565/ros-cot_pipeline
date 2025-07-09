#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SimDataPublisher : public rclcpp::Node
{
public:
  SimDataPublisher();

private:
  void publish_fake_data();

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};
