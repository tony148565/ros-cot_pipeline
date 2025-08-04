#ifndef COT_PIPELINE__COT_ENCODER_NODE_HPP_
#define COT_PIPELINE__COT_ENCODER_NODE_HPP_

#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
class CotEncoderNode : public rclcpp::Node
{
public:
  CotEncoderNode();

private:
  void eventpush();
// rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // COT_PIPELINE__COT_ENCODER_NODE_HPP_
