#ifndef COT_PIPELINE__PROCESSOR_NODE_HPP_
#define COT_PIPELINE__PROCESSOR_NODE_HPP_

#include <regex>
#include <string>
#include "cot_pipeline/target_info.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ProcessorNode : public rclcpp::Node
{
public:
  ProcessorNode();

private:
  void process_callback(const std_msgs::msg::String::SharedPtr msg);
  TargetInfo parse_target_info_from_json(const std::string & input);
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

#endif  // COT_PIPELINE__PROCESSOR_NODE_HPP_
