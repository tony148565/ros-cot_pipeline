#include "cot_pipeline/cot_encoder_node.hpp"
#include "cot_info.hpp"
#include "cot_builder.hpp"
#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

CotEncoderNode::CotEncoderNode() : Node("cot_encoder_node")
{
    /*subscription_ = this->create_subscription<std_msgs::msg::String>(
      "target_info", 10,
      std::bind(&CotEncoderNode::callback, this, std::placeholders::_1));*/

    publisher_ = this->create_publisher<std_msgs::msg::String>("cot_event", 10);
    RCLCPP_INFO(this->get_logger(), "CoT Encoder Node started.");
    timer_ = this->create_wall_timer(
    1000ms, std::bind(&CotEncoderNode::eventpush, this));
};

void CotEncoderNode::eventpush()
{
    // 🔧 TODO: 實際應解析 msg->data (JSON) → struct → XML
    // 這裡為簡化，直接產生一筆固定格式 CoT XML

    CoTInfo info("drone-001", "a-f-A-M-F-U-C"); // uid, type
    //info.lat = 24.12345;
    //info.lon = 120.54321;

    
    std::string cot_xml = CoTBuilder::toXML(info);

    std_msgs::msg::String out_msg;
    out_msg.data = cot_xml;

    publisher_->publish(out_msg);
    RCLCPP_INFO(this->get_logger(), "Published CoT XML event.");
};