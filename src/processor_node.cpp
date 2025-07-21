# include "cot_pipeline/processor_node.hpp"

ProcessorNode::ProcessorNode() : Node("processor_node")
{
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "cot_event", 10,
        std::bind(&ProcessorNode::process_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::String>("target_info", 10);

    RCLCPP_INFO(this->get_logger(), "Processor node initialized.");
}

void ProcessorNode::process_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    auto input = msg->data;
    std::string info = input;

    RCLCPP_INFO(this->get_logger(), "%s", info.c_str());
  }

  TargetInfo ProcessorNode::parse_target_info_from_json(const std::string& input)
  {
    TargetInfo info;
    std::regex pair_re("\"(\\w+)\":\\s*(\"[^\"]+\"|[\\d\\.]+)");
    auto begin = std::sregex_iterator(input.begin(), input.end(), pair_re);
    auto end = std::sregex_iterator();

    for (auto i = begin; i != end; ++i) {
      std::smatch match = *i;
      std::string key = match[1];
      std::string val = match[2];

      // 去掉引號
      if (val.front() == '"') val = val.substr(1, val.length() - 2);

      if (key == "id") info.id = val;
      else if (key == "lat") info.lat = std::stod(val);
      else if (key == "lon") info.lon = std::stod(val);
      else if (key == "alt") info.alt = std::stod(val);
      else if (key == "type") info.type = val;
      else if (key == "heading") info.heading = std::stod(val);
    }

    return info;
  }