#include "cot_pipeline/sim_data_publisher.hpp"
#include <chrono>
#include <sstream>

using namespace std::chrono_literals;

SimDataPublisher::SimDataPublisher()
: Node("sim_data_publisher")
{
  publisher_ = this->create_publisher<std_msgs::msg::String>("raw_sim_data", 10);
  timer_ = this->create_wall_timer(
    1000ms, std::bind(&SimDataPublisher::publish_fake_data, this));
}

void SimDataPublisher::publish_fake_data()
{
  std_msgs::msg::String msg;
  msg.data = R"({
    "id": "target_001",
    "lat": 24.12345,
    "lon": 120.54321,
    "alt": 100,
    "type": "vehicle",
    "heading": 270.0
  })";
  publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(), "Published: %s", msg.data.c_str());
}
