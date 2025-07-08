#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <sstream>
#include <string>

using namespace std::chrono_literals;

class SimDataPublisher : public rclcpp::Node
{
public:
  SimDataPublisher()
  : Node("sim_data_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("raw_sim_data", 10);
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&SimDataPublisher::publish_fake_data, this));
  }

private:
  void publish_fake_data()
  {
    std_msgs::msg::String msg;

    std::stringstream ss;
    ss << "{"
       << "\"id\": \"target_001\", "
       << "\"lat\": 24.12345, "
       << "\"lon\": 120.54321, "
       << "\"alt\": 100, "
       << "\"type\": \"vehicle\", "
       << "\"heading\": 270.0"
       << "}";

    msg.data = ss.str();
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published fake data: %s", msg.data.c_str());
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimDataPublisher>());
  rclcpp::shutdown();
  return 0;
}
