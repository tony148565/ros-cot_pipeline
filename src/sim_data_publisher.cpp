#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SimDataPublisher : public rclcpp::Node
{
public:
  SimDataPublisher()
  : Node("sim_data_publisher")
  {
    RCLCPP_INFO(this->get_logger(), "SimDataPublisher node started.");
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimDataPublisher>());
  rclcpp::shutdown();
  return 0;
}
