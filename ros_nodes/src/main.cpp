#include "cot_pipeline/cot_encoder_node.hpp"
#include "cot_pipeline/processor_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto data_publisher = std::make_shared<CotEncoderNode>();
  auto processor = std::make_shared<ProcessorNode>();
  rclcpp::executors::SingleThreadedExecutor exec;

  try {
    exec.add_node(data_publisher);
    exec.add_node(processor);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception during node addition: %s", e.what());
  }

  exec.spin();
  rclcpp::shutdown();
}
