cmake_minimum_required(VERSION 3.8)
project(cot_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. find ROS2 依賴
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 2. include 自定義的 include path（ROS node + CoT_event）
include_directories(
  ros_nodes/include
  CoT_event/include
)

# 3. 建立 CoT_event 核心模組 library
add_library(cot_builder
  CoT_event/src/cot_builder.cpp
)
target_include_directories(cot_builder PUBLIC CoT_event/include)

# (之後你可以額外加上 cot_validator.cpp、cot_decoder.cpp 等）

# 4. ROS node 可執行檔（仍叫 SimDataPublisher）
add_executable(SimDataPublisher
  ros_nodes/src/sim_data_publisher.cpp
  ros_nodes/src/processor_node.cpp
  ros_nodes/src/cot_encoder_node.cpp
  ros_nodes/src/main.cpp
)

# 5. 指定 ROS 依賴
ament_target_dependencies(SimDataPublisher
  rclcpp
  std_msgs
)

# 6. 將 ROS node 連接 cot_builder library
target_link_libraries(SimDataPublisher cot_builder)

# 7. 若啟用測試（可留著不動）
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 8. 安裝
install(TARGETS SimDataPublisher
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
