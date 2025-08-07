cmake_minimum_required(VERSION 3.8)
project(cot_pipeline)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 啟動測試設定
enable_testing()
find_package(GTest REQUIRED)

# 1. 找 ROS2 依賴
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 2. include path（支援 JSON / XML / ROS node / CoT）
include_directories(
  ros_nodes/include
  CoT_event/include
  /usr/include/nlohmann
)

# 3. CoT library: cot_builder（僅含 builder / parser）
add_library(cot_builder
  CoT_event/src/event_builder.cpp
  CoT_event/src/sidc_parser.cpp
  CoT_event/src/time_formatter.cpp
  CoT_event/src/detail_builder.cpp
)
target_include_directories(cot_builder PUBLIC CoT_event/include)

# 4. CoT library: cot_validator（含 validator / json loader）
add_library(cot_validator
  CoT_event/src/cot_validator.cpp
  CoT_event/src/sidc_validator.cpp
)
target_include_directories(cot_validator PUBLIC CoT_event/include)
target_link_libraries(cot_validator tinyxml2)

# 5. ROS node 可執行檔
add_executable(SimDataPublisher
  ros_nodes/src/sim_data_publisher.cpp
  ros_nodes/src/processor_node.cpp
  ros_nodes/src/cot_encoder_node.cpp
  ros_nodes/src/main.cpp
)
ament_target_dependencies(SimDataPublisher
  rclcpp
  std_msgs
)
target_link_libraries(SimDataPublisher
  cot_builder
  cot_validator
)

# 6. CoT 測試用主程式（main.cpp）
add_executable(cot_test
  CoT_event/test/main.cpp
)
target_link_libraries(cot_test
  cot_builder
  cot_validator
)

# gtest
add_executable(unit_test
  CoT_event/test/test_time_formatter.cpp
  CoT_event/test/test_sidc_parser.cpp
  CoT_event/test/test_sidc_validator.cpp
  CoT_event/test/test_detail_builder.cpp
  CoT_event/test/test_event_builder.cpp
  CoT_event/test/test_cot_validator.cpp
)

target_link_libraries(unit_test PRIVATE cot_builder PRIVATE cot_validator GTest::gtest_main)

# 複製檔案至"build"中
add_custom_command(TARGET unit_test POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
        ${CMAKE_SOURCE_DIR}/CoT_event/resources
        $<TARGET_FILE_DIR:unit_test>/resources)


include(GoogleTest)
gtest_discover_tests(unit_test)

# 7. 安裝執行檔
install(TARGETS SimDataPublisher
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS cot_test
  DESTINATION lib/${PROJECT_NAME}
)

# 8. 測試設定
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  #set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()