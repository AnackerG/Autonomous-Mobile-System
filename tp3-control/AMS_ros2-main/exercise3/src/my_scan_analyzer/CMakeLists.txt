cmake_minimum_required(VERSION 3.8)
project(my_scan_analyzer)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED) # <--- ADD THIS

add_executable(scan_analyzer_node src/scan_analyzer.cpp)

# Add OpenCV to dependencies
target_link_libraries(scan_analyzer_node ${OpenCV_LIBRARIES})
ament_target_dependencies(scan_analyzer_node rclcpp sensor_msgs OpenCV)

install(TARGETS
  scan_analyzer_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()