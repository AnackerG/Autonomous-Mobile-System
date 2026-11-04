cmake_minimum_required(VERSION 3.8)
project(exercise4)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# 2. Add Executable
add_executable(calibrate calibrate.cc)

# 3. Handle includes and linking (The ROS2 way)
# This includes the rclcpp headers you are missing
target_link_libraries(calibrate ${OpenCV_LIBS})
ament_target_dependencies(calibrate rclcpp geometry_msgs)

# 4. Install
install(TARGETS
  calibrate
  DESTINATION lib/${PROJECT_NAME})

ament_package()