cmake_minimum_required(VERSION 3.8)
project(simple_boat)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# Executable
add_executable(boat_mover_node src/boat_control.cpp)

# Link Libraries
ament_target_dependencies(boat_mover_node
  rclcpp
  std_msgs
  mavros_msgs
)

install(TARGETS
  boat_mover_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()