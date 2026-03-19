cmake_minimum_required(VERSION 3.8)
project(ackermann_odom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(ackermann_odom_node
  src/ackermann_odom_node.cpp
)

ament_target_dependencies(ackermann_odom_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS ackermann_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
