cmake_minimum_required(VERSION 3.8)
project(encoder_vel)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

add_executable(encoder_vel_node src/encoder_vel_node.cpp)

target_include_directories(encoder_vel_node PRIVATE
  ${LIBGPIOD_INCLUDE_DIRS}
)

target_link_libraries(encoder_vel_node
  ${LIBGPIOD_LIBRARIES}
)

target_compile_options(encoder_vel_node PRIVATE ${LIBGPIOD_CFLAGS_OTHER})

target_link_directories(encoder_vel_node PRIVATE ${LIBGPIOD_LIBRARY_DIRS})

ament_target_dependencies(encoder_vel_node
  rclcpp
  std_msgs
)

install(TARGETS encoder_vel_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
