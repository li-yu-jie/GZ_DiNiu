cmake_minimum_required(VERSION 3.8)
project(steer_closed_loop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

add_executable(steer_closed_loop_node
  src/steer_closed_loop_node.cpp
)

target_include_directories(steer_closed_loop_node PRIVATE
  ${LIBGPIOD_INCLUDE_DIRS}
)

target_link_directories(steer_closed_loop_node PRIVATE
  ${LIBGPIOD_LIBRARY_DIRS}
)

target_link_libraries(steer_closed_loop_node
  ${LIBGPIOD_LIBRARIES}
  Threads::Threads
)

target_compile_options(steer_closed_loop_node PRIVATE ${LIBGPIOD_CFLAGS_OTHER})

ament_target_dependencies(steer_closed_loop_node
  rclcpp
  std_msgs
)

install(TARGETS steer_closed_loop_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
