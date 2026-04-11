# 最低 CMake 版本要求。
cmake_minimum_required(VERSION 3.8)
# 编码器测速功能包。
project(encoder_vel)

# 打开编译告警，帮助尽早发现代码问题。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS2 与第三方依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

# libgpiod 用于直接读取 Linux GPIO 边沿事件。
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

# 构建编码器测速节点。
add_executable(encoder_vel_node src/encoder_vel_node.cpp)

# 添加 libgpiod 头文件搜索路径。
target_include_directories(encoder_vel_node PRIVATE
  ${LIBGPIOD_INCLUDE_DIRS}
)

# 链接 libgpiod 动态库。
target_link_libraries(encoder_vel_node
  ${LIBGPIOD_LIBRARIES}
)

# 追加 libgpiod 编译选项。
target_compile_options(encoder_vel_node PRIVATE ${LIBGPIOD_CFLAGS_OTHER})

# 追加 libgpiod 库目录。
target_link_directories(encoder_vel_node PRIVATE ${LIBGPIOD_LIBRARY_DIRS})

# 声明 ROS2 依赖。
ament_target_dependencies(encoder_vel_node
  rclcpp
  std_msgs
)

# 安装节点，供 ros2 run 调用。
install(TARGETS encoder_vel_node
  DESTINATION lib/${PROJECT_NAME}
)

# 导出 ament 包信息。
ament_package()
