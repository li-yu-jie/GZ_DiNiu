# 最低 CMake 版本要求。
cmake_minimum_required(VERSION 3.8)
# 转向闭环控制功能包。
project(steer_closed_loop)

# 启用常用编译告警。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找 ROS2、线程库与第三方库。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)

# libgpiod 用于读取霍尔编码器与限位开关 GPIO。
pkg_check_modules(LIBGPIOD REQUIRED libgpiod)

# 构建转向闭环节点。
add_executable(steer_closed_loop_node
  src/steer_closed_loop_node.cpp
)

# 头文件搜索路径。
target_include_directories(steer_closed_loop_node PRIVATE
  ${LIBGPIOD_INCLUDE_DIRS}
)

# 动态库搜索路径。
target_link_directories(steer_closed_loop_node PRIVATE
  ${LIBGPIOD_LIBRARY_DIRS}
)

# 链接 libgpiod 和线程库。
target_link_libraries(steer_closed_loop_node
  ${LIBGPIOD_LIBRARIES}
  Threads::Threads
)

# 追加 libgpiod 编译选项。
target_compile_options(steer_closed_loop_node PRIVATE ${LIBGPIOD_CFLAGS_OTHER})

# 声明 ROS2 依赖。
ament_target_dependencies(steer_closed_loop_node
  rclcpp
  std_msgs
)

# 安装节点到 ROS2 标准执行目录。
install(TARGETS steer_closed_loop_node
  DESTINATION lib/${PROJECT_NAME}
)

# 导出 ament 包信息。
ament_package()
