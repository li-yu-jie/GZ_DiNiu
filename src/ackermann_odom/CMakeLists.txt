# 最低 CMake 版本要求。
cmake_minimum_required(VERSION 3.8)
# 功能包名称，需要与 package.xml 中保持一致。
project(ackermann_odom)

# 打开常用编译告警，便于在开发阶段尽早发现潜在问题。
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找构建和运行本节点所需的 ROS2 依赖。
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 生成可执行节点：
# ackermann_odom_node 会由 src/ackermann_odom_node.cpp 编译得到。
add_executable(ackermann_odom_node
  src/ackermann_odom_node.cpp
)

# 声明该节点链接到的 ROS2 依赖。
ament_target_dependencies(ackermann_odom_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

# 安装目标节点到 ROS2 约定目录：
# 安装后可通过 `ros2 run ackermann_odom ackermann_odom_node` 启动。
install(TARGETS ackermann_odom_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件目录：
# 供其他 launch 文件按 share/ackermann_odom/config 路径读取默认参数。
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# 导出 ament 包信息。
ament_package()
