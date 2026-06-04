"""手柄遥控启动文件。

同时启动 joy_node 与 betop_teleop 的遥控转换节点，便于现场调试和人工接管。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "linear_scale",
            default_value="2.0",
            description="线速度缩放系数，值越大机器人前后运动越快",
        ),
        DeclareLaunchArgument(
            "angular_scale",
            default_value="2.5",
            description="角速度缩放系数，值越大机器人转向越快",
        ),

        # joy_node 负责读取 Linux 手柄设备并发布 /joy。
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            parameters=[{
                "dev": "/dev/input/js0",
                "deadzone": 0.05,
                "autorepeat_rate": 20.0,
            }],
            output="screen",
        ),

        # omni_teleop 负责将 /joy 转成 /cmd_vel。
        Node(
            package="betop_teleop",
            executable="omni_teleop",
            name="omni_teleop",
            parameters=[{
                "axis_linear_x": 1,  # 左摇杆前后 -> 机器人前后速度
                "axis_linear_y": 0,  # 左摇杆左右 -> 横向速度（阿克曼底盘一般不使用）
                "axis_angular_z": 2,  # 右摇杆左右 -> 角速度
                "scale_linear_x": LaunchConfiguration("linear_scale"),
                "scale_linear_y": LaunchConfiguration("linear_scale"),
                "scale_angular_z": LaunchConfiguration("angular_scale"),
                "enable_button": 4,  # 预留使能键
                "turbo_button": 5,  # 加速键
                "deadzone": 0.1,  # 避免摇杆回中抖动导致误动作
            }],
            output="screen",
        ),
    ])
