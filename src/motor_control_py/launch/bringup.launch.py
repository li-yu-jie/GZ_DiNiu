"""
整车基础 bringup：启动编码器、驱动、转向闭环与 IMU。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # 默认参数文件路径：
    # 按当前工程要求，统一放在 ackermann_odom 包的 config 目录中管理。
    default_motor_control_params = os.path.join(
        get_package_share_directory("ackermann_odom"),
        "config",
        "motor_control_defaults.yaml",
    )

    # LaunchConfiguration 表示“运行 launch 时才能确定的参数”。
    # 这样可以通过命令行覆盖默认值，例如：
    # ros2 launch motor_control_py bringup.launch.py enable_imu:=false
    start_pigpiod = LaunchConfiguration("start_pigpiod")
    pigpiod_use_sudo = LaunchConfiguration("pigpiod_use_sudo")
    enable_imu = LaunchConfiguration("enable_imu")
    enable_steer = LaunchConfiguration("enable_steer")
    enable_odom = LaunchConfiguration("enable_odom")
    enable_io_control = LaunchConfiguration("enable_io_control")
    motor_control_params = LaunchConfiguration("motor_control_params")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_pigpiod",
                default_value="true",
                description="是否在 launch 中自动启动 pigpiod 守护进程。",
            ),
            DeclareLaunchArgument(
                "pigpiod_use_sudo",
                default_value="false",
                description="启动 pigpiod 时是否使用 sudo -n，适合需要提权的环境。",
            ),
            DeclareLaunchArgument(
                "enable_imu",
                default_value="true",
                description="是否启动 IMU 节点 imu_bno08x_node。",
            ),
            DeclareLaunchArgument(
                "enable_steer",
                default_value="true",
                description="是否启动转向闭环节点 steer_closed_loop_node。",
            ),
            DeclareLaunchArgument(
                "enable_odom",
                default_value="true",
                description="是否启动阿克曼里程计节点 ackermann_odom_node。",
            ),
            DeclareLaunchArgument(
                "enable_io_control",
                default_value="true",
                description="是否启动 GPIO IO 控制节点 io_control_node。",
            ),
            DeclareLaunchArgument(
                "motor_control_params",
                default_value=default_motor_control_params,
                description="motor_control_node 默认参数 YAML 文件路径。",
            ),
            # 如有需要，先启动 pigpiod 守护进程。
            GroupAction(
                actions=[
                    ExecuteProcess(
                        cmd=["sudo", "-n", "pigpiod"],
                        output="screen",
                        condition=IfCondition(pigpiod_use_sudo),
                    ),
                    ExecuteProcess(
                        cmd=["pigpiod"],
                        output="screen",
                        condition=UnlessCondition(pigpiod_use_sudo),
                    ),
                ],
                condition=IfCondition(start_pigpiod),
            ),
            # 编码器测速节点：发布线速度和编码器计数。
            Node(
                package="encoder_vel",
                executable="encoder_vel_node",
                name="encoder_vel_node",
                output="screen",
            ),
            # 里程计节点：根据速度与转角计算 /odom。
            Node(
                package="ackermann_odom",
                executable="ackermann_odom_node",
                name="ackermann_odom_node",
                output="screen",
                condition=IfCondition(enable_odom),
            ),
            # 底盘主控制节点：
            # 接收 /cmd_vel，输出驱动 PWM 和转向目标。
            Node(
                package="motor_control_py",
                executable="motor_control_node",
                name="motor_control_node",
                output="screen",
                # 默认参数改为从 YAML 配置文件加载，
                # 便于统一维护和现场调参。
                parameters=[motor_control_params],
            ),
            # IMU 采集节点。
            Node(
                package="imu_bno08x",
                executable="imu_bno08x_node",
                name="imu_bno08x_node",
                output="screen",
                condition=IfCondition(enable_imu),
            ),
            # 转向闭环控制节点。
            Node(
                package="steer_closed_loop",
                executable="steer_closed_loop_node",
                name="steer_closed_loop_node",
                output="screen",
                parameters=[motor_control_params],
                condition=IfCondition(enable_steer),
            ),
            # GPIO / 外设控制节点。
            Node(
                package="io_control_py",
                executable="io_control_node",
                name="io_control_node",
                output="screen",
                condition=IfCondition(enable_io_control),
            ),
        ]
    )
