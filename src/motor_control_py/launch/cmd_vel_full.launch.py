"""
/cmd_vel 驱动完整链路：编码器、驱动控制、转向闭环、IMU。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # 默认参数文件路径：
    # 按当前工程要求，集中放在 ackermann_odom 包的 config 目录中。
    default_motor_control_params = os.path.join(
        get_package_share_directory("ackermann_odom"),
        "config",
        "motor_control_defaults.yaml",
    )

    # 这些启动参数都支持在命令行覆盖。
    # 例如：
    # ros2 launch motor_control_py cmd_vel_full.launch.py enable_imu:=true
    start_pigpiod = LaunchConfiguration("start_pigpiod")
    pigpiod_use_sudo = LaunchConfiguration("pigpiod_use_sudo")
    enable_steer = LaunchConfiguration("enable_steer")
    enable_imu = LaunchConfiguration("enable_imu")
    enable_odom = LaunchConfiguration("enable_odom")
    enable_io_control = LaunchConfiguration("enable_io_control")
    motor_control_params = LaunchConfiguration("motor_control_params")
    pigpiod_ready_delay = LaunchConfiguration("pigpiod_ready_delay")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_pigpiod",
                default_value="true",
                description="是否在启动时自动拉起 pigpiod 守护进程。",
            ),
            DeclareLaunchArgument(
                "pigpiod_use_sudo",
                default_value="false",
                description="启动 pigpiod 时是否使用 sudo -n。",
            ),
            DeclareLaunchArgument(
                "enable_steer",
                default_value="true",
                description="是否启动转向闭环节点。",
            ),
            DeclareLaunchArgument(
                "enable_imu",
                default_value="false",
                description="是否启动 IMU 节点。",
            ),
            DeclareLaunchArgument(
                "enable_odom",
                default_value="true",
                description="是否启动阿克曼里程计节点。",
            ),
            DeclareLaunchArgument(
                "enable_io_control",
                default_value="true",
                description="是否启动 GPIO IO 控制节点。",
            ),
            DeclareLaunchArgument(
                "motor_control_params",
                default_value=default_motor_control_params,
                description="motor_control_node 默认参数 YAML 文件路径。",
            ),
            DeclareLaunchArgument(
                "pigpiod_ready_delay",
                default_value="1.0",
                description="给 pigpiod 预留的启动时间，避免依赖 pigpio 的节点抢跑。",
            ),
            # 根据参数决定是否启动 pigpiod。
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
            # 速度反馈链路入口：编码器测速。
            Node(
                package="encoder_vel",
                executable="encoder_vel_node",
                name="encoder_vel_node",
                output="screen",
            ),
            # 里程计解算。
            Node(
                package="ackermann_odom",
                executable="ackermann_odom_node",
                name="ackermann_odom_node",
                output="screen",
                parameters=[motor_control_params],
                condition=IfCondition(enable_odom),
            ),
            # 依赖 pigpio 的节点延迟启动，降低 pigpiod 尚未 ready 时的启动失败概率。
            TimerAction(
                period=pigpiod_ready_delay,
                actions=[
                    # 主控制节点：订阅 /cmd_vel，输出驱动 PWM 与转向目标。
                    Node(
                        package="motor_control_py",
                        executable="motor_control_node",
                        name="motor_control_node",
                        output="screen",
                        parameters=[motor_control_params],
                    ),
                    # GPIO 辅助控制。
                    Node(
                        package="io_control_py",
                        executable="io_control_node",
                        name="io_control_node",
                        output="screen",
                        condition=IfCondition(enable_io_control),
                    ),
                ],
            ),
            # 转向执行闭环。
            Node(
                package="steer_closed_loop",
                executable="steer_closed_loop_node",
                name="steer_closed_loop_node",
                output="screen",
                parameters=[motor_control_params],
                condition=IfCondition(enable_steer),
            ),
            # IMU 可选启动。
            Node(
                package="imu_bno08x",
                executable="imu_bno08x_node",
                name="imu_bno08x_node",
                output="screen",
                condition=IfCondition(enable_imu),
            ),
        ]
    )
