"""
/cmd_vel 驱动完整链路：编码器、驱动控制、转向闭环、IMU。
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    start_pigpiod = LaunchConfiguration("start_pigpiod")
    pigpiod_use_sudo = LaunchConfiguration("pigpiod_use_sudo")
    enable_steer = LaunchConfiguration("enable_steer")
    enable_imu = LaunchConfiguration("enable_imu")
    enable_odom = LaunchConfiguration("enable_odom")
    enable_io_control = LaunchConfiguration("enable_io_control")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_pigpiod",
                default_value="true",
                description="Start pigpiod daemon.",
            ),
            DeclareLaunchArgument(
                "pigpiod_use_sudo",
                default_value="false",
                description="Use sudo -n when starting pigpiod.",
            ),
            DeclareLaunchArgument(
                "enable_steer",
                default_value="true",
                description="Launch steering closed-loop node.",
            ),
            DeclareLaunchArgument(
                "enable_imu",
                default_value="false",
                description="Launch IMU node.",
            ),
            DeclareLaunchArgument(
                "enable_odom",
                default_value="true",
                description="Launch the Ackermann odometry node.",
            ),
            DeclareLaunchArgument(
                "enable_io_control",
                default_value="true",
                description="Launch the GPIO IO control node.",
            ),
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
            Node(
                package="encoder_vel",
                executable="encoder_vel_node",
                name="encoder_vel_node",
                output="screen",
            ),
            Node(
                package="ackermann_odom",
                executable="ackermann_odom_node",
                name="ackermann_odom_node",
                output="screen",
                condition=IfCondition(enable_odom),
            ),
            Node(
                package="motor_control_py",
                executable="motor_control_node",
                name="motor_control_node",
                output="screen",
                parameters=[
                    {
                        "steer_mode": "direct",
                        "cmd_vel_axis": "x",
                        "ackermann_wheelbase_m": 1.38,
                        "ackermann_min_speed_m_s": 0.12,
                        "ackermann_max_steer_angle_rad": 0.7853981633974483,
                    }
                ],
            ),
            Node(
                package="steer_closed_loop",
                executable="steer_closed_loop_node",
                name="steer_closed_loop_node",
                output="screen",
                condition=IfCondition(enable_steer),
            ),
            Node(
                package="imu_bno08x",
                executable="imu_bno08x_node",
                name="imu_bno08x_node",
                output="screen",
                condition=IfCondition(enable_imu),
            ),
            Node(
                package="io_control_py",
                executable="io_control_node",
                name="io_control_node",
                output="screen",
                condition=IfCondition(enable_io_control),
            ),
        ]
    )
