from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    start_pigpiod = LaunchConfiguration("start_pigpiod")
    enable_imu = LaunchConfiguration("enable_imu")
    enable_steer = LaunchConfiguration("enable_steer")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_pigpiod",
                default_value="false",
                description="Start pigpiod daemon (no sudo).",
            ),
            DeclareLaunchArgument(
                "enable_imu",
                default_value="true",
                description="Launch the IMU node.",
            ),
            DeclareLaunchArgument(
                "enable_steer",
                default_value="true",
                description="Launch the steering closed-loop node.",
            ),
            ExecuteProcess(
                cmd=["sudo", "pigpiod"],
                output="screen",
                condition=IfCondition(start_pigpiod),
            ),
            Node(
                package="encoder_vel",
                executable="encoder_vel_node",
                name="encoder_vel_node",
                output="screen",
            ),
            Node(
                package="motor_control_py",
                executable="motor_control_node",
                name="motor_control_node",
                output="screen",
            ),
            Node(
                package="imu_bno08x",
                executable="imu_bno08x_node",
                name="imu_bno08x_node",
                output="screen",
                condition=IfCondition(enable_imu),
            ),
            Node(
                package="steer_closed_loop",
                executable="steer_closed_loop_node",
                name="steer_closed_loop_node",
                output="screen",
                condition=IfCondition(enable_steer),
            ),
        ]
    )
