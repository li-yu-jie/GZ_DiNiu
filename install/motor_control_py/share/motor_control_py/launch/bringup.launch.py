from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    start_pigpiod = LaunchConfiguration("start_pigpiod")
    pigpiod_use_sudo = LaunchConfiguration("pigpiod_use_sudo")
    enable_imu = LaunchConfiguration("enable_imu")
    enable_steer = LaunchConfiguration("enable_steer")

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
                "enable_imu",
                default_value="true",
                description="Launch the IMU node.",
            ),
            DeclareLaunchArgument(
                "enable_steer",
                default_value="true",
                description="Launch the steering closed-loop node.",
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
