#!/usr/bin/env python3
"""手柄遥控节点。

该节点负责将 /joy 手柄输入转换为 /cmd_vel 速度指令，便于在调试、
接管或无导航场景下直接控制机器人运动。
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


class OmniTeleopNode(Node):
    """将手柄摇杆输入映射为机器人速度命令。"""

    def __init__(self):
        super().__init__("omni_teleop")

        # 摇杆轴映射：
        # 1) axis_linear_x: 前后速度
        # 2) axis_linear_y: 左右速度（当前叉车通常不会实际使用）
        # 3) axis_angular_z: 角速度/转向指令
        self.declare_parameter("axis_linear_x", 1)
        self.declare_parameter("axis_linear_y", 0)
        self.declare_parameter("axis_angular_z", 2)

        # 速度缩放参数：
        # 数值越大，手柄同样偏转量对应的速度越大。
        self.declare_parameter("scale_linear_x", 0.7)
        self.declare_parameter("scale_linear_y", 0.7)
        self.declare_parameter("scale_angular_z", 1.0)

        # 按键参数：
        # enable_button 当前工程中保留为扩展位，turbo_button 用于加速。
        self.declare_parameter("enable_button", 4)
        self.declare_parameter("turbo_button", 5)

        # 死区参数：
        # 用于过滤摇杆轻微抖动，避免机器人在手柄回中时仍缓慢移动。
        self.declare_parameter("deadzone", 0.1)

        self.axis_linear_x = self.get_parameter("axis_linear_x").value
        self.axis_linear_y = self.get_parameter("axis_linear_y").value
        self.axis_angular_z = self.get_parameter("axis_angular_z").value
        self.scale_linear_x = self.get_parameter("scale_linear_x").value
        self.scale_linear_y = self.get_parameter("scale_linear_y").value
        self.scale_angular_z = self.get_parameter("scale_angular_z").value
        self.enable_button = self.get_parameter("enable_button").value
        self.turbo_button = self.get_parameter("turbo_button").value
        self.deadzone = self.get_parameter("deadzone").value

        # 发布 /cmd_vel 给底盘控制节点或下位机桥接节点。
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

        # 超时保护：
        # 超过 0.5 秒收不到手柄数据时主动下发零速度，避免失联后继续运动。
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.last_joy_time = self.get_clock().now()
        self.enabled = False

        self.get_logger().info("手柄遥控节点已启动")
        self.get_logger().info("左摇杆控制线速度，右摇杆控制角速度")
        self.get_logger().info("RB 可触发加速模式，未收到手柄数据时会自动停车")

    def apply_deadzone(self, value):
        """对摇杆输入应用死区映射。"""
        if abs(value) < self.deadzone:
            return 0.0
        return (value - math.copysign(self.deadzone, value)) / (1.0 - self.deadzone)

    def joy_callback(self, msg):
        """将 Joy 消息转换为 Twist。"""
        self.last_joy_time = self.get_clock().now()
        twist = Twist()

        # 当前项目版本默认允许手柄直接输出速度；
        # 如后续需要“按住某键才生效”，可在这里恢复 enable_button 判断。
        self.enabled = True

        turbo = len(msg.buttons) > self.turbo_button and msg.buttons[self.turbo_button]
        turbo_scale = 2.0 if turbo else 1.0

        x_raw = msg.axes[self.axis_linear_x] if len(msg.axes) > self.axis_linear_x else 0.0
        y_raw = msg.axes[self.axis_linear_y] if len(msg.axes) > self.axis_linear_y else 0.0
        z_raw = msg.axes[self.axis_angular_z] if len(msg.axes) > self.axis_angular_z else 0.0

        x_val = self.apply_deadzone(x_raw)
        y_val = self.apply_deadzone(y_raw)
        z_val = self.apply_deadzone(z_raw)

        # 坐标映射说明：
        # linear.x 代表前后运动，linear.y 预留给全向或横移底盘，
        # angular.z 代表绕 Z 轴旋转。阿克曼底盘通常只实际关心 x 和 z。
        twist.linear.x = x_val * self.scale_linear_x * turbo_scale
        twist.linear.y = y_val * self.scale_linear_y * turbo_scale
        twist.angular.z = z_val * self.scale_angular_z * turbo_scale

        mode = "TURBO" if turbo else "NORMAL"
        self.get_logger().debug(
            f"[{mode}] X:{twist.linear.x:.2f} Y:{twist.linear.y:.2f} Z:{twist.angular.z:.2f}"
        )

        self.cmd_vel_pub.publish(twist)

    def timer_callback(self):
        """手柄超时保护。"""
        now = self.get_clock().now()
        if (now - self.last_joy_time).nanoseconds > 5e8:
            if self.enabled:
                self.get_logger().warn("手柄信号超时，自动下发零速度")
                self.enabled = False
                self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = OmniTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
