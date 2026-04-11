"""
GPIO IO 控制节点。

功能：
- 订阅 Joy 手柄消息，将指定按钮映射到 GPIO 输出。
- 订阅 Bool 命令话题，允许其他 ROS 节点直接控制 GPIO。
- 发布 GPIO 当前逻辑状态，方便联调与状态监测。

说明：
- Joy 输入和话题输入最终都会走同一套 `_set_output()` 逻辑，
  这样可以保证两种控制方式行为一致。
- 支持低电平有效/高电平有效两种输出模式，便于适配继电器、驱动板等硬件。
"""

import pigpio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class IoControlNode(Node):
    """将 Joy 按钮与 Bool 控制话题统一映射到 pigpio GPIO 输出。"""

    def __init__(self) -> None:
        super().__init__('io_control_node')

        # ---------------- 参数区 ----------------
        # joy_topic: 手柄消息来源。
        # enable_joy: 是否启用 Joy 控制；关闭后仅接受话题命令。
        self.joy_topic = self.declare_parameter('joy_topic', '/joy').value
        self.enable_joy = bool(self.declare_parameter('enable_joy', True).value)
        # 受控 GPIO，使用 BCM 编号。
        self.button1_pin = int(self.declare_parameter('button1_pin', 27).value)
        self.button3_pin = int(self.declare_parameter('button3_pin', 17).value)
        # 状态发布话题：外部看到的是“逻辑激活/未激活”，不是原始电平。
        self.button1_state_topic = self.declare_parameter(
            'button1_state_topic', '/gpio/bcm27_active'
        ).value
        self.button3_state_topic = self.declare_parameter(
            'button3_state_topic', '/gpio/bcm17_active'
        ).value
        self.button1_command_topic = self.declare_parameter(
            'button1_command_topic', '/gpio/bcm27_set'
        ).value
        self.button3_command_topic = self.declare_parameter(
            'button3_command_topic', '/gpio/bcm17_set'
        ).value
        # active_low=True:
        # - 激活态输出 0
        # - 未激活态输出 1
        self.active_low = bool(self.declare_parameter('active_low', True).value)

        # 手柄按钮索引与 GPIO 的映射关系。
        self._button_map = {
            0: self.button1_pin,
            4: self.button3_pin,
        }
        # 仅用于日志展示，便于排查。
        self._button_names = {
            0: '按钮 1',
            4: '按钮 3',
        }
        # 记录每一路 GPIO 当前实际输出电平，避免重复写引脚。
        self._pin_levels = {
            self.button1_pin: self._inactive_level(),
            self.button3_pin: self._inactive_level(),
        }

        # 与 pigpio 守护进程建立连接，实际 GPIO 读写都经由它完成。
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError('pigpio daemon not running. Run: sudo systemctl start pigpiod')

        # 初始化所有 GPIO 为输出模式，并设置到默认非激活态。
        for pin in self._pin_levels:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, self._inactive_level())

        # 为每一路 GPIO 创建状态发布器。
        self._state_publishers = {
            0: self.create_publisher(Bool, self.button1_state_topic, 10),
            4: self.create_publisher(Bool, self.button3_state_topic, 10),
        }
        # 为每一路 GPIO 创建命令订阅。
        # 使用 lambda 绑定按钮索引，从而复用统一回调逻辑。
        self._command_subscriptions = [
            self.create_subscription(
                Bool,
                self.button1_command_topic,
                lambda msg: self._command_callback(0, msg),
                10,
            ),
            self.create_subscription(
                Bool,
                self.button3_command_topic,
                lambda msg: self._command_callback(4, msg),
                10,
            ),
        ]
        self._joy_subscription = None
        if self.enable_joy:
            # 仅在启用手柄控制时才创建 Joy 订阅。
            self._joy_subscription = self.create_subscription(
                Joy, self.joy_topic, self._joy_callback, 10
            )

        if self.enable_joy:
            self.get_logger().info(
                f'监听 {self.joy_topic}，按钮 1/3 分别控制 BCM{self.button1_pin} / BCM{self.button3_pin}'
            )
        else:
            self.get_logger().info('未启用 Joy 订阅，仅使用话题命令控制 GPIO')
        self.get_logger().info(
            f'命令话题: 按钮 1 -> {self.button1_command_topic}，按钮 3 -> {self.button3_command_topic}'
        )
        self.get_logger().info(
            f'状态话题: 按钮 1 -> {self.button1_state_topic}，按钮 3 -> {self.button3_state_topic}'
        )
        self.get_logger().info(
            f'输出模式: {"低电平有效" if self.active_low else "高电平有效"}'
        )
        self._publish_all_states()

    def _active_level(self) -> int:
        # 逻辑“激活”对应的真实输出电平。
        return 0 if self.active_low else 1

    def _inactive_level(self) -> int:
        # 逻辑“未激活”对应的真实输出电平。
        return 1 - self._active_level()

    def _joy_callback(self, msg: Joy) -> None:
        # 遍历被映射的按钮索引。
        # 某个按钮索引存在且值为 1，表示该按钮被按下。
        for button_index in self._button_map:
            is_pressed = button_index < len(msg.buttons) and msg.buttons[button_index] == 1
            self._set_output(button_index, bool(is_pressed), source='Joy')

    def _command_callback(self, button_index: int, msg: Bool) -> None:
        # Bool.data 直接表示“逻辑上是否激活”。
        self._set_output(button_index, msg.data, source='话题命令')

    def _set_output(self, button_index: int, active: bool, source: str) -> None:
        # 统一输出控制入口：
        # 1) 根据 active_low/high 规则换算真实电平
        # 2) 避免重复写相同状态
        # 3) 写 GPIO
        # 4) 发布当前状态
        # 5) 输出日志
        pin = self._button_map[button_index]
        output_level = self._active_level() if active else self._inactive_level()
        if self._pin_levels[pin] == output_level:
            return

        self._pin_levels[pin] = output_level
        self.pi.write(pin, output_level)
        self._publish_state(button_index)
        self.get_logger().info(
            f'{source} {self._button_names[button_index]} '
            f'{"激活" if active else "释放"} -> BCM{pin}={output_level}'
        )

    def _publish_state(self, button_index: int) -> None:
        # 对外统一发布“逻辑激活状态”，屏蔽底层电平细节。
        pin = self._button_map[button_index]
        msg = Bool()
        msg.data = self._pin_levels[pin] == self._active_level()
        self._state_publishers[button_index].publish(msg)

    def _publish_all_states(self) -> None:
        # 启动后立即发布一次完整状态，便于其他节点立刻获知当前输出。
        for button_index in self._button_map:
            self._publish_state(button_index)

    def shutdown(self) -> None:
        # 退出时统一恢复到非激活态，避免 GPIO 在进程结束后仍保持吸合状态。
        for pin in self._pin_levels:
            self._pin_levels[pin] = self._inactive_level()
            self.pi.write(pin, self._inactive_level())
        self._publish_all_states()
        self.pi.stop()


def main() -> None:
    rclpy.init()
    node = IoControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
