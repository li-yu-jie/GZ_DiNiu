"""GPIO IO control node using Joy input and ROS Bool command topics."""

import pigpio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool


class IoControlNode(Node):
    """Map Joy buttons and Bool command topics to pigpio GPIO outputs."""

    def __init__(self) -> None:
        super().__init__('io_control_node')

        self.joy_topic = self.declare_parameter('joy_topic', '/joy').value
        self.enable_joy = bool(self.declare_parameter('enable_joy', True).value)
        self.button1_pin = int(self.declare_parameter('button1_pin', 27).value)
        self.button3_pin = int(self.declare_parameter('button3_pin', 17).value)
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
        self.active_low = bool(self.declare_parameter('active_low', True).value)

        self._button_map = {
            0: self.button1_pin,
            4: self.button3_pin,
        }
        self._button_names = {
            0: '按钮 1',
            4: '按钮 3',
        }
        self._pin_levels = {
            self.button1_pin: self._inactive_level(),
            self.button3_pin: self._inactive_level(),
        }

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError('pigpio daemon not running. Run: sudo systemctl start pigpiod')

        for pin in self._pin_levels:
            self.pi.set_mode(pin, pigpio.OUTPUT)
            self.pi.write(pin, self._inactive_level())

        self._state_publishers = {
            0: self.create_publisher(Bool, self.button1_state_topic, 10),
            4: self.create_publisher(Bool, self.button3_state_topic, 10),
        }
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
                lambda msg: self._command_callback(2, msg),
                10,
            ),
        ]
        self._joy_subscription = None
        if self.enable_joy:
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
        return 0 if self.active_low else 1

    def _inactive_level(self) -> int:
        return 1 - self._active_level()

    def _joy_callback(self, msg: Joy) -> None:
        for button_index in self._button_map:
            is_pressed = button_index < len(msg.buttons) and msg.buttons[button_index] == 1
            self._set_output(button_index, bool(is_pressed), source='Joy')

    def _command_callback(self, button_index: int, msg: Bool) -> None:
        self._set_output(button_index, msg.data, source='话题命令')

    def _set_output(self, button_index: int, active: bool, source: str) -> None:
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
        pin = self._button_map[button_index]
        msg = Bool()
        msg.data = self._pin_levels[pin] == self._active_level()
        self._state_publishers[button_index].publish(msg)

    def _publish_all_states(self) -> None:
        for button_index in self._button_map:
            self._publish_state(button_index)

    def shutdown(self) -> None:
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
