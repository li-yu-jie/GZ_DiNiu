from collections import deque
from time import monotonic, sleep

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


class SpeedPlotNode(Node):
    def __init__(self):
        super().__init__('speed_plot_node')

        self.cmd_topic = self.declare_parameter('cmd_topic', '/cmd_vel').value
        self.feedback_topic = self.declare_parameter('feedback_topic', 'linear_velocity').value
        self.cmd_vel_axis = str(self.declare_parameter('cmd_vel_axis', 'x').value).lower()
        self.cmd_vel_scale = float(self.declare_parameter('cmd_vel_scale', 1.0).value)
        self.window_s = float(self.declare_parameter('window_s', 12.0).value)
        self.plot_hz = float(self.declare_parameter('plot_hz', 20.0).value)

        if self.cmd_vel_axis not in ('x', 'y', 'z'):
            raise RuntimeError("cmd_vel_axis must be one of: 'x', 'y', 'z'")
        if self.window_s <= 0.0:
            raise RuntimeError('window_s must be > 0')
        if self.plot_hz <= 0.0:
            raise RuntimeError('plot_hz must be > 0')

        self.current_target = 0.0
        self.times = deque()
        self.targets = deque()
        self.feedbacks = deque()
        self.start_t = monotonic()

        self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 10)
        self.create_subscription(Float64, self.feedback_topic, self.on_feedback, 10)

        self.get_logger().info(
            f'speed_plot started: cmd_topic={self.cmd_topic} feedback_topic={self.feedback_topic} '
            f'axis={self.cmd_vel_axis} scale={self.cmd_vel_scale} window={self.window_s}s'
        )

    def on_cmd(self, msg: Twist):
        axis_value = {
            'x': float(msg.linear.x),
            'y': float(msg.linear.y),
            'z': float(msg.linear.z),
        }[self.cmd_vel_axis]
        self.current_target = axis_value * self.cmd_vel_scale

    def on_feedback(self, msg: Float64):
        t = monotonic() - self.start_t
        self.times.append(t)
        self.targets.append(self.current_target)
        self.feedbacks.append(float(msg.data))
        self._trim_old(t)

    def _trim_old(self, now_t: float):
        min_t = now_t - self.window_s
        while self.times and self.times[0] < min_t:
            self.times.popleft()
            self.targets.popleft()
            self.feedbacks.popleft()


def main():
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print(f'Failed to import matplotlib: {exc}')
        print('Please install matplotlib, e.g. sudo apt install -y python3-matplotlib')
        return

    rclpy.init()
    node = SpeedPlotNode()

    fig, ax = plt.subplots(figsize=(10, 5))
    (line_target,) = ax.plot([], [], label='target_speed (m/s)', linewidth=2)
    (line_feedback,) = ax.plot([], [], label='feedback_speed (m/s)', linewidth=2)
    text_info = ax.text(0.01, 0.98, '', transform=ax.transAxes, va='top')
    ax.set_title('Real-time Speed Curve')
    ax.set_xlabel('Time (s, rolling window)')
    ax.set_ylabel('Speed (m/s)')
    ax.legend(loc='upper right')
    ax.grid(True, linestyle='--', alpha=0.3)
    plt.tight_layout()
    plt.ion()
    plt.show(block=False)

    plot_period = 1.0 / node.plot_hz
    last_plot = 0.0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)

            now = monotonic()
            if (now - last_plot) < plot_period:
                continue
            last_plot = now

            if not node.times:
                continue

            now_t = node.times[-1]
            x = [t - now_t for t in node.times]
            y_target = list(node.targets)
            y_feedback = list(node.feedbacks)

            line_target.set_data(x, y_target)
            line_feedback.set_data(x, y_feedback)
            ax.set_xlim(-node.window_s, 0.0)

            y_min = min(min(y_target), min(y_feedback))
            y_max = max(max(y_target), max(y_feedback))
            margin = max(0.05, (y_max - y_min) * 0.15)
            ax.set_ylim(y_min - margin, y_max + margin)

            text_info.set_text(
                f'target={y_target[-1]:.3f} m/s\n'
                f'feedback={y_feedback[-1]:.3f} m/s\n'
                f'error={y_target[-1] - y_feedback[-1]:.3f} m/s'
            )

            fig.canvas.draw_idle()
            fig.canvas.flush_events()
            sleep(0.001)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close(fig)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
