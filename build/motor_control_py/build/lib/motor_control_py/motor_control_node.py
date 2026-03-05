from collections import deque
import pigpio
import rclpy
from time import monotonic
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


class PidController:
    def __init__(
        self,
        name: str,
        kp: float,
        ki: float,
        kd: float,
        i_max: float,
        deadband: float,
        max_pwm_percent: float,
        max_pwm_step: float,
        filter_alpha: float,
    ):
        self.name = name
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.i_max = i_max
        self.deadband = deadband
        self.max_pwm_percent = max_pwm_percent
        self.max_pwm_step = max_pwm_step
        self.filter_alpha = filter_alpha

        self.target = 0.0
        self.measured = None
        self.filtered = None
        self.integral = 0.0
        self.prev_output = 0.0
        self.prev_filtered = None
        self.last_time = None

    def update_target(self, value: float):
        self.target = value

    def update_feedback(self, value: float):
        self.measured = value
        if self.filtered is None:
            self.filtered = value
        else:
            a = self.filter_alpha
            self.filtered = (a * value) + ((1.0 - a) * self.filtered)

    def step(self, now):
        if self.measured is None or self.filtered is None:
            return None
        if self.last_time is None:
            self.last_time = now
            self.prev_filtered = self.filtered
            return None

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return None

        error = self.target - self.filtered
        if abs(error) < self.deadband:
            error = 0.0

        derivative = 0.0
        if self.prev_filtered is not None:
            derivative = -(self.filtered - self.prev_filtered) / dt

        integral_candidate = self.integral + (error * dt)
        integral_candidate = max(-self.i_max, min(self.i_max, integral_candidate))

        raw_output = (self.kp * error) + (self.ki * integral_candidate) + (self.kd * derivative)
        output = max(-self.max_pwm_percent, min(self.max_pwm_percent, raw_output))

        if output == raw_output:
            self.integral = integral_candidate
        else:
            if not ((output >= self.max_pwm_percent and error > 0.0) or
                    (output <= -self.max_pwm_percent and error < 0.0)):
                self.integral = integral_candidate

        delta = output - self.prev_output
        max_step = self.max_pwm_step
        if delta > max_step:
            output = self.prev_output + max_step
        elif delta < -max_step:
            output = self.prev_output - max_step
        self.prev_output = output
        self.prev_filtered = self.filtered
        self.last_time = now

        return output, error, self.filtered


class MotorControlNode(Node):
    @staticmethod
    def _sign(value: float) -> int:
        if value > 0.0:
            return 1
        if value < 0.0:
            return -1
        return 0

    def __init__(self):
        super().__init__('motor_control_node')
        # 驱动电机 PWM 引脚（BCM 编号）
        self.pwm_gpio = self.declare_parameter('pwm_gpio', 18).value
        # 驱动电机方向控制引脚（BCM 编号）
        self.dir_gpio = self.declare_parameter('dir_gpio', 26).value
        # PWM 频率（Hz）
        self.pwm_freq = self.declare_parameter('pwm_freq_hz', 100000).value
        # 是否反转方向引脚逻辑
        self.invert_dir = self.declare_parameter('invert_dir', True).value
        # 速度指令订阅话题（Twist）
        self.cmd_topic = self.declare_parameter('cmd_topic', '/cmd_vel').value
        # cmd_vel 使用的线速度轴：x / y / z
        self.cmd_vel_axis = str(self.declare_parameter('cmd_vel_axis', 'x').value).lower()
        # cmd_vel 线速度缩放系数
        self.cmd_vel_scale = float(self.declare_parameter('cmd_vel_scale', 1.0).value)
        # 指令超时阈值（秒）
        self.cmd_timeout_s = float(self.declare_parameter('cmd_timeout_s', 0.0).value)
        # 速度反馈订阅话题（Float64，单位 m/s）
        self.feedback_topic = self.declare_parameter('feedback_topic', 'linear_velocity').value
        # 是否发布转向控制指令
        self.enable_steer_cmd = bool(self.declare_parameter('enable_steer_cmd', True).value)
        # 转向目标角速度发布话题（Float64，单位 deg/s）
        self.steer_topic = self.declare_parameter('steer_topic', 'steer_target_rate_deg_s').value
        # 将 angular.z（rad/s）换算为转向角速度（deg/s）的比例
        self.steer_rate_scale_deg_per_rad_s = float(
            self.declare_parameter('steer_rate_scale_deg_per_rad_s', 30.0).value
        )

        # PID 比例系数
        self.kp = self.declare_parameter('kp', 85.0).value
        # PID 积分系数
        self.ki = self.declare_parameter('ki', 18.0).value
        # PID 微分系数
        self.kd = self.declare_parameter('kd', 0.0).value
        # 积分项绝对值上限（防积分饱和）
        self.i_max = self.declare_parameter('i_max', 60.0).value
        # 控制循环频率（Hz）
        self.control_hz = self.declare_parameter('control_hz', 50.0).value
        # PWM 输出百分比上限（0~100）
        self.max_pwm_percent = self.declare_parameter('max_pwm_percent', 100.0).value
        # 反馈一阶低通滤波系数（0~1）
        self.filter_alpha = self.declare_parameter('filter_alpha', 0.03).value
        # 误差死区（|error| 小于该值视为 0）
        self.deadband = self.declare_parameter('deadband', 0.008).value
        # 每次控制周期允许的 PWM 最大变化量（百分比）
        self.max_pwm_step = self.declare_parameter('max_pwm_step', 4.0).value
        # 产生有效驱动的最小 PWM 百分比
        self.min_effective_pwm_percent = float(
            self.declare_parameter('min_effective_pwm_percent', 1.0).value
        )
        # 换向前判定“接近静止”的速度阈值（m/s）
        self.switch_dir_stop_speed_threshold = float(
            self.declare_parameter('switch_dir_stop_speed_threshold', 0.05).value
        )
        # 驱动状态日志输出频率（每 N 次控制周期打印一次）
        self.drive_log_every_n = int(self.declare_parameter('drive_log_every_n', 10).value)
        # 反馈中值滤波窗口长度（奇数，>=1）
        self.feedback_median_window = int(self.declare_parameter('feedback_median_window', 5).value)
        # 单步反馈变化限幅（m/s），用于抑制偶发尖峰
        self.feedback_jump_limit = float(self.declare_parameter('feedback_jump_limit', 0.18).value)

        if self.control_hz <= 0.0:
            raise RuntimeError('control_hz must be > 0')
        if self.max_pwm_percent <= 0.0:
            raise RuntimeError('max_pwm_percent must be > 0')
        if not (0.0 < self.filter_alpha <= 1.0):
            raise RuntimeError('filter_alpha must be in (0, 1]')
        if self.deadband < 0.0:
            raise RuntimeError('deadband must be >= 0')
        if self.max_pwm_step <= 0.0:
            raise RuntimeError('max_pwm_step must be > 0')
        if self.min_effective_pwm_percent < 0.0:
            raise RuntimeError('min_effective_pwm_percent must be >= 0')
        if self.min_effective_pwm_percent > self.max_pwm_percent:
            raise RuntimeError('min_effective_pwm_percent must be <= max_pwm_percent')
        if self.switch_dir_stop_speed_threshold < 0.0:
            raise RuntimeError('switch_dir_stop_speed_threshold must be >= 0')
        if self.cmd_vel_axis not in ('x', 'y', 'z'):
            raise RuntimeError("cmd_vel_axis must be one of: 'x', 'y', 'z'")
        if self.cmd_timeout_s < 0.0:
            raise RuntimeError('cmd_timeout_s must be >= 0')
        if self.pwm_freq <= 0:
            raise RuntimeError('pwm_freq_hz must be > 0')
        if self.drive_log_every_n <= 0:
            raise RuntimeError('drive_log_every_n must be > 0')
        if self.feedback_median_window <= 0:
            raise RuntimeError('feedback_median_window must be > 0')
        if self.feedback_median_window % 2 == 0:
            raise RuntimeError('feedback_median_window must be odd')
        if self.feedback_jump_limit < 0.0:
            raise RuntimeError('feedback_jump_limit must be >= 0')

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError('pigpio daemon not running. Run: sudo pigpiod')

        self.pi.set_mode(self.dir_gpio, pigpio.OUTPUT)
        self.pi.write(self.dir_gpio, 0)

        self.drive_pid = PidController(
            name='drive',
            kp=self.kp,
            ki=self.ki,
            kd=self.kd,
            i_max=self.i_max,
            deadband=self.deadband,
            max_pwm_percent=self.max_pwm_percent,
            max_pwm_step=self.max_pwm_step,
            filter_alpha=self.filter_alpha,
        )

        self.sub_cmd = self.create_subscription(Twist, self.cmd_topic, self.on_cmd_vel, 10)
        self.sub_fb = self.create_subscription(Float64, self.feedback_topic, self.on_feedback, 10)
        self.steer_pub = self.create_publisher(Float64, self.steer_topic, 10)
        self.last_cmd_time = monotonic()
        self.timed_out = False
        self.control_tick = 0
        self.pending_target_speed = None
        self.feedback_samples = deque(maxlen=self.feedback_median_window)
        self.last_feedback_filtered = None

        period = 1.0 / self.control_hz
        self.timer = self.create_timer(period, self.control_step)

    @staticmethod
    def percent_to_duty(percent: float) -> int:
        percent = max(0.0, min(100.0, float(percent)))
        return int(round(percent * 10000))

    def on_cmd_vel(self, msg: Twist):
        self.last_cmd_time = monotonic()
        self.timed_out = False

        axis_value = {
            'x': float(msg.linear.x),
            'y': float(msg.linear.y),
            'z': float(msg.linear.z),
        }[self.cmd_vel_axis]
        target_speed = axis_value * self.cmd_vel_scale
        current_target = self.drive_pid.target
        current_sign = self._sign(current_target)
        target_sign = self._sign(target_speed)
        feedback_speed = self.drive_pid.filtered
        if feedback_speed is None:
            feedback_speed = self.drive_pid.measured

        reverse_requested = (
            target_sign != 0 and
            current_sign != 0 and
            target_sign != current_sign
        )
        moving = (
            feedback_speed is not None and
            abs(feedback_speed) > self.switch_dir_stop_speed_threshold
        )

        if reverse_requested and moving:
            self.pending_target_speed = target_speed
            self.drive_pid.update_target(0.0)
        else:
            self.pending_target_speed = None
            self.drive_pid.update_target(target_speed)

        steer_rate_deg_s = float(msg.angular.z) * self.steer_rate_scale_deg_per_rad_s
        if self.enable_steer_cmd:
            steer_msg = Float64()
            steer_msg.data = steer_rate_deg_s
            self.steer_pub.publish(steer_msg)

        self.get_logger().info(
            f'cmd_vel linear=({msg.linear.x:.3f},{msg.linear.y:.3f},{msg.linear.z:.3f}) '
            f'axis={self.cmd_vel_axis} target_speed={target_speed:.3f} m/s '
            f'angular.z={msg.angular.z:.3f} steer_rate_deg_s={steer_rate_deg_s:.2f}'
        )

    def on_feedback(self, msg: Float64):
        raw = float(msg.data)
        self.feedback_samples.append(raw)

        samples = sorted(self.feedback_samples)
        median = samples[len(samples) // 2]

        if self.last_feedback_filtered is not None and self.feedback_jump_limit > 0.0:
            delta = median - self.last_feedback_filtered
            if abs(delta) > self.feedback_jump_limit:
                median = self.last_feedback_filtered + (self.feedback_jump_limit * self._sign(delta))

        self.last_feedback_filtered = median
        self.drive_pid.update_feedback(median)

    def control_step(self):
        # Timeout protection disabled: do not force target_speed to 0 on cmd_vel timeout.
        self.control_tick += 1

        if self.pending_target_speed is not None:
            feedback_speed = self.drive_pid.filtered
            if feedback_speed is None:
                feedback_speed = self.drive_pid.measured
            if (
                feedback_speed is not None and
                abs(feedback_speed) <= self.switch_dir_stop_speed_threshold
            ):
                self.drive_pid.update_target(self.pending_target_speed)
                self.pending_target_speed = None

        now = self.get_clock().now()
        drive_result = self.drive_pid.step(now)
        if drive_result is not None:
            output, error, filtered = drive_result
            target = self.drive_pid.target

            # Zero-speed lock: avoid dithering around stop due to residual integral/noise.
            if abs(target) <= self.deadband and abs(filtered) <= self.switch_dir_stop_speed_threshold:
                output = 0.0
                self.drive_pid.integral = 0.0
                self.drive_pid.prev_output = 0.0

            # Keep low-speed command above static-friction threshold when non-zero.
            if (
                0.0 < abs(output) < self.min_effective_pwm_percent and
                abs(target) > self.deadband
            ):
                output = self.min_effective_pwm_percent if target > 0.0 else -self.min_effective_pwm_percent

            dir_level = 0 if output >= 0.0 else 1
            if self.invert_dir:
                dir_level = 1 - dir_level
            self.pi.write(self.dir_gpio, dir_level)

            percent = abs(output)
            duty = self.percent_to_duty(percent)
            self.pi.hardware_PWM(self.pwm_gpio, int(self.pwm_freq), duty)

            if (self.control_tick % self.drive_log_every_n) == 0:
                self.get_logger().info(
                    f'drive fb={filtered:.3f} err={error:.3f} out%={output:.2f} duty={duty} '
                    f'f={int(self.pwm_freq)}Hz'
                )


def main():
    rclpy.init()
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.pi.hardware_PWM(node.pwm_gpio, 0, 0)
    node.pi.write(node.dir_gpio, 0)
    node.pi.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
