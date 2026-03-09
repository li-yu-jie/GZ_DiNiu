"""
电机与转向控制主节点。

职责：
- 订阅 /cmd_vel（Nav2 默认输出的 Twist）
- 计算目标车速与 Ackermann 前轮转角
- 用 PID 控制驱动 PWM
"""
from collections import deque
import math
import pigpio
import rclpy
from time import monotonic
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float64


class PidController:
    """简化的位置式 PID，带低通滤波、死区、限幅与积分抗饱和。"""
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
        self.prev_error = None
        self.last_time = None

    def update_target(self, value: float):
        self.target = value

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def reset(self, *, clear_integral: bool = True, clear_output: bool = False):
        if clear_integral:
            self.integral = 0.0
        if clear_output:
            self.prev_output = 0.0
        self.prev_error = None

    def update_feedback(self, value: float):
        # 将测量值写入，同时进行一阶低通滤波，抑制速度测量噪声
        self.measured = value
        if self.filtered is None:
            self.filtered = value
        else:
            a = self.filter_alpha
            self.filtered = (a * value) + ((1.0 - a) * self.filtered)

    def step(self, now):
        # 单步 PID 计算：使用当前目标与过滤后的速度反馈生成 PWM 输出
        if self.measured is None or self.filtered is None:
            return None
        if self.last_time is None:
            self.last_time = now
            self.prev_error = self.target - self.filtered
            return None

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return None

        # 位置式 PID：
        # u(k) = Kp*e(k) + Ki*∫e(k) + Kd*(e(k)-e(k-1))/dt
        error = self.target - self.filtered
        if abs(error) < self.deadband:
            error = 0.0

        integral_candidate = self._clamp(self.integral + (error * dt), self.i_max)

        derivative = 0.0
        if self.prev_error is not None:
            derivative = (error - self.prev_error) / dt

        raw_candidate = (self.kp * error) + (self.ki * integral_candidate) + (self.kd * derivative)
        saturated_candidate = self._clamp(raw_candidate, self.max_pwm_percent)

        # 条件积分抗饱和：
        # 仅在未饱和或误差能推动输出离开饱和时，才更新积分项。
        if (
            raw_candidate == saturated_candidate or
            (saturated_candidate >= self.max_pwm_percent and error < 0.0) or
            (saturated_candidate <= -self.max_pwm_percent and error > 0.0)
        ):
            self.integral = integral_candidate

        raw_output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = self._clamp(raw_output, self.max_pwm_percent)

        # 输出斜率限制：限制单周期 PWM 变化量，避免突变导致抖动
        delta = output - self.prev_output
        max_step = self.max_pwm_step
        if delta > max_step:
            output = self.prev_output + max_step
        elif delta < -max_step:
            output = self.prev_output - max_step
        self.prev_output = output
        self.prev_error = error
        self.last_time = now

        return output, error, self.filtered


class MotorControlNode(Node):
    """驱动与转向控制节点：/cmd_vel -> 目标速度与转向角 -> PWM 与转向话题。"""
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
        # 速度指令订阅话题（Twist，Nav2 默认输出 /cmd_vel）
        self.cmd_topic = self.declare_parameter('cmd_topic', '/cmd_vel').value
        # Ackermann 固定使用 linear.x 作为前进速度 vx。
        requested_axis = str(self.declare_parameter('cmd_vel_axis', 'x').value).lower()
        if requested_axis != 'x':
            self.get_logger().warn(
                f"cmd_vel_axis={requested_axis} is not supported in Ackermann mode, forcing to 'x'"
            )
        self.cmd_vel_axis = 'x'
        # cmd_vel 线速度缩放系数（用于单位或标定修正）
        self.cmd_vel_scale = float(self.declare_parameter('cmd_vel_scale', 1.0).value)
        # 指令超时阈值（秒）
        self.cmd_timeout_s = float(self.declare_parameter('cmd_timeout_s', 0.0).value)
        # 速度反馈订阅话题（Float64，单位 m/s）
        self.feedback_topic = self.declare_parameter('feedback_topic', 'linear_velocity').value
        # 是否发布转向控制指令
        self.enable_steer_cmd = bool(self.declare_parameter('enable_steer_cmd', True).value)
        # 转向目标角度发布话题（Float64，单位 rad）
        self.steer_topic = self.declare_parameter('steer_topic', 'target_steer').value
        # 底盘运动学固定为 Ackermann。
        requested_mode = str(self.declare_parameter('steer_mode', 'ackermann').value).lower()
        if requested_mode != 'ackermann':
            self.get_logger().warn(
                f"steer_mode={requested_mode} is not supported, forcing to 'ackermann'"
            )
        self.steer_mode = 'ackermann'
        # Ackermann: 轴距（m），L = 前后轴距离
        self.ackermann_wheelbase_m = float(
            self.declare_parameter('ackermann_wheelbase_m', 1.38).value
        )
        # Ackermann: 最小等效速度阈值（m/s）
        # vx 很小时 atan(ω*L/vx) 会发散，使用 v_eff = sign(vx)*max(|vx|, v_min) 保护
        self.ackermann_min_speed_m_s = float(
            self.declare_parameter('ackermann_min_speed_m_s', 0.12).value
        )
        # Ackermann: 前轮目标转角限幅（rad），保护机械极限
        self.ackermann_max_steer_angle_rad = float(
            self.declare_parameter('ackermann_max_steer_angle_rad', math.pi / 4.0).value
        )
        # 转向命令超时自动回零（秒），0 表示关闭
        self.steer_cmd_timeout_s = float(
            self.declare_parameter('steer_cmd_timeout_s', 0.0).value
        )

        # PID 比例系数
        self.kp = self.declare_parameter('kp', 68.0).value
        # PID 积分系数
        self.ki = self.declare_parameter('ki', 22.0).value
        # PID 微分系数
        self.kd = self.declare_parameter('kd', 0.0).value
        # 积分项绝对值上限（防积分饱和）
        self.i_max = self.declare_parameter('i_max', 70.0).value
        # 控制循环频率（Hz）
        self.control_hz = self.declare_parameter('control_hz', 50.0).value
        # PWM 输出百分比上限（0~100）
        self.max_pwm_percent = self.declare_parameter('max_pwm_percent', 100.0).value
        # 反馈一阶低通滤波系数（0~1）
        self.filter_alpha = self.declare_parameter('filter_alpha', 0.04).value
        # 误差死区（|error| 小于该值视为 0）
        self.deadband = self.declare_parameter('deadband', 0.003).value
        # 每次控制周期允许的 PWM 最大变化量（百分比）
        self.max_pwm_step = self.declare_parameter('max_pwm_step', 1.0).value
        # 产生有效驱动的最小 PWM 百分比
        self.min_effective_pwm_percent = float(
            self.declare_parameter('min_effective_pwm_percent', 6.0).value
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
        self.feedback_jump_limit = float(self.declare_parameter('feedback_jump_limit', 0.20).value)

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
        if self.cmd_vel_axis != 'x':
            raise RuntimeError("Ackermann mode requires cmd_vel_axis to be 'x'")
        if self.steer_mode != 'ackermann':
            raise RuntimeError("steer_mode must be 'ackermann'")
        if self.ackermann_wheelbase_m <= 0.0:
            raise RuntimeError('ackermann_wheelbase_m must be > 0')
        if self.ackermann_min_speed_m_s < 0.0:
            raise RuntimeError('ackermann_min_speed_m_s must be >= 0')
        if self.ackermann_max_steer_angle_rad <= 0.0:
            raise RuntimeError('ackermann_max_steer_angle_rad must be > 0')
        if self.steer_cmd_timeout_s < 0.0:
            raise RuntimeError('steer_cmd_timeout_s must be >= 0')
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

        # ROS2 订阅与发布
        self.sub_cmd = self.create_subscription(Twist, self.cmd_topic, self.on_cmd_vel, 10)
        self.sub_fb = self.create_subscription(Float64, self.feedback_topic, self.on_feedback, 10)
        self.steer_pub = self.create_publisher(Float64, self.steer_topic, 10)
        self.last_cmd_time = monotonic()
        self.timed_out = False
        self.control_tick = 0
        self.pending_target_speed = None
        self.feedback_samples = deque(maxlen=self.feedback_median_window)
        self.last_feedback_filtered = None
        self.last_target_for_reset = 0.0
        self.steer_target_rad = 0.0

        period = 1.0 / self.control_hz
        self.timer = self.create_timer(period, self.control_step)

    @staticmethod
    def percent_to_duty(percent: float) -> int:
        percent = max(0.0, min(100.0, float(percent)))
        return int(round(percent * 10000))

    @staticmethod
    def _clamp_abs(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    def _compute_steer_target_rad(self, target_speed: float, yaw_rate_rad_s: float):
        # Ackermann：
        # 输入：vx=target_speed（m/s），ω=yaw_rate_rad_s（rad/s）
        # 前轮转角（rad）：α = atan(ω * L / v_eff)
        # 其中 L 为轴距，v_eff 为低速保护后的等效速度
        effective_speed = target_speed
        if abs(effective_speed) < self.ackermann_min_speed_m_s:
            # 低速保护：避免除零和过大转角，同时保持转向方向由 angular.z 决定
            effective_speed = self.ackermann_min_speed_m_s
        target_steer_rad = math.atan(
            (self.ackermann_wheelbase_m * yaw_rate_rad_s) / effective_speed
        )
        target_steer_rad = float(target_steer_rad)

        # 限制目标转角，保护机械极限
        target_steer_rad = self._clamp_abs(target_steer_rad, self.ackermann_max_steer_angle_rad)
        self.steer_target_rad = target_steer_rad
        return target_steer_rad

    def on_cmd_vel(self, msg: Twist):
        # /cmd_vel: linear.* + angular.z
        # 本项目默认使用 linear.x 作为 vx，angular.z 作为 ω
        self.last_cmd_time = monotonic()
        self.timed_out = False

        # Ackermann 固定取 linear.x 作为线速度目标（m/s）
        target_speed = float(msg.linear.x) * self.cmd_vel_scale
        current_target = self.drive_pid.target
        current_sign = self._sign(current_target)
        target_sign = self._sign(target_speed)
        feedback_speed = self.drive_pid.filtered
        if feedback_speed is None:
            feedback_speed = self.drive_pid.measured

        # 处理换向：若当前在运动且方向将反转，则先置零再切换目标
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

        # Large target step: clear integral memory to reduce speed-switch stutter.
        if abs(target_speed - self.last_target_for_reset) > 0.12:
            self.drive_pid.reset(clear_integral=True, clear_output=False)
        self.last_target_for_reset = target_speed

        # 由 vx 与 ω 计算转向目标角（rad），并发布到目标话题
        target_steer_rad = self._compute_steer_target_rad(
            target_speed=target_speed,
            yaw_rate_rad_s=float(msg.angular.z),
        )
        if self.enable_steer_cmd:
            steer_msg = Float64()
            steer_msg.data = target_steer_rad
            self.steer_pub.publish(steer_msg)

        self.get_logger().info(
            f'cmd_vel linear=({msg.linear.x:.3f},{msg.linear.y:.3f},{msg.linear.z:.3f}) '
            f'axis={self.cmd_vel_axis} target_speed={target_speed:.3f} m/s '
            f'angular.z={msg.angular.z:.3f} mode={self.steer_mode} '
            f'steer_target_rad={target_steer_rad:.3f}'
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
        if self.enable_steer_cmd and self.steer_cmd_timeout_s > 0.0:
            cmd_age_s = monotonic() - self.last_cmd_time
            if cmd_age_s >= self.steer_cmd_timeout_s:
                self.steer_target_rad = 0.0
                steer_msg = Float64()
                steer_msg.data = self.steer_target_rad
                self.steer_pub.publish(steer_msg)
                if (self.control_tick % self.drive_log_every_n) == 0:
                    self.get_logger().info(
                        f'steer timeout active age={cmd_age_s:.2f}s mode={self.steer_mode} '
                        f'cmd_target_rad={self.steer_target_rad:.3f}'
                    )

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
                self.drive_pid.reset(clear_integral=True, clear_output=True)

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
