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
        # 仅更新目标值，真正的 PID 运算在 step() 中按固定周期执行。
        self.target = value

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        # 对称限幅到 [-limit, +limit]。
        return max(-limit, min(limit, value))

    def reset(self, *, clear_integral: bool = True, clear_output: bool = False):
        # 重置 PID 内部状态。
        # 常用于停车、换向、目标大幅变化等场景，减少残余积分带来的顿挫。
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
        # ---------------- 驱动硬件参数 ----------------
        # 驱动电机 PWM 引脚（BCM 编号）
        self.pwm_gpio = self.declare_parameter('pwm_gpio', 18).value
        # 驱动电机方向控制引脚（BCM 编号）
        self.dir_gpio = self.declare_parameter('dir_gpio', 26).value
        # PWM 频率（Hz）
        self.pwm_freq = self.declare_parameter('pwm_freq_hz', 100000).value
        # 是否反转方向引脚逻辑
        self.invert_dir = self.declare_parameter('invert_dir', False).value
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
        # 速度反馈订阅话题（Float64，单位 m/s）
        self.feedback_topic = self.declare_parameter('feedback_topic', 'linear_velocity').value
        # 是否发布转向控制指令
        self.enable_steer_cmd = bool(self.declare_parameter('enable_steer_cmd', True).value)
        # 转向目标角度发布话题（Float64，单位 rad）
        self.steer_topic = self.declare_parameter('steer_topic', 'target_steer').value
        # 转向模式：
        # - ackermann: angular.z 作为车体角速度（rad/s），换算前轮转角
        # - direct: angular.z 直接作为前轮目标转角（rad）
        requested_mode = str(self.declare_parameter('steer_mode', 'direct').value).lower()
        if requested_mode not in ('ackermann', 'direct'):
            self.get_logger().warn(
                f"steer_mode={requested_mode} is not supported, forcing to 'direct'"
            )
            requested_mode = 'direct'
        self.steer_mode = requested_mode
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
        # PID 比例系数
        self.kp = self.declare_parameter('kp', 90).value
        # PID 积分系数
        self.ki = self.declare_parameter('ki', 15).value
        # PID 微分系数
        self.kd = self.declare_parameter('kd', 0).value
        # 积分项绝对值上限（防积分饱和）
        self.i_max = self.declare_parameter('i_max',500.0).value
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

        # ---------------- 参数合法性检查 ----------------
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
            raise RuntimeError("cmd_vel_axis must be 'x'")
        if self.steer_mode not in ('ackermann', 'direct'):
            raise RuntimeError("steer_mode must be 'ackermann' or 'direct'")
        if self.ackermann_wheelbase_m <= 0.0:
            raise RuntimeError('ackermann_wheelbase_m must be > 0')
        if self.ackermann_min_speed_m_s < 0.0:
            raise RuntimeError('ackermann_min_speed_m_s must be >= 0')
        if self.ackermann_max_steer_angle_rad <= 0.0:
            raise RuntimeError('ackermann_max_steer_angle_rad must be > 0')
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

        # 建立 pigpio 连接，用于控制方向引脚和硬件 PWM。
        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError('pigpio daemon not running. Run: sudo pigpiod')

        # 初始化方向引脚，PWM 在真正输出前由 hardware_PWM 动态设置。
        self.pi.set_mode(self.dir_gpio, pigpio.OUTPUT)
        self.pi.write(self.dir_gpio, 0)

        # 创建驱动轮闭环控制器。
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
        self.control_tick = 0
        # 反向切换时，新的目标速度会先暂存在这里，待车速接近 0 再生效。
        self.pending_target_speed = None
        # 用滑动窗口做中值滤波，抑制单点脉冲噪声。
        self.feedback_samples = deque(maxlen=self.feedback_median_window)
        self.last_feedback_filtered = None
        # 记录上一次目标速度，用于判断是否发生“大目标阶跃”。
        self.last_target_for_reset = 0.0
        # 缓存最近一次目标转角，仅作状态保存/调试用途。
        self.steer_target_rad = 0.0

        period = 1.0 / self.control_hz
        self.timer = self.create_timer(period, self.control_step)

    @staticmethod
    def percent_to_duty(percent: float) -> int:
        # pigpio.hardware_PWM 的占空比范围是 0~1,000,000，
        # 因此百分比需要转换成百万分比。
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
        # Ackermann 固定取 linear.x 作为线速度目标（m/s）
        target_speed = float(msg.linear.x) * self.cmd_vel_scale
        # 获取当前目标与当前反馈，用于判断是否需要“先停稳再换向”。
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
            # 当前仍在运动且请求反向时，不直接硬切目标。
            # 先命令减速到 0，等接近静止后再切换成新的反向目标。
            self.pending_target_speed = target_speed
            self.drive_pid.update_target(0.0)
        else:
            self.pending_target_speed = None
            self.drive_pid.update_target(target_speed)

        # 目标速度大幅变化时清积分，能减少速度切换时的历史残留影响。
        if abs(target_speed - self.last_target_for_reset) > 0.12:
            self.drive_pid.reset(clear_integral=True, clear_output=False)
        self.last_target_for_reset = target_speed

        # 由 vx 与 ω 计算/直通转向目标角（rad），并发布到目标话题
        if self.steer_mode == 'ackermann':
            target_steer_rad = self._compute_steer_target_rad(
                target_speed=target_speed,
                yaw_rate_rad_s=float(msg.angular.z),
            )
        else:
            target_steer_rad = self._clamp_abs(float(msg.angular.z), self.ackermann_max_steer_angle_rad)
            self.steer_target_rad = target_steer_rad
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
        # 反馈过滤链路：
        # 1) 中值滤波，抑制偶发尖峰；
        # 2) 跳变限幅，抑制相邻周期不合理突变；
        # 3) 结果再送入 PID 内部一阶低通滤波。
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
        # 控制定时主循环：
        # 1) 若存在待切换的反向目标，先检查车是否停稳；
        # 2) 执行 PID；
        # 3) 执行停车锁定、最小有效 PWM、方向输出和硬件 PWM 输出。
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

            # 零速锁定：
            # 目标和反馈都接近 0 时，强制输出 0 并清理 PID 状态，
            # 避免因为残余积分或测量噪声导致电机在停车点附近抖动。
            if abs(target) <= self.deadband and abs(filtered) <= self.switch_dir_stop_speed_threshold:
                output = 0.0
                self.drive_pid.reset(clear_integral=True, clear_output=True)

            # 非零输出过小时，电机可能克服不了静摩擦而不转。
            # 因此把它提升到最小有效 PWM。
            if (
                0.0 < abs(output) < self.min_effective_pwm_percent and
                abs(target) > self.deadband
            ):
                output = self.min_effective_pwm_percent if target > 0.0 else -self.min_effective_pwm_percent

            # 输出符号决定方向引脚，invert_dir 用于适配接线极性。
            dir_level = 0 if output >= 0.0 else 1
            if self.invert_dir:
                dir_level = 1 - dir_level
            self.pi.write(self.dir_gpio, dir_level)

            # 输出绝对值换算成硬件 PWM 占空比。
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
    # 退出前显式关闭 PWM 和方向输出，避免进程结束后电机残留驱动。
    node.pi.hardware_PWM(node.pwm_gpio, 0, 0)
    node.pi.write(node.dir_gpio, 0)
    node.pi.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
