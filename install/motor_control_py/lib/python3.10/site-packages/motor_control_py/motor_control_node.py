import pigpio
import rclpy
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
    def __init__(self):
        super().__init__('motor_control_node')

        self.pwm_gpio = self.declare_parameter('pwm_gpio', 18).value
        self.dir_gpio = self.declare_parameter('dir_gpio', 26).value
        self.pwm_freq = self.declare_parameter('pwm_freq_hz', 100000).value
        self.invert_dir = self.declare_parameter('invert_dir', True).value
        self.cmd_topic = self.declare_parameter('cmd_topic', '/cmd_vel').value
        self.cmd_vel_axis = str(self.declare_parameter('cmd_vel_axis', 'x').value).lower()
        self.cmd_vel_scale = float(self.declare_parameter('cmd_vel_scale', 1.0).value)
        self.feedback_topic = self.declare_parameter('feedback_topic', 'linear_velocity').value
        self.enable_steer_cmd = bool(self.declare_parameter('enable_steer_cmd', True).value)
        self.steer_topic = self.declare_parameter('steer_topic', 'steer_target_rate_deg_s').value
        self.steer_rate_scale_deg_per_rad_s = float(
            self.declare_parameter('steer_rate_scale_deg_per_rad_s', 30.0).value
        )

        self.kp = self.declare_parameter('kp', 85.0).value
        self.ki = self.declare_parameter('ki', 20.0).value
        self.kd = self.declare_parameter('kd', 2.0).value
        self.i_max = self.declare_parameter('i_max', 50.0).value
        self.control_hz = self.declare_parameter('control_hz', 50.0).value
        self.max_pwm_percent = self.declare_parameter('max_pwm_percent', 100.0).value
        self.filter_alpha = self.declare_parameter('filter_alpha', 0.05).value
        self.deadband = self.declare_parameter('deadband', 0.03).value
        self.max_pwm_step = self.declare_parameter('max_pwm_step', 1.5).value

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
        if self.cmd_vel_axis not in ('x', 'y', 'z'):
            raise RuntimeError("cmd_vel_axis must be one of: 'x', 'y', 'z'")

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

        period = 1.0 / self.control_hz
        self.timer = self.create_timer(period, self.control_step)

    @staticmethod
    def percent_to_duty(percent: int) -> int:
        percent = max(0, min(100, percent))
        return int(percent * 10000)

    def on_cmd_vel(self, msg: Twist):
        axis_value = {
            'x': float(msg.linear.x),
            'y': float(msg.linear.y),
            'z': float(msg.linear.z),
        }[self.cmd_vel_axis]
        target_speed = axis_value * self.cmd_vel_scale
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
        self.drive_pid.update_feedback(msg.data)

    def control_step(self):
        now = self.get_clock().now()
        drive_result = self.drive_pid.step(now)
        if drive_result is not None:
            output, error, filtered = drive_result
            dir_level = 0 if output >= 0.0 else 1
            if self.invert_dir:
                dir_level = 1 - dir_level
            self.pi.write(self.dir_gpio, dir_level)

            percent = int(round(abs(output)))
            duty = self.percent_to_duty(percent)
            self.pi.hardware_PWM(self.pwm_gpio, int(self.pwm_freq), duty)

            self.get_logger().info(
                f'drive fb={filtered:.3f} err={error:.3f} out%={output:.1f} duty={duty}'
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
