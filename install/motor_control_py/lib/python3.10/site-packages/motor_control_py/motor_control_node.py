import pigpio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        self.pwm_gpio = self.declare_parameter('pwm_gpio', 18).value
        self.dir_gpio = self.declare_parameter('dir_gpio', 26).value
        self.pwm_freq = self.declare_parameter('pwm_freq_hz', 100000).value
        self.invert_dir = self.declare_parameter('invert_dir', True).value
        self.cmd_topic = self.declare_parameter('cmd_topic', 'target_speed').value
        self.feedback_topic = self.declare_parameter('feedback_topic', 'linear_velocity').value

        self.kp = self.declare_parameter('kp', 200.0).value
        self.ki = self.declare_parameter('ki', 50.0).value
        self.kd = self.declare_parameter('kd', 10.0).value
        self.i_max = self.declare_parameter('i_max', 50.0).value
        self.control_hz = self.declare_parameter('control_hz', 50.0).value
        self.max_pwm_percent = self.declare_parameter('max_pwm_percent', 100.0).value
        self.filter_alpha = self.declare_parameter('filter_alpha', 0.1).value
        self.deadband = self.declare_parameter('deadband', 0.02).value
        self.max_pwm_step = self.declare_parameter('max_pwm_step', 5.0).value

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

        self.pi = pigpio.pi()
        if not self.pi.connected:
            raise RuntimeError('pigpio daemon not running. Run: sudo pigpiod')

        self.pi.set_mode(self.dir_gpio, pigpio.OUTPUT)
        self.pi.write(self.dir_gpio, 0)

        self.target_speed = 0.0
        self.measured_speed = None
        self.filtered_speed = None
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_output = 0.0
        self.prev_filtered = None
        self.last_time = None

        self.sub_cmd = self.create_subscription(Float64, self.cmd_topic, self.on_cmd, 10)
        self.sub_fb = self.create_subscription(Float64, self.feedback_topic, self.on_feedback, 10)

        period = 1.0 / self.control_hz
        self.timer = self.create_timer(period, self.control_step)

    @staticmethod
    def percent_to_duty(percent: int) -> int:
        percent = max(0, min(100, percent))
        return int(percent * 10000)

    def on_cmd(self, msg: Float64):
        self.target_speed = msg.data
        self.get_logger().info(f'target_speed={self.target_speed:.3f} m/s')

    def on_feedback(self, msg: Float64):
        self.measured_speed = msg.data
        if self.filtered_speed is None:
            self.filtered_speed = self.measured_speed
        else:
            a = self.filter_alpha
            self.filtered_speed = (a * self.measured_speed) + ((1.0 - a) * self.filtered_speed)

    def control_step(self):
        if self.measured_speed is None:
            return
        if self.filtered_speed is None:
            return

        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            self.prev_filtered = self.filtered_speed
            return

        dt = (now - self.last_time).nanoseconds / 1e9
        if dt <= 0.0:
            return

        error = self.target_speed - self.filtered_speed
        if abs(error) < self.deadband:
            error = 0.0

        derivative = 0.0
        if self.prev_filtered is not None:
            derivative = -(self.filtered_speed - self.prev_filtered) / dt

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
        self.prev_error = error
        self.prev_filtered = self.filtered_speed
        self.last_time = now

        dir_level = 0 if output >= 0.0 else 1
        if self.invert_dir:
            dir_level = 1 - dir_level
        self.pi.write(self.dir_gpio, dir_level)

        percent = int(round(abs(output)))
        duty = self.percent_to_duty(percent)
        self.pi.hardware_PWM(self.pwm_gpio, int(self.pwm_freq), duty)

        self.get_logger().info(
            f'fb={self.filtered_speed:.3f} err={error:.3f} out%={output:.1f} duty={duty}'
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
