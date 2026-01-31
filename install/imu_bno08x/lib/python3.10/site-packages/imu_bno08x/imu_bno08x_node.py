import time

import board
import busio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField

from adafruit_bno08x.i2c import BNO08X_I2C
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
)


class Bno08xNode(Node):
    def __init__(self):
        super().__init__('imu_bno08x_node')

        self.i2c_addr = self.declare_parameter('i2c_address', 0x4B).value
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').value
        self.imu_topic = self.declare_parameter('imu_topic', 'imu/data').value
        self.mag_topic = self.declare_parameter('mag_topic', 'imu/mag').value
        self.rate_hz = self.declare_parameter('rate_hz', 50.0).value
        self.mag_rate_hz = self.declare_parameter('mag_rate_hz', 20.0).value
        self.use_reset = self.declare_parameter('use_reset', True).value

        if self.rate_hz <= 0.0:
            raise RuntimeError('rate_hz must be > 0')
        if self.mag_rate_hz <= 0.0:
            raise RuntimeError('mag_rate_hz must be > 0')

        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(i2c, address=int(self.i2c_addr))

        if self.use_reset:
            try:
                self.bno.soft_reset()
                time.sleep(0.5)
            except Exception:
                self.get_logger().warn('BNO08x soft reset failed')

        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, int(1e6 / self.rate_hz))
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER, int(1e6 / self.rate_hz))
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE, int(1e6 / self.rate_hz))
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER, int(1e6 / self.mag_rate_hz))

        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.mag_pub = self.create_publisher(MagneticField, self.mag_topic, 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.poll)

    def poll(self):
        quat = self.bno.quaternion
        accel = self.bno.acceleration
        gyro = self.bno.gyro
        mag = self.bno.magnetic

        if not (quat and accel and gyro):
            return

        now = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.stamp = now
        imu_msg.header.frame_id = self.frame_id
        imu_msg.orientation.x = float(quat[0])
        imu_msg.orientation.y = float(quat[1])
        imu_msg.orientation.z = float(quat[2])
        imu_msg.orientation.w = float(quat[3])

        imu_msg.angular_velocity.x = float(gyro[0])
        imu_msg.angular_velocity.y = float(gyro[1])
        imu_msg.angular_velocity.z = float(gyro[2])

        imu_msg.linear_acceleration.x = float(accel[0])
        imu_msg.linear_acceleration.y = float(accel[1])
        imu_msg.linear_acceleration.z = float(accel[2])

        imu_msg.orientation_covariance[0] = -1.0
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)

        if mag:
            mag_msg = MagneticField()
            mag_msg.header.stamp = now
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field.x = float(mag[0])
            mag_msg.magnetic_field.y = float(mag[1])
            mag_msg.magnetic_field.z = float(mag[2])
            mag_msg.magnetic_field_covariance[0] = -1.0
            self.mag_pub.publish(mag_msg)


def main():
    rclpy.init()
    node = Bno08xNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
