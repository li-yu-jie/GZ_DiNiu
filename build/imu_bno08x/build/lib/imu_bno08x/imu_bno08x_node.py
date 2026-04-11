"""
BNO08x IMU 节点：通过 I2C 读取姿态/加速度/陀螺/磁力，并发布标准 ROS2 话题。
"""
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

        # ---------------- 参数区 ----------------
        # i2c_address: 设备 I2C 地址，常见为 0x4A 或 0x4B。
        # frame_id: 发布消息时采用的 TF 坐标系名称。
        self.i2c_addr = self.declare_parameter('i2c_address', 0x4B).value
        self.frame_id = self.declare_parameter('frame_id', 'imu_link').value
        # imu_topic / mag_topic: 分别用于 IMU 与磁力计输出。
        self.imu_topic = self.declare_parameter('imu_topic', 'imu/data').value
        self.mag_topic = self.declare_parameter('mag_topic', 'imu/mag').value
        # rate_hz: 主轮询频率。
        # mag_rate_hz: 磁力计输出频率，可独立于 IMU 主频。
        self.rate_hz = self.declare_parameter('rate_hz', 50.0).value
        self.mag_rate_hz = self.declare_parameter('mag_rate_hz', 20.0).value
        # use_reset: 启动时是否尝试做软复位，用于清理异常内部状态。
        self.use_reset = self.declare_parameter('use_reset', True).value

        if self.rate_hz <= 0.0:
            raise RuntimeError('rate_hz must be > 0')
        if self.mag_rate_hz <= 0.0:
            raise RuntimeError('mag_rate_hz must be > 0')

        # 初始化 I2C 总线与 BNO08x 设备对象。
        i2c = busio.I2C(board.SCL, board.SDA)
        self.bno = BNO08X_I2C(i2c, address=int(self.i2c_addr))

        if self.use_reset:
            try:
                # 软复位以清理异常状态
                self.bno.soft_reset()
                time.sleep(0.5)
            except Exception:
                self.get_logger().warn('BNO08x soft reset failed')

        # 启用各类 report。
        # Adafruit 接口要求的周期单位是“微秒”，因此这里将频率换算成周期。
        self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR, int(1e6 / self.rate_hz))
        self.bno.enable_feature(BNO_REPORT_ACCELEROMETER, int(1e6 / self.rate_hz))
        self.bno.enable_feature(BNO_REPORT_GYROSCOPE, int(1e6 / self.rate_hz))
        self.bno.enable_feature(BNO_REPORT_MAGNETOMETER, int(1e6 / self.mag_rate_hz))

        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)
        self.mag_pub = self.create_publisher(MagneticField, self.mag_topic, 10)

        self.timer = self.create_timer(1.0 / self.rate_hz, self.poll)

    def poll(self):
        # 轮询读取传感器数据并发布 IMU 与磁力计消息
        # 常见返回形式：
        # - quaternion: (x, y, z, w)
        # - acceleration: (ax, ay, az)
        # - gyro: (gx, gy, gz)
        # - magnetic: (mx, my, mz)
        quat = self.bno.quaternion
        accel = self.bno.acceleration
        gyro = self.bno.gyro
        mag = self.bno.magnetic

        # IMU 主消息至少需要姿态、角速度、线加速度三类信息。
        # 如果其中任一类缺失，就跳过本周期，避免下游拿到半残缺数据。
        if not (quat and accel and gyro):
            return

        now = self.get_clock().now().to_msg()

        # ---------------- 组装 IMU 消息 ----------------
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

        # 当前节点未提供标定后的协方差，按 ROS 约定写 -1 表示“未知”。
        imu_msg.orientation_covariance[0] = -1.0
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)

        if mag:
            # ---------------- 组装磁力计消息 ----------------
            mag_msg = MagneticField()
            mag_msg.header.stamp = now
            mag_msg.header.frame_id = self.frame_id
            mag_msg.magnetic_field.x = float(mag[0])
            mag_msg.magnetic_field.y = float(mag[1])
            mag_msg.magnetic_field.z = float(mag[2])
            # 同样使用 -1 标记“协方差未知”。
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
