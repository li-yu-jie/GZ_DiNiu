"""根据转向角动态发布传感器 TF。

适用于雷达或摄像头随转向机构一起偏转的场景。节点监听 /steer_angle，
实时更新 base_link 到传感器坐标系的 yaw 角，确保感知结果与实际姿态一致。
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster


class SteeredSensorTf(Node):
    """发布随转向角变化的动态 TF。"""

    def __init__(self) -> None:
        super().__init__("steered_sensor_tf")

        # steer_topic:
        #   转向角输入话题，单位默认按弧度处理。
        # parent_frame / child_frame:
        #   动态 TF 的父子坐标系名称。
        self._steer_topic = self.declare_parameter("steer_topic", "/steer_angle").value
        self._parent_frame = self.declare_parameter("parent_frame", "base_link").value
        self._child_frame = self.declare_parameter("child_frame", "livox_frame").value

        # 传感器安装位姿：
        # x/y/z 为平移量，roll/pitch 为固定安装角，yaw 由偏置 + 转向角共同决定。
        self._x = float(self.declare_parameter("x", 0.0).value)
        self._y = float(self.declare_parameter("y", 0.0).value)
        self._z = float(self.declare_parameter("z", 0.0).value)
        self._roll = float(self.declare_parameter("roll", 0.0).value)
        self._pitch = float(self.declare_parameter("pitch", 0.0).value)
        self._yaw_offset = float(self.declare_parameter("yaw_offset", 0.0).value)

        # steer_scale:
        #   转向角映射比例。若传感器偏转角与舵轮转角不完全相同，可在此做缩放。
        # steer_in_degrees:
        #   若下位机上报的是角度制，则设置为 True 自动转成弧度。
        self._steer_scale = float(self.declare_parameter("steer_scale", 1.0).value)
        self._steer_in_degrees = bool(self.declare_parameter("steer_in_degrees", False).value)

        # publish_rate:
        #   TF 发布频率，建议与转向角更新频率相匹配。
        self._publish_rate = float(self.declare_parameter("publish_rate", 30.0).value)

        self._latest_yaw = self._yaw_offset
        self._broadcaster = TransformBroadcaster(self)
        self._subscription = self.create_subscription(
            Float64, self._steer_topic, self._steer_callback, 20
        )
        self._timer = self.create_timer(
            1.0 / max(self._publish_rate, 1.0),
            self._publish_transform,
        )

    def _steer_callback(self, msg: Float64) -> None:
        """更新当前转向角。"""
        steer = float(msg.data)
        if self._steer_in_degrees:
            steer = math.radians(steer)

        self._latest_yaw = self._yaw_offset + steer * self._steer_scale
        self._publish_transform()

    def _publish_transform(self) -> None:
        """发布最新的传感器 TF。"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self._parent_frame
        transform.child_frame_id = self._child_frame
        transform.transform.translation.x = self._x
        transform.transform.translation.y = self._y
        transform.transform.translation.z = self._z

        qx, qy, qz, qw = self._quaternion_from_euler(
            self._roll, self._pitch, self._latest_yaw
        )
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self._broadcaster.sendTransform(transform)

    @staticmethod
    def _quaternion_from_euler(
        roll: float, pitch: float, yaw: float
    ) -> tuple[float, float, float, float]:
        """欧拉角转四元数。"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return qx, qy, qz, qw


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SteeredSensorTf()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
