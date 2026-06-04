"""将里程计消息桥接为 TF 变换。

某些底盘或下位机只发布 nav_msgs/Odometry，不直接发布 odom -> base_link TF。
Nav2 和 TF 树需要该变换，因此这里做一个轻量桥接节点。
"""

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class OdomTfBridge(Node):
    """监听 Odometry 并广播对应的 TF。"""

    def __init__(self) -> None:
        super().__init__("odom_tf_bridge")

        # odom_topic:
        #   底盘或融合定位输出的里程计话题。
        # parent_frame:
        #   TF 父坐标系，通常为 odom。
        # child_frame:
        #   TF 子坐标系，通常为 base_link。
        self._odom_topic = self.declare_parameter("odom_topic", "/odom").value
        self._parent_frame = self.declare_parameter("parent_frame", "odom").value
        self._child_frame = self.declare_parameter("child_frame", "base_link").value

        self._broadcaster = TransformBroadcaster(self)
        self._subscription = self.create_subscription(
            Odometry, self._odom_topic, self._odom_callback, 20
        )

    def _odom_callback(self, msg: Odometry) -> None:
        """将里程计位姿直接映射成 TF。"""
        transform = TransformStamped()
        transform.header = msg.header

        # 某些底盘上报的里程计时间戳可能为 0，此时使用当前 ROS 时间。
        if transform.header.stamp.sec == 0 and transform.header.stamp.nanosec == 0:
            transform.header.stamp = self.get_clock().now().to_msg()

        transform.header.frame_id = msg.header.frame_id or self._parent_frame
        transform.child_frame_id = msg.child_frame_id or self._child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTfBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
