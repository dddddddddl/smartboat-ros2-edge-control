import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


def yaw_from_quaternion(x, y, z, w):
    """
    Convert quaternion to yaw angle in radians.
    ROS convention: yaw around Z axis.
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class YawFromImuNode(Node):
    def __init__(self):
        super().__init__('yaw_from_imu')

        self.declare_parameter('input_imu_topic', '/mavros/imu/data')
        self.declare_parameter('output_yaw_topic', '/smartboat/localization/yaw')

        self.input_imu_topic = str(
            self.get_parameter('input_imu_topic').value
        )
        self.output_yaw_topic = str(
            self.get_parameter('output_yaw_topic').value
        )

        self.last_log_time = 0.0

        self.yaw_pub = self.create_publisher(
            Float32,
            self.output_yaw_topic,
            10
        )

        self.create_subscription(
            Imu,
            self.input_imu_topic,
            self.imu_callback,
            10
        )

        self.get_logger().warning(
            f'YawFromImu started | input={self.input_imu_topic} | '
            f'output={self.output_yaw_topic}'
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def imu_callback(self, msg):
        q = msg.orientation

        yaw = yaw_from_quaternion(
            q.x,
            q.y,
            q.z,
            q.w
        )

        out = Float32()
        out.data = float(yaw)
        self.yaw_pub.publish(out)

        now = self.now_sec()
        if now - self.last_log_time > 1.0:
            self.last_log_time = now
            self.get_logger().warning(
                f'published yaw={math.degrees(yaw):.1f} deg'
            )


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = YawFromImuNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
