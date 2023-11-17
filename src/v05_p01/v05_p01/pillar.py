import rclpy
from rclpy.node import Node

import numpy as np
import transforms3d as t3d

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from sensor_msgs.msg import LaserScan


class MinScan(Node):
    def __init__(self):
        super().__init__("minscan")

        self.create_subscription(LaserScan, "/scan", self.laser_scan_callback, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def laser_scan_callback(self, msg):
        distance = min(msg.ranges)
        dist_index = np.argmin(msg.ranges)
        yaw = msg.angle_min + dist_index * msg.angle_increment

        if distance > msg.range_max:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_scan", rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().error(f"Could not transform odom to base_scan: {e}")
            return

        while yaw > np.pi:
            yaw -= 2 * np.pi

        while yaw < -np.pi:
            yaw += 2 * np.pi

        dx = distance * np.cos(yaw)
        dy = distance * np.sin(yaw)

        mx = dx + transform.transform.translation.x
        my = dy + transform.transform.translation.y

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        rY, _, _ = t3d.euler.quat2euler([qx, qy, qz, qw])

        self.get_logger().info(
            f"dist: {distance:.2f}, yaw: {yaw:.2f}, mx: {mx:.2f}, my: {my:.2f}, rY: {rY:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)

    ms = MinScan()
    rclpy.spin(ms)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
