import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class BB9Follower(Node):
    def __init__(self):
        super().__init__("bb9_follower")
        self.image_subscription = self.create_subscription(
            Image, "/robot/camera/image_raw", self.image_callback, 10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, "/robot/cmd_vel", 10)

        self.focal_length = 265.23
        self.center_x = 160.5

    def image_callback(self, msg):
        frame = CvBridge().imgmsg_to_cv2(msg)
        grayFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, bwFrame = cv2.threshold(grayFrame, 127, 255, cv2.THRESH_BINARY_INV)
        moments = cv2.moments(bwFrame)

        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])

            angular_velocity = float(
                np.arctan((self.center_x - cx) / self.focal_length)
            )

            cmd_vel_msg = Twist()
            cmd_vel_msg.angular.z = angular_velocity
            cmd_vel_msg.linear.x = 0.0
            cmd_vel_msg.linear.y = 0.0
            cmd_vel_msg.linear.z = 0.0

            self.cmd_vel_publisher.publish(cmd_vel_msg)


def main(args=None):
    rclpy.init(args=args)
    bb9_follower = BB9Follower()
    rclpy.spin(bb9_follower)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
