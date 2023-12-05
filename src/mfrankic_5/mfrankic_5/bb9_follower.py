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

        self.declare_parameter('h_min', 100)
        self.declare_parameter('s_min', 100)
        self.declare_parameter('v_min', 0)

        self.declare_parameter('h_max', 120)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_max', 150)

        self.declare_parameter('input_topic', '/robot/camera/image_raw')
        self.declare_parameter('output_topic', '/robot/camera/image_threshold')
        
        self.image_subscription = self.create_subscription(
            Image, self.get_parameter('input_topic').value, self.image_callback, 10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, "/robot/cmd_vel", 10)
        self.image_publisher = self.create_publisher(Image, self.get_parameter('output_topic').value, 10)

        self.focal_length = 265.23
        self.center_x = 160.5

    def image_callback(self, msg):
        frame = CvBridge().imgmsg_to_cv2(msg)

        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        h_min = int(self.get_parameter('h_min').value)
        s_min = int(self.get_parameter('s_min').value)
        v_min = int(self.get_parameter('v_min').value)
        h_max = int(self.get_parameter('h_max').value)
        s_max = int(self.get_parameter('s_max').value)
        v_max = int(self.get_parameter('v_max').value)
        
        frame_threshold = cv2.inRange(hsv_frame, (h_min, s_min, v_min), (h_max, s_max, v_max))
        
        # image_msg = CvBridge().cv2_to_imgmsg(frame_threshold, encoding="mono8")
        # image_msg.header = msg.header
        # self.image_publisher.publish(image_msg)
        
        moments = cv2.moments(frame_threshold)

        if moments["m00"] > 0:
            cx = int(moments["m10"] / moments["m00"])

            angular_velocity = float(
                np.arctan((self.center_x - cx) / self.focal_length)
            )

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = angular_velocity

            self.cmd_vel_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    bb9_follower = BB9Follower()
    rclpy.spin(bb9_follower)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
