import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult


class ColorThresholding(Node):
    def __init__(self):
        super().__init__('color_thresholding')

        # Parameter declaration
        self.declare_parameter('r_min', 0)
        self.declare_parameter('g_min', 0)
        self.declare_parameter('b_min', 0)

        self.declare_parameter('r_max', 255)
        self.declare_parameter('g_max', 255)
        self.declare_parameter('b_max', 255)

        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_threshold')

        # Defining the callback for parameter change
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Outputing the input and output topic
        self.get_logger().info(f'Input topic: {self.get_parameter("input_topic").value}')
        self.get_logger().info(f'Output topic: {self.get_parameter("output_topic").value}')

        # Initializing the subscriber for input raw image
        self.image_subscription = self.create_subscription(Image, self.get_parameter('input_topic').value, self.image_callback, 10)

        # Initializing the publisher for output thresholded image
        self.image_publisher = self.create_publisher(Image, self.get_parameter('output_topic').value, 10)

    def parameter_callback(self, parameter_list):
        # Going through the list of changed parameters
        for p in parameter_list:
            self.get_logger().info(f'Set {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        # Convert the Image message to the OpenCV image frame
        frame = CvBridge().imgmsg_to_cv2(msg)

        # Get color threshold values from parameteres
        r_min = int(self.get_parameter('r_min').value)
        g_min = int(self.get_parameter('g_min').value)
        b_min = int(self.get_parameter('b_min').value)
        r_max = int(self.get_parameter('r_max').value)
        g_max = int(self.get_parameter('g_max').value)
        b_max = int(self.get_parameter('b_max').value)

        # Threshold the image - image is written in RGB format
        frame_threshold = cv2.inRange(frame, (r_min, g_min, b_min), (r_max, g_max, b_max))

        # Convert the thresholded OpenCV image frame to Image message
        image_msg = CvBridge().cv2_to_imgmsg(frame_threshold, encoding="mono8")

        # Copy the same message header - including the frame number and timestamp
        image_msg.header = msg.header

        # Publish the thresholded image
        self.image_publisher.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)

    color_thresholding = ColorThresholding()
    rclpy.spin(color_thresholding)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
