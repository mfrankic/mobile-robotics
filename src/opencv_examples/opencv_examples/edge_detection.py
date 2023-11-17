import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult


class EdgeDetection(Node):
    def __init__(self):
        super().__init__('edge_detection')

        # Parameter declaration
        self.declare_parameter('thr1', 50)
        self.declare_parameter('thr2', 150)

        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_edges')

        # Defining the callback for parameter change
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Defining the callback for parameter change
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Outputing the input and output topic
        self.get_logger().info(f'Input topic: {self.get_parameter("input_topic").value}')
        self.get_logger().info(f'Output topic: {self.get_parameter("output_topic").value}')

        # Initializing the subscriber for input raw image
        self.image_subscription = self.create_subscription(Image, self.get_parameter('input_topic').value, self.image_callback, 10)

        # Initializing the publisher for output image with edges
        self.image_publisher = self.create_publisher(Image, self.get_parameter('output_topic').value, 10)

    def parameter_callback(self, parameter_list):
        # Going through the list of changed parameters
        for p in parameter_list:
            self.get_logger().info(f'Set {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        # Convert the Image message to the OpenCV image frame
        frame = CvBridge().imgmsg_to_cv2(msg)

        # Get Canny edge detector threshold values from parameteres
        thr1 = int(self.get_parameter('thr1').value)
        thr2 = int(self.get_parameter('thr2').value)

        # Detect the edges using the Canny edge detector
        frame_edges = cv2.Canny(frame, thr1, thr2)

        # Convert the thresholded OpenCV image frame to Image message
        image_msg = CvBridge().cv2_to_imgmsg(frame_edges, encoding="mono8")

        # Copy the same message header - including the frame number and timestamp
        image_msg.header = msg.header

        # Publish the thresholded image
        self.image_publisher.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)

    edge_detection = EdgeDetection()
    rclpy.spin(edge_detection)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
