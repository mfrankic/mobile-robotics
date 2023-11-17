import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult


class PeopleDetection(Node):
    def __init__(self):
        super().__init__('people_detection')

        # Parameter declaration
        self.declare_parameter('stride', 8)
        self.declare_parameter('scale', 1.5)

        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_detected_people')

        # Defining the callback for parameter change
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Outputing the input and output topic
        self.get_logger().info(f'Input topic: {self.get_parameter("input_topic").value}')
        self.get_logger().info(f'Output topic: {self.get_parameter("output_topic").value}')

        # Initializing the subscriber for input raw image
        self.image_subscription = self.create_subscription(Image, self.get_parameter('input_topic').value, self.image_callback, 10)

        # Initializing the publisher for output image with detected people
        self.image_publisher = self.create_publisher(Image, self.get_parameter('output_topic').value, 10)

        # Initialize the HOG descriptor (person detector)
        self.hog_detector = cv2.HOGDescriptor()
        self.hog_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def parameter_callback(self, parameter_list):
        # Going through the list of changed parameters
        for p in parameter_list:
            self.get_logger().info(f'Set {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        # Convert the Image message to the OpenCV image frame
        frame = CvBridge().imgmsg_to_cv2(msg)

        # Convert to grayscale for faster detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Detect people in the image - returns the bounding boxes for the detected objects
        stride = int(self.get_parameter('stride').value)
        scale = float(self.get_parameter('scale').value)
        boxes, _ = self.hog_detector.detectMultiScale(gray, winStride=(stride, stride), scale=scale)

        # Display the detected boxes in the colour picture
        for (x, y, w, h) in boxes:
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Convert the OpenCV image frame with detected boxes to Image message
        image_msg = CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")

        # Copy the same message header - including the frame number and timestamp
        image_msg.header = msg.header

        # Publish the image
        self.image_publisher.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)

    people_detection = PeopleDetection()
    rclpy.spin(people_detection)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
