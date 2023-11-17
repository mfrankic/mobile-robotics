import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from std_srvs.srv import Empty


class FeatureMatching(Node):
    def __init__(self):
        super().__init__('feature_matching')

        # Parameter declaration
        self.declare_parameter('max_features', 500)

        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_detected_keypoints')

        # Defining the callback for parameter change
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Outputing the input and output topic
        self.get_logger().info(f'Input topic: {self.get_parameter("input_topic").value}')
        self.get_logger().info(f'Output topic: {self.get_parameter("output_topic").value}')

        # Initializing the subscriber for input raw image
        self.image_subscription = self.create_subscription(Image, self.get_parameter('input_topic').value, self.image_callback, 10)

        # Initializing the publisher for output image with detected keypoints
        self.image_publisher = self.create_publisher(Image, self.get_parameter('output_topic').value, 10)

        self.matched_publisher = self.create_publisher(Image, 'match', 10)

        self.create_service(Empty, 'save_image', self.save_image_callback)
        self.create_service(Empty, 'match_image', self.match_image_callback)

        self.saved_image = None
        self.flag_save_image = False

        self.matched_image = None
        self.flag_match_image = False

        self.orb = cv2.ORB_create(nfeatures=int(self.get_parameter('max_features').value))

    def parameter_callback(self, parameter_list):
        # Going through the list of changed parameters
        for p in parameter_list:
            if p.name == 'max_features':
                self.orb.setMaxFeatures(int(p.value))
            self.get_logger().info(f'Set {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        # Convert the Image message to the OpenCV image frame
        frame = CvBridge().imgmsg_to_cv2(msg)

        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        keypoints = self.orb.detect(frame)

        if self.flag_save_image:
            self.saved_image = frame
            self.flag_save_image = False
            self.get_logger().warning(f'Image with {len(keypoints)} keypoints saved')

        if self.flag_match_image:
            self.matched_image = frame
            self.flag_match_image = False
            self.get_logger().warning(f'Trying to match image with {len(keypoints)} keypoints')

            if self.saved_image is not None:
                kp1, des1 = self.orb.detectAndCompute(self.saved_image, None)
                kp2, des2 = self.orb.compute(frame, keypoints)

                bf = cv2.BFMatcher()
                matches = bf.knnMatch(des1, des2, k=2)

                # Apply ratio test
                good = []
                for m, n in matches:
                    if m.distance < 0.75 * n.distance:
                        good.append([m])

                self.get_logger().info(f'Found {len(good)} good matches')

                # cv2.drawMatchesKnn expects list of lists as matches.
                self.matched_image = cv2.drawMatchesKnn(self.saved_image, kp1, frame, kp2, good, None)

                # Convert the OpenCV image frame with detected boxes to Image message
                image_msg = CvBridge().cv2_to_imgmsg(self.matched_image, encoding="bgr8")

                # Copy the same message header - including the frame number and timestamp
                image_msg.header = msg.header

                # Publish the image
                self.matched_publisher.publish(image_msg)
            else:
                self.get_logger().error(f'No previous image saved')


        frame = cv2.drawKeypoints(frame, keypoints, None, color=(0, 255, 0))

        # Convert the OpenCV image frame with detected boxes to Image message
        image_msg = CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")

        # Copy the same message header - including the frame number and timestamp
        image_msg.header = msg.header

        # Publish the image
        self.image_publisher.publish(image_msg)

    def save_image_callback(self, request, response):
        self.get_logger().info(f'Service /save_image called')
        self.flag_save_image = True
        return response

    def match_image_callback(self, request, response):
        self.get_logger().info(f'Service /match_image called')
        self.flag_match_image = True
        return response



def main(args=None):
    rclpy.init(args=args)

    feature_matching = FeatureMatching()
    rclpy.spin(feature_matching)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
