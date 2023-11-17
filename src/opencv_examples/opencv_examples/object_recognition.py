import rclpy
from rclpy.node import Node

import os
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult


class ObjectRecognition(Node):
    def __init__(self):
        super().__init__('object_recognition')

        # Parameter declaration
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('threshold', 0.3)

        self.declare_parameter('weights_file', '')
        self.declare_parameter('cfg_file', '')
        self.declare_parameter('classes_file', '')

        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/camera/image_detected_objects')

        # Defining the callback for parameter change
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Outputing the input and output topic
        self.get_logger().info(f'Input topic: {self.get_parameter("input_topic").value}')
        self.get_logger().info(f'Output topic: {self.get_parameter("output_topic").value}')

        # Outputing weights file path
        self.get_logger().info(f'Weights file: {os.path.expanduser(self.get_parameter("weights_file").value)}')
        if not os.path.isfile(os.path.expanduser(self.get_parameter('weights_file').value)):
            self.get_logger().error(f'Weights file path is not correct')
            exit()

        # Outputing config file path
        self.get_logger().info(f'Config file: {os.path.expanduser(self.get_parameter("cfg_file").value)}')
        if not os.path.isfile(os.path.expanduser(self.get_parameter('cfg_file').value)):
            self.get_logger().error(f'Config file path is not correct')
            exit()

        # Outputing classes file path
        self.get_logger().info(f'Classes file: {os.path.expanduser(self.get_parameter("classes_file").value)}')
        if not os.path.isfile(os.path.expanduser(self.get_parameter('classes_file').value)):
            self.get_logger().error(f'Classes file path is not correct')
            exit()

        # Initializing the subscriber for input raw image
        self.image_subscription = self.create_subscription(Image, self.get_parameter('input_topic').value, self.image_callback, 10)

        # Initializing the publisher for output image with detected people
        self.image_publisher = self.create_publisher(Image, self.get_parameter('output_topic').value, 10)

        # Loading class labels
        self.classes = open(os.path.expanduser(self.get_parameter("classes_file").value)).read().strip().split("\n")
        self.colors = np.random.uniform(0, 255, size=(len(self.classes), 3))

        # Initialize the CNN darknet detector
        self.net_detector = cv2.dnn.readNetFromDarknet(os.path.expanduser(self.get_parameter('cfg_file').value), os.path.expanduser(self.get_parameter('weights_file').value))

    def parameter_callback(self, parameter_list):
        # Going through the list of changed parameters
        for p in parameter_list:
            self.get_logger().info(f'Set {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    def image_callback(self, msg):
        # Convert the Image message to the OpenCV image frame
        frame = CvBridge().imgmsg_to_cv2(msg)
        (h, w) = frame.shape[:2]

        # Determine only the output layer names from YOLO
        layer_names = self.net_detector.getLayerNames()
        layer_names = [layer_names[i - 1] for i in self.net_detector.getUnconnectedOutLayers()]

        # Construct a blob from the input image and then perform a forward pass of the YOLO object detector, resulting in bounding boxes and associated probabilities
        blob = cv2.dnn.blobFromImage(frame, 1 / 255.0, (416, 416), swapRB=True, crop=False)
        self.net_detector.setInput(blob)
        layerOutputs = self.net_detector.forward(layer_names)

        boxes = []
        confidences = []
        classIDs = []

        # Loop over each of the layer outputs
        for output in layerOutputs:
            # Loop over each of the detections
            for detection in output:
                # Extract the class ID and confidence (i.e., probability) of the current object detection
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]

                # Filter out weak predictions by ensuring the detected probability is greater than the minimum probability
                if confidence > float(self.get_parameter('confidence').value):
                    # Scale the bounding box coordinates back relative to the size of the image, keeping in mind that YOLO actually returns the center (x, y)-coordinates of the bounding box followed by the boxes' width and height
                    box = detection[0:4] * np.array([w, h, w, h])
                    (centerX, centerY, width, height) = box.astype("int")

                    # Use the center (x, y)-coordinates to derive the top and and left corner of the bounding box
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))

                    # Update list of bounding box coordinates, confidences, and class IDs
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        # Removing some boxes using non maximum supression
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, float(self.get_parameter('confidence').value), float(self.get_parameter('threshold').value))

        # Ensure at least one detection exists
        if len(idxs) > 0:
            for i in idxs.flatten():
                # Extract the bounding box coordinates
                (x, y, w, h) = boxes[i]

                # Draw a bounding box rectangle on the image
                color = [int(c) for c in self.colors[classIDs[i]]]
                cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)

                # Writing the class label and confidence score on the image
                text = "{}: {:.3f}".format(self.classes[classIDs[i]], confidences[i])
                cv2.putText(frame, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        # Convert the OpenCV image frame with detected boxes to Image message
        image_msg = CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")

        # Copy the same message header - including the frame number and timestamp
        image_msg.header = msg.header

        # Publish the image
        self.image_publisher.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)

    object_recognition = ObjectRecognition()
    rclpy.spin(object_recognition)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
