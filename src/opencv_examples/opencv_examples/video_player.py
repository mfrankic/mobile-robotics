import rclpy
from rclpy.node import Node

import os
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult


class VideoPlayer(Node):
    def __init__(self):
        super().__init__('video_player')

        # Parameter declaration
        self.declare_parameter('video_path', '')
        self.declare_parameter('repeat', False)
        self.declare_parameter('frame_rate', 0)
        self.declare_parameter('output_topic', '/camera/image_raw')

        # Defining the callback for parameter change
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Outputing important values
        self.get_logger().info(f'Video file: {os.path.expanduser(self.get_parameter("video_path").value)}')
        self.get_logger().info(f'Repeat: {self.get_parameter("repeat").value}')
        self.get_logger().info(f'Frame rate: {self.get_parameter("frame_rate").value}')
        self.get_logger().info(f'Output topic: {self.get_parameter("output_topic").value}')

        # Initializing the video capture for a specific file
        self.video_capture = cv2.VideoCapture(os.path.expanduser(self.get_parameter('video_path').value))

        # Check if the file is properly opened
        if not self.video_capture.isOpened():
            self.get_logger().error(f'Cannot open the video file')
            exit()

        # Get the FPS value from the parameter or the video FPS
        frame_rate = self.get_framerate(int(self.get_parameter('frame_rate').value))
        self.get_logger().info(f'Video FPS: {frame_rate}')

        # Get the number of frames in the video
        self.number_of_frames = int(self.video_capture.get(cv2.CAP_PROP_FRAME_COUNT))
        self.get_logger().info(f'Number of frames: {self.number_of_frames}')

        # Initializing a timer based callback
        self.timer = self.create_timer(1.0 / frame_rate, self.timer_callback)

        # Initializing the publisher for output image
        self.image_publisher = self.create_publisher(Image, self.get_parameter('output_topic').value, 10)

    def get_framerate(self, frame_rate):
        # If frame_rate value is not a valid number, get a video FPS
        if frame_rate <= 0:
            frame_rate = self.video_capture.get(cv2.CAP_PROP_FPS)

        return frame_rate

    def parameter_callback(self, parameter_list):
        # Going through the list of changed parameters
        for p in parameter_list:
            # If one of the changed parameters is frame_rate period of the timer should be changed accordingly
            if p.name == 'frame_rate':
                self.timer.timer_period_ns = 1000000000 // self.get_framerate(int(p.value))
            self.get_logger().info(f'Set {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    def timer_callback(self):
        if self.video_capture.get(cv2.CAP_PROP_POS_FRAMES) == self.number_of_frames:
            # If video has ended (current frame iterator equals to the number of frames in the video) check if repeat is enabled
            if bool(self.get_parameter('repeat').value):
                # If repeat is enabled, restart the video
                self.video_capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
            else:
                self.get_logger().info(f'Playback done')
                exit()

        # Fetch the next frame from the video
        ret, frame = self.video_capture.read()

        # If return value is not True, there was an error
        if not ret:
            self.get_logger().error(f'Error while reading the video file')
            return

        # Convert the fetched OpenCV image frame to Image message
        image_msg = CvBridge().cv2_to_imgmsg(frame, encoding="bgr8")

        # Setup the Image message header - including the frame number and timestamp
        image_msg.header.stamp = rclpy.time.Time().to_msg()
        image_msg.header.frame_id = str(self.video_capture.get(cv2.CAP_PROP_POS_FRAMES))

        # Publish the image
        self.image_publisher.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)

    video_player = VideoPlayer()
    rclpy.spin(video_player)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
