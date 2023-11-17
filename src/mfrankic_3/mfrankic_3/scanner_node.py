import math
import numpy as np
import transforms3d as t3d

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_srvs.srv import Trigger

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker


class LaserScanNode(Node):
    MAX_ANGULAR_VELOCITY = math.radians(45)
    MAX_ANGLE_DIFF = math.radians(45)
    MIN_DISTANCE_TO_GOAL = 0.3
    SLOW_APPROACH_DISTANCE = 0.5
    SLOW_LINEAR_VELOCITY = 0.1
    NORMAL_LINEAR_VELOCITY = 0.5

    def __init__(self):
        super().__init__("laser_scan_node")
        self.subscription = self.create_subscription(
            LaserScan, "/scan", self.scan_callback, 10
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_publisher = self.create_publisher(
            Marker, "/visualization_marker", 10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_service(Trigger, "/goto_closest", self.goto_closest_callback)

        self.goal_set = False
        self.drive = False
        self.distance_to_goal = 0.0
        self.angle_to_goal = 0.0

    def scan_callback(self, msg):
        self.process_scan(msg)

        if self.goal_set and self.drive:
            self.go_to_goal()

    def process_scan(self, msg):
        self.distance_to_goal = min(msg.ranges)

        if self.distance_to_goal > msg.range_max:
            self.goal_set = False
            return

        dist_index = np.argmin(msg.ranges)
        self.angle_to_goal = msg.angle_min + dist_index * msg.angle_increment
        self.angle_to_goal = self.normalize_angle(self.angle_to_goal)

        try:
            transform = self.tf_buffer.lookup_transform(
                "odom", "base_scan", rclpy.time.Time()
            )
        except TransformException as e:
            self.get_logger().error(f"Could not transform odom to base_scan: {e}")
            return

        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        rY, ra, rb = t3d.euler.quat2euler([qx, qy, qz, qw])
        rY += self.angle_to_goal

        dx = self.distance_to_goal * np.cos(rY)
        dy = self.distance_to_goal * np.sin(rY)

        mx = dx + transform.transform.translation.x
        my = dy + transform.transform.translation.y

        self.publish_marker(mx, my)
        self.goal_set = True

    def normalize_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi

        while angle < -np.pi:
            angle += 2 * np.pi

        return angle

    def publish_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.id = 0
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.55
        marker.color.g = 0.0
        marker.color.b = 0.55

        self.marker_publisher.publish(marker)

    def go_to_goal(self):
        angular_velocity = max(
            min(self.angle_to_goal, self.MAX_ANGULAR_VELOCITY),
            -self.MAX_ANGULAR_VELOCITY,
        )
        if self.distance_to_goal < self.MIN_DISTANCE_TO_GOAL:
            linear_velocity = 0.0
            angular_velocity = 0.0
            self.drive = False
        elif self.distance_to_goal < self.SLOW_APPROACH_DISTANCE:
            linear_velocity = self.SLOW_LINEAR_VELOCITY
        else:
            linear_velocity = self.NORMAL_LINEAR_VELOCITY

        if abs(self.angle_to_goal) > self.MAX_ANGLE_DIFF:
            linear_velocity = 0.0

        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(twist)

    def goto_closest_callback(self, request, response):
        if not self.goal_set:
            response.message = "No goal set"
            response.success = False
            return response

        self.drive = True
        response.message = "Going to goal"
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
