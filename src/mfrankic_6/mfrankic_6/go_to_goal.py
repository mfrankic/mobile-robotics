import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
import math

class GoToGoal(Node):

    def __init__(self):
        super().__init__('go_to_goal')
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_position = None
        self.Kp = 1
        self.MAX_ANGULAR_VELOCITY = math.radians(45)
        self.MAX_ANGLE_DIFF = math.radians(45)
        self.MIN_DISTANCE_TO_GOAL = 0.05
        self.SLOW_APPROACH_DISTANCE = 0.3
        self.SLOW_LINEAR_VELOCITY = 0.1
        self.NORMAL_LINEAR_VELOCITY = 0.5
        
    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

    def goal_callback(self, msg):
        self.goal_position = msg.pose.position

    def odom_callback(self, msg):
        if self.goal_position is None:
            return

        current_position = msg.pose.pose.position
        dx = self.goal_position.x - current_position.x
        dy = self.goal_position.y - current_position.y
        distance_to_goal = math.sqrt(dx*dx + dy*dy)
        angle_to_goal = math.atan2(dy, dx)
        
        quat = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(quat.z*quat.w + quat.x*quat.y), 1.0 - 2.0*(quat.y*quat.y + quat.z*quat.z))
        
        angle_difference = self.normalize_angle(angle_to_goal - yaw)
        angular_velocity = self.Kp * angle_difference

        angular_velocity = max(min(angular_velocity, self.MAX_ANGULAR_VELOCITY), -self.MAX_ANGULAR_VELOCITY)
        
        if distance_to_goal < self.MIN_DISTANCE_TO_GOAL:
            linear_velocity = 0.0
            angular_velocity = 0.0
            self.goal_position = None
        elif distance_to_goal < self.SLOW_APPROACH_DISTANCE:
            linear_velocity = self.SLOW_LINEAR_VELOCITY
        else:
            linear_velocity = self.NORMAL_LINEAR_VELOCITY
            
        if abs(angle_difference) > self.MAX_ANGLE_DIFF:
            linear_velocity = 0.0
            
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        self.cmd_vel_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    odom_subscriber = GoToGoal()

    rclpy.spin(odom_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
