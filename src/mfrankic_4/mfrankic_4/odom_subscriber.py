import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSubscriber(Node):

    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.get_logger().info(f'Robot position: x: {x}, y: {y}')

def main(args=None):
    rclpy.init(args=args)

    odom_subscriber = OdomSubscriber()

    rclpy.spin(odom_subscriber)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
