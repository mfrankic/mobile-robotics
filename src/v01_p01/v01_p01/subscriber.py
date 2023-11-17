import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinSub(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.subscription = self.create_subscription(
            String,
            'razgovor',
            self.msg_callback,
            10
        )
        
    def msg_callback(self, msg: String):
        self.get_logger().error(f'I heard: {msg.data}')
        
def main(args=None):
    rclpy.init(args=args)
    
    minsub = MinSub()
    
    rclpy.spin(minsub)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
