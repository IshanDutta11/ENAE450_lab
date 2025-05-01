import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class CardinalNode(Node):

    def __init__(self):
        super().__init__('cardinal_node')
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    cardinal_node = CardinalNode()
    rclpy.spin(cardinal_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cardinal_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()