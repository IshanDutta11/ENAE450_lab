import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist


class MoveNode(Node):

    def __init__(self):
        super().__init__('move_node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.move_command)

    def move_command(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.0
        self.publisher_.publish(msg)

        time.sleep(.9)

        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    move_node = MoveNode()

    rclpy.spin_once(move_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    move_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()