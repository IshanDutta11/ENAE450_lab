import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class NavigateMaze(Node):

    def __init__(self):
        super().__init__('navigate_maze')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.move_command)

        self.distance=[]

    def move_command(self):
        msg = Twist()

        # Length of list is 720
        # 0: -pi    719: pi  360: 0
        if(len(self.distance) > 0 and self.distance[360] > 0.2):
            msg.linear.x = 0.3
            print("move")
        else:
            msg.linear.x = 0.0
            print("dont move")

        self.publisher_.publish(msg)

    def listener_callback(self, msg):
        self.distance = msg.ranges

def main(args=None):
    rclpy.init(args=args)

    navigate_maze = NavigateMaze()

    rclpy.spin(navigate_maze)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigate_maze.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()