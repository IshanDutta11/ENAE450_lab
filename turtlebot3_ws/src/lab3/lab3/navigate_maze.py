import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class NavigateMaze(Node):

    def __init__(self):
        super().__init__('navigate_maze')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.retreive_distances, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.move_command)

        self.stopping_distance = 0.3
        self.max = 0.2
        self.distances=[]

    def move_command(self):

        # Length of list is 720
        # index: axes
        # 0: -y (-pi)
        # 180: -x (-pi/2)
        # 360: +y (0)
        # 540: +x (pi/2)
        # 719: -y  (pi)

        # +y
        # ^
        # |
        # |
        #  - - - > +x

        msg = Twist()

        if(len(self.distances) < 0):
            print("Distance not detected")
            msg.linear.x = 0.0
        else:
            if(self.distances[360] > self.stopping_distance):
                msg.linear.z = 1.6
                print("Distance not detected")
            else:
                msg.linear.x = -self.max


            # if(self.distances[360] > self.stopping_distance):
            #     msg.linear.x = 0.3
            #     print("move")
            # else:
            #     msg.linear.x = 0.0
            #     print("Dont move")
            #     if(self.distances[180] > self.distances[540]):
            #         msg.linear.z = 1.6
            #         self.publisher_.publish(msg)
            #         print("Turn right")

            #     else:
            #         msg.linear.z = -1.6
            #         self.publisher_.publish(msg)
            #         print("Turn left")
                
            #     time.sleep(1)
            #     msg.linear.z = 0.0
            #     print("stop_turning")
                
        self.publisher_.publish(msg)

    def retreive_distances(self, msg):
        self.distances = msg.ranges

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