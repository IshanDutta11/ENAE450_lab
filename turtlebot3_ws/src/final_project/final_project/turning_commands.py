import rclpy
from rclpy.node import Node
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class TurningCommands(Node):

    def __init__(self):
        super().__init__('navigate_maze')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.retreive_distances, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.rate = 10 #hz
        timer_period = 1.0/self.rate  # seconds
        self.timer = self.create_timer(timer_period, self.move_robot)
        self.count = 0
        self.count_delay = 3 # tuning
        self.stopping_distance = 0.2
        self.distances=[]
        self.iterator = 0

    def move_robot(self):
        self.get_logger().info(f"Callbacks: {self.count}")
        self.count += self.iterator
        actions = [0, 20 + self.count_delay, 40, 60 + self.count_delay]
        
        self.get_logger().info(f"Distance in front: {self.distances[360]}")
        self.get_logger().info(f"Distance to the right: {self.distances[540]}")
        self.get_logger().info(f"Distance behind: {self.distances[0]}")
        self.get_logger().info(f"Distance to the left: {self.distances[180]}")



        if(self.distances[360] > 0.4 or self.distances[360] == float('inf')):
            self.forward(0.2)
        elif(self.distances[360] < 0.4 and self.distances[540] > self.distances[180]):
            self.turn(0.7854)
            time.sleep(0.8)
        elif(self.distances[360] < 0.4 and self.distances[540] < self.distances[180]):
            self.turn(-0.7854)
            time.sleep(0.8)
        else:
            self.turn(0.0)
        
            # self.iterator = 1
        
            # if self.count > actions[0] and self.count < actions[1]:
            #     self.turn(-0.7854)
            # elif self.count > actions[2] and self.count < actions[3]:
            #     self.turn(0.7854)
            # else:
            #     self.turn(0.0)

    def turn(self, angle):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = angle 
        self.publisher_.publish(msg)

    def forward(self, velocity):
        msg = Twist()
        msg.linear.x = velocity
        msg.angular.z = 0.0 
        self.publisher_.publish(msg)

    def retreive_distances(self, msg):
        self.distances = msg.ranges

def main(args=None):
    rclpy.init(args=args)

    navigate_maze = TurningCommands()

    rclpy.spin(navigate_maze)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    navigate_maze.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()