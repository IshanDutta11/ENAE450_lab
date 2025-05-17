import rclpy
from rclpy.node import Node
import time
import numpy as np

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Distance Front: self.distances[0]
# Distance Left: self.distances[90]
# Distance Behind: self.distances[180]
# Distance Right: self.distances[270]


class SimulationNav(Node):

    def __init__(self):
        super().__init__('simulation_nav')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.retreive_distances, 10)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.rate = 10 #hz
        timer_period = 1.0/self.rate  # seconds
        self.timer = self.create_timer(timer_period, self.navigate_maze)

        self.stopping_distance = 0.2
        self.distances=[]
        self.iterator = 0
        self.forward_threshold = 25
        self.side_threshold = 40
        # 0.5 for 0 and 2
        self.min_forward_distance = 0.4
        self.max_side_distance = 0.25
        self.complete = False

    def navigate_maze(self):
        msg = Twist()

        # Check if the lidar data is populating and if the task isn't complete
        if(len(self.distances) != 0 and self.complete == False):
            
            # These represent the average of a range of left/right lidar data. Using one point 
            # Can be inaccurate (oscillations tended to occur when a corner came along for instance).
            # Averaging these values makes it much easier to compare the distance from the left/right walls
            average_left = 0
            average_right = 0

            for i in range(-self.side_threshold, self.side_threshold+1, 1):
                average_left = average_left + self.distances[90+i]
                average_right = average_right + self.distances[270+i]

            average_left=average_left/(2*self.side_threshold + 1)
            average_right=average_right/(2*self.side_threshold + 1)

            # This is the range of forward values to look for (like your FOV). Again, one value can cause issues.
            # The bot never does a perfect turn, which means that you can't rely on just 1 value since the corners
            # are now vulnerable for collision, thus why we use an FOV. Too high and it adjusts too much, too low 
            # and a corner may hit the wall. Found that threshold of 25, or a range of about 51 degrees, is pretty 
            # good in a confined space.

            forward_quad = []

            for i in range(-self.forward_threshold, self.forward_threshold+1, 1):
                forward_quad.append(self.distances[0+i])

            # self.get_logger().info(f"Left: {average_left}")
            # self.get_logger().info(f"Right: {average_right}")

            # Ensure that all values of FOV are greater than some minimum forward distance.
            # Also, check if right and left walls are a bit too close while going forward to help avoid
            # hitting corner of bot.
            if(all(x > self.min_forward_distance for x in forward_quad) and average_right > self.max_side_distance and average_left > self.max_side_distance):
                msg.linear.x = 0.2
                msg.angular.z = 0.0
            else:
                # Left wall is further, rotate counter clockwise to turn towards it
                if(average_right < average_left):
                    msg.linear.x = 0.0
                    msg.angular.z = (np.pi/2)
                # Vice versa
                else:
                    msg.linear.x = 0.0
                    msg.angular.z = -(np.pi/2)

            # End condition. Infinte distance is assumed to be outside of maze. Get a certain distance from maze, and spin.
            if(average_left == float('inf') and average_right == float('inf') and self.distances[180] > 1): 
                msg.linear.x = 0.0
                msg.angular.z = np.pi
                self.publisher_.publish(msg)
                time.sleep(1.25)
                self.complete = True
                
        # After end condition, do nothing.
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.publisher_.publish(msg)


    def retreive_distances(self, msg):
        self.distances = msg.ranges

def main(args=None):
    rclpy.init(args=args)
    simulation_nav = SimulationNav()
    rclpy.spin(simulation_nav)
    simulation_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()