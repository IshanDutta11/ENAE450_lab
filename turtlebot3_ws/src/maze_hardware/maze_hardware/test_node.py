import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from collections import deque
import time

import numpy as np

#list of checkpoints in order, each checkpoint is a tuple
#keep track of current left, right, forward, back as "state"
#we move forward until we've come close enough, i.e. within some error of the initial state of the current element of the list
#make the turn specified by element 3 of the checkpoint tuple, check if we've reached the necessary state for exiting the tuple


#(3-tuple)
#element 1 is initial distances (4-tuple): (left_i, right_i, forward_i, back_i)
#element 2 is final distances (4-tuple): (left_f, right_f, forward_f, back_f)
#element 3 is angle (float): angle
#Ultimately, a checkpoint will look like: ((left_i, right_i, forward_i, back_i), (left_f, right_f, forward_f, back_f), angle)


#Information retrieval from msg.ranges where msg is of type LaserScan
# Length of list is 720
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


class CheckpointNav(Node):

    def __init__(self):
        super().__init__("checkpoint_nav")
        self.subscription = self.create_subscription(LaserScan, 'scan', self.retrieve_distances, 10)
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.state = (None, None, None, None) #left, right, front, back (540, 180, 360, 0) 
        self.checkpoint = 0 # state
        
        self.latest_ranges = [-1.0] * 720

        self.small_offset = -4 #degree variance old
        self.big_offset = 4 #degree variance old

        self.largest_i = -1
        self.smallest_i = -1 # range left old

        self.timer = self.create_timer(0.1, self.control_loop)

    # accepts linear and angular velocity, publishes them to robot
    def move_robot (self, x, a):
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = a
        self.publisher.publish(msg)
        
    # compares desired and actual headings and turns
    def turn_robot (self):
        turn_left = 1.82
        turn_right = -1.82

        deg_change = (self.largest_i - 360)/2
        angular_speed = 104
        duration = abs(deg_change)/angular_speed # how many seconds are needed to traverse the distance   

        # depending on parity of the degree change, turn left or right for the duration
        if(deg_change < 0):
            self.move_robot(0.0, turn_right)
            time.sleep(duration)
            self.move_robot(0.0, 0.0)
        elif(deg_change > 0):
            self.move_robot(0.0, turn_left)
            time.sleep(duration)
            self.move_robot(0.0, 0.0)

    def largest_i_over_3_1 (self):
        reduced = self.latest_ranges[260:541] # check northeast through west
        reduced = [-1.0 if x > 6 else x for x in reduced]
        x = 0
        while (x < len(reduced)):
            self.largest_i = 260 + x # declare the point
            if (reduced[x] >= 3.0):
                return True #end function if the point is good
            else:
                x = x +1

        return False
    
    def largest_i_over_3_2 (self):
        reduced = self.latest_ranges[440:631] # check northwest through southwest
        reduced = [-1.0 if x > 6.0 else x for x in reduced]
        x = 0
        while (x < len(reduced)):
            self.largest_i = 440 + x # declare the point
            if (reduced[x] >= 3.0):
                return True #end function if the point is good
            else:
                x = x +1

        return False

    #this is being called repetitively at a very short interval
    def control_loop(self):
        self.get_logger().info(f' Checkpoint {self.checkpoint} Desired Forward {self.largest_i}')

        match(self.checkpoint):
            case 0: #Pose: random orientation, no movement; FIll lidar ranges, get farthest (rightmost) angle, turn the robot, stop turning.
                if (self.state[0] != None):
                    if(self.largest_i_over_3_1()):
                        # self.checkpoint = self.checkpoint
                        self.checkpoint = self.checkpoint + 1
                        self.turn_robot()
            case 1: #Pose: facing farthest valid distance, no movement; Start moving forward. 
                self.move_robot(0.26, 0.0)
                self.checkpoint = self.checkpoint + 1
            case 2: #Pose: facing farthest valid distance, moving forward; Stop when top_right quadrant is close to the halfway wall.
                reduced = self.latest_ranges[270:450]
                reduced = [-1.0 if d > 6 else d for d in reduced]
                counter = 0
                for d in reduced:
                    if d < 0.33:
                        counter = counter + 1
                    if counter >=5:
                        self.move_robot(0.0, 0.0)
                        self.checkpoint = self.checkpoint + 1
                        break
            case 3: # Pose: facing wall in the way no movement; turn towards next furthest, stop turning
                self.largest_i_over_3_2()
                # self.smallest_i_over_3()
                new_forward = int(self.largest_i + 16)
                self.largest_i = new_forward
                self.turn_robot()
                self.checkpoint = self.checkpoint + 1
            case 4: #Pose: facing the second furthest valid distance, no movement; Start moving forward. 
                self.move_robot(0.26, 0.0)
            case _: #default: do nothing
                self.move_robot(0.0, 0.0)

    # accept the msg from the lidar publisher and use it accordingly, saves time to store it and mess with the stored value.
    def retrieve_distances(self, msg):
        if len(msg.ranges) == 720 :
            self.latest_ranges = msg.ranges
            self.state = (msg.ranges[540], msg.ranges[180], msg.ranges[360], msg.ranges[0]) # left, right, front, back
            # self.get_logger().info(f'Lidar Reading: Left {self.state[0]} Right {self.state[1]} Front {self.state[2]} Back {self.state[3]}')
       
            
       
def main(args=None):
    rclpy.init(args=args)
    checkpoint_node = CheckpointNav()
    
    rclpy.spin(checkpoint_node)
    checkpoint_node.destroy_node()
    
    rclpy.shutdown()
    
                
if __name__ == '__main__':
    main()