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
        self.checkpoint = 0
        
        self.latest_ranges = [-1.0] * 720

        self.small_offset = -4
        self.big_offset = 4

        self.furthest_1_dist = -1
        self.furthest_1_dist = -1.0

        self.furthest_2 = -1
        self.furthest_2_dist = -1.0

        self.timer = self.create_timer(0.1, self.control_loop)

    def start_ranges (self):
        small = 360 + (self.small_offset)
        big = 360 + (self.big_offset)

        if (self.furthest_1 < 360 + (self.small_offset)):
            small = self.furthest_1 
            big = 360 + (self.big_offset)
        elif (self.furthest_1 > 360 + (self.big_offset)):
            small = 360 + (self.small_offset)
            big = self.furthest_1 

        reduced = self.latest_ranges[small:big]
        reduced = [-1.0 if x > 7.7 else x for x in reduced]

        self.furthest_1_dist = max(reduced)
        self.furthest_1 = small + reduced.index(self.furthest_1_dist) 
        self.get_logger().info(f' Furthest Index {self.furthest_1} Furthest Distance {self.furthest_1_dist}')


    def move_robot (self, x, a):
        msg = Twist()
        msg.linear.x = x
        msg.angular.z = a
        self.publisher.publish(msg)
        
        
    #this is being called repetitively at a very short interval
    def control_loop(self):
        self.get_logger().info(f' Checkpoint {self.checkpoint} ')
        # self.get_logger().info(f'')

        match(self.checkpoint):
            case 0: #Nothing has happened, we are in a random orientation
                if (self.furthest_1 != -1):
                    self.checkpoint = self.checkpoint + 1
            case 1: #Nothing has happened, we are in a random orientation
                if (self.furthest_1 < 360 + (self.small_offset)):
                    self.move_robot(0.0, -0.22)
                elif (self.furthest_1 > 360 + (self.big_offset)):
                    self.move_robot(0.0, 0.22)
                else:
                    self.checkpoint = self.checkpoint + 1
                self.start_ranges()
            # case 2:
            #     self.move_robot(0.26, 0.0)
            #     time.sleep(2.0)
            #     self.checkpoint = self.checkpoint + 1
            case _:
                self.move_robot(0.0, 0.0)


    def retrieve_distances(self, msg):
        if len(msg.ranges) == 720 :
            self.latest_ranges = msg.ranges
            self.state = (msg.ranges[540], msg.ranges[180], msg.ranges[360], msg.ranges[0]) # right is 180 
            self.get_logger().info(f'Lidar Reading: Left {self.state[0]} Right {self.state[1]} Front {self.state[2]} Back {self.state[3]}')
        
            if self.furthest_1 == -1:
                reduced = self.latest_ranges[270:541]
                reduced = [-1.0 if x > 7 else x for x in reduced]

                self.furthest_1_dist = max(reduced)
                self.furthest_1 = 270 + reduced.index(self.furthest_1_dist)
                
                # self.get_logger().info(f'Farthest initial {self.furthest_1}')


        
            
       
def main(args=None):
    rclpy.init(args=args)
    checkpoint_node = CheckpointNav()
    
    rclpy.spin(checkpoint_node)
    checkpoint_node.destroy_node()
    
    rclpy.shutdown()
    
                
if __name__ == '__main__':
    main()
    
 
    # def extract_zz (self):
    #     #Check all <.3 for range of 090 where 44turtleis the smallest
    #     # return index in lidar map

    # def extract_lidar_pos (self):
    #     #distance and angle from (0,0) + circular offset

    # def extract_robot_pos (self):
    #     #lidar pos shifted

    # def refine_lidar (self, msg, num):
        
    # 

    # def wall_check (self):
    #     #pivot left, pivot right, forward, inplace
    # def front_drive_check (self):