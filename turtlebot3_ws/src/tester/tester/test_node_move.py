import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from collections import deque
import time

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
        self.checkpoints = deque([]) #POPULATE WITH STATES LATER
        self.state = (None, None, None, None) #left, right, front, back (540, 180, 360, 0)
        self.dist_threshold = 0.1 #account for robot dims so +15ish cm
        self.halfangle_threshold = 1
        
        time.sleep(1.0) 
        self.initial_movement()

        self.timer = self.create_timer(0.1, self.control_loop)
       
    def intial_movement(self):
        self.goalstate = (self.state[2], self.state[1], self.state[3], self.state[0])
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 1.182
        self.publisher_.publish(msg)

    def reached_checkpoint(self):
        for actual, target in zip(self.state, self.goalstate):
            if actual is None or abs(actual - target) > self.dist_threshold:
                return False
        return True
    
        
    #this is being called repetitively at a very short interval
    def control_loop(self):
        msg = Twist()
        
        if(self.reached_checkpoint):
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
 


    def retrieve_distances(self, msg):
        if len(msg.ranges) ==720 :
            self.state = (msg.ranges[540], msg.ranges[180], msg.ranges[360], msg.ranges[0]) # right is 180 
            # self.get_logger().info(f'Lidar Reading: Left {self.state[0]} Right {self.state[1]} Front {self.state[2]} Back {self.state[3]}')
            # self.get_logger().info(f'Back_-1 {msg.ranges[719]} Back {msg.ranges[0]} Back1_+1 {msg.ranges[1]}')
            self.get_logger().info(f'Back_-1 {msg.ranges[719]} Back {msg.ranges[0]} Back1_+1 {msg.ranges[1]}')
        else :
            self.get_logger().info(f"Buggin like sai")
            
       
def main(args=None):
    rclpy.init(args=args)
    checkpoint_node = CheckpointNav()
    
    rclpy.spin(checkpoint_node)
    checkpoint_node.destroy_node()
    
    rclpy.shutdown()
    
                
if __name__ == '__main__':
    main()
    
 