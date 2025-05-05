import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from collections import deque

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
        
        # these are all approximated based off the diagram at the following link:
        # https://docs.google.com/drawings/d/1gDSnVN_PaYoXHPEN8NycJxKMBHnt_hwfdg-CdWP1--g/edit?usp=sharing
        # rests on the assumption that the blocks are 30 x 30 cm and that lidar measurments are in meters
        self.checkpoints = deque([
            ((.75,.75,3.45,.45),(.75,.75,3.45,.45),0),
            ((.75,.75,1.8,2.1),(2.1,1.8,.75,.75),1.86),
            ((.3,1.2,.3,1.2),(.3,1.2,1.2,.3),-1.86),
            ((1.2,1.2,.3,1.2),(1.2,.3,1.2,1.2),1.86),
            ((2.1,.9,.45,1.95),(.45,1.95,.9,2.1),-1.86),
            ((1.05,.45,.6,2.25),(2.25,.6,1.05,.45),1.86),
            ((.3,.6,.3,1.2),(.3,1.2,.6,.3),-1.86),
            ((float('inf'),.3,.3,.6),(.6,.3,float('inf'),.3),1.86),
            ((.6,.6,float('inf'),1.05),(.6,.6,float('inf'),1.05),0)            
            ])
        self.state = [None]*4 #left, right, front, back (540, 180, 360, 0)
        self.dist_threshold = 0.2 #account for robot dims so +15ish cm
        self.halfangle_threshold = 1
        self.timer = self.create_timer(0.05, self.control_loop)
        

    def reached_checkpoint(self, goalstate):
        for actual, target in zip(self.state, goalstate):
            if actual == float('inf'): #inf is just a possibility
                continue
            if actual is None or abs(actual - target) > self.dist_threshold:
                return False
        return True
    
        
    #this is being called repetitively at a very short interval
    def control_loop(self):
        if not self.checkpoints:
            self.publisher.publish(Twist()) #stop
            self.get_logger().info('All checkpoints done')
            return
        
        init, final, angle = self.checkpoints[0]
        msg = Twist()
        
        if not self.reached_checkpoint(init):
            msg.linear.x = 0.26
            self.publisher.publish(msg)
            return
        
        #otherwise we've entered a checkpoint
        self.get_logger().info(f'Checkpoint Reached. Turning by angle: {angle} rad')
        msg.angular.z = angle
        self.publisher.publish(msg)
        
        if self.reached_checkpoint(final): #want to consume the checkpoint
            #we've finished turning
            self.get_logger().info("Turn Complete.")
            #need to stop
            self.publisher.publish(Twist())
            self.checkpoints.popleft()
        


    def retrieve_distances(self, msg):
        dirs = (540, 180, 360, 0)
        
        for j,dir in enumerate(dirs):
            total = 0
            count = 0
            for i in range(dir - self.halfangle_threshold, dir + self.halfangle_threshold + 1):
                if msg.ranges[i] != float('inf'):
                    total += msg.ranges[i]
                    count += 1
            self.state[j] = total/count if count > 0 else float('inf')
            
       
def main(args=None):
    rclpy.init(args=args)
    checkpoint_node = CheckpointNav()
    
    rclpy.spin(checkpoint_node)
    checkpoint_node.destroy_node()
    
    rclpy.shutdown()
    
                
if __name__ == '__main__':
    main()
    