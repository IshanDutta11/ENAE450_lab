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
        # self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        # self.checkpoints = deque([]) #POPULATE WITH STATES LATER
        self.state = (10.0000000000000, 10.0000000000000, 10.0000000000000, 10.0000000000000)
        self.threshold = 0 #account for robot dims so +15ish cm
        # self.timer = self.create_timer(0.1, self.control_loop)
        

    def reached_checkpoint(self, goalstate):
        self.get_logger().info(f'CHECKPOINT')
        # for actual, target in zip(self.state, goalstate):
        #     if actual is None or abs(actual - target) > self.threshold:
        #         return False
        # return True
    
        
    #this is being called repetitively at a very short interval
    def control_loop(self):
        self.get_logger().info(f'')
        # if not self.checkpoints:
        #     self.publisher.publish(Twist()) #stop
        #     self.get_logger().info('All checkpoints done')
        #     return
        
        # init, final, angle = self.checkpoints[0]
        # msg = Twist()
        
        # if not self.reached_checkpoint(init):
        #     msg.linear.x = 0.26
        #     self.publisher.publish(msg)
        #     return
        
        #otherwise we've entered a checkpoint
        # self.get_logger().info(f'Checkpoint Reached. Turning by angle: {angle} rad')
        # msg.linear.z = angle
        # self.publisher.publish(msg)
        
        # if self.reached_checkpoint(final): #want to consume the checkpoint
        #     #we've finished turning
        #     self.get_logger().info("Turn Complete.")
        #     #need to stop
        #     self.publisher.publish(Twist())
        #     self.checkpoints.popleft()
        


    def retrieve_distances(self, msg):
        if len(msg.ranges) ==720 :
            req_ranges[540, 180, 360, 0]


            self.state = (msg.ranges[540], msg.ranges[180], msg.ranges[360], msg.ranges[719]) # 
            # self.get_logger().info(f'Lidar Reading: Left {self.state[0]} Right {self.state[1]} Front {self.state[2]} Back {self.state[3]}')
            # self.get_logger().info(f'Back_-1 {msg.ranges[719]} Back {msg.ranges[0]} Back1_+1 {msg.ranges[1]}')
            self.get_logger().info(f'Back {(msg.ranges[0])}')
            self.get_logger().info(f'Back {type(msg.ranges[0])}')
        else :
            self.get_logger().info(f"Buggin like sai")
            
    def refine_lidar_reading(self, msg, req):
        for x in range(0,4):
            value = float('inf')
            if(msg.ranges[req[x]] == value):
                before = req[x] -1
                if(before == -1) : 
                    before = 719
                after = req[x]+1

            self.state[x] = value


       
def main(args=None):
    rclpy.init(args=args)
    checkpoint_node = CheckpointNav()
    
    rclpy.spin(checkpoint_node)
    checkpoint_node.destroy_node()
    
    rclpy.shutdown()
    
                
if __name__ == '__main__':
    main()
    