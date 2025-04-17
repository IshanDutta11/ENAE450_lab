import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/duttaishan01/robotics_course/ENAE450_lab/turtlebot3_ws/install/lab3'
