import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/duttaishan01/robotics_course/enae450/turtlebot3_ws/install/turtlebot3_example'
