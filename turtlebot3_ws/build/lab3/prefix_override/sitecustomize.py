import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/saijay/enae450/FinalProject/ENAE450_lab/turtlebot3_ws/install/lab3'
