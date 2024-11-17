import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/yusuf/turtlebot3_ws/install/yusuf_ozyer'
