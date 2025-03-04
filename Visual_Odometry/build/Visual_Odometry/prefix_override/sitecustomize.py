import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ibrahim/ros2_ws/src/Visual_Odometry/install/Visual_Odometry'
