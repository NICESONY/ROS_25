import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ros25/ROS/ROS_25/colcon_ws/install/turtlebot3_dqn'
