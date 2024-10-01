import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mounted_volume/ros2_ws/src/turtlebot_sim/install/turtlebot_sim'
