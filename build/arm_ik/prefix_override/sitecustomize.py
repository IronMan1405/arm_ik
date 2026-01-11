import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dakshesh/ros2_ws/src/arm_ik/install/arm_ik'
