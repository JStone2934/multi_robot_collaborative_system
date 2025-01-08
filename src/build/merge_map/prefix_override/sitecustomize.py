import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jstone/Documents/graduation_project/9T/robot_army/src/install/merge_map'
