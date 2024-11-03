import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jstone/Documents/graduation_project/T7/install/gazebo_tutorials'
