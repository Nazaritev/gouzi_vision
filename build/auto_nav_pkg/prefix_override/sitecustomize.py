import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gouzi/obstacle_race_src/install/auto_nav_pkg'
