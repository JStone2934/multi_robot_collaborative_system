#多机器人协同搜索系统
在ROBOT_ARMY下运行编译命令：

'''bash
colcon build
'''
每次打开一个新的终端都要运行：
'''bash
source install/setup.bash
'''
第一个终端启动机器人和gazebo：
''bash
ros2 launch turtlebot3_gazebo multi_robot_launch.py
'''
第二个终端运行地图合成器：
'''bash
ros2 launch merge_map merge_map_launch.py
'''
第三个终端运行自动探图控制器：
'''bash
ros2 run multi_robot_exploration control
'''