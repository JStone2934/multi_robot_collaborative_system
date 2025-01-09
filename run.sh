#!/bin/bash

# 编译工作空间
echo "开始编译工作空间..."
colcon build

# 确保每次打开新终端时都运行source
echo "运行 source install/setup.bash..."
source install/setup.bash

# 启动机器人和Gazebo（第一个终端）
echo "启动机器人和Gazebo..."
gnome-terminal -- bash -c "ros2 launch turtlebot3_gazebo multi_robot_launch.py; exec bash"

# 启动地图合成器（第二个终端）
echo "启动地图合成器..."
gnome-terminal -- bash -c "ros2 launch merge_map merge_map_launch.py; exec bash"

# 启动自动探图控制器（第三个终端）
echo "启动自动探图控制器..."
gnome-terminal -- bash -c "ros2 run multi_robot_exploration control; exec bash"

echo "所有终端已启动。"
