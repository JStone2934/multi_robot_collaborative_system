#!/bin/bash
# 启动第一个终端并启动机器人和Gazebo
gnome-terminal -- bash -c "source install/setup.bash && ros2 launch turtlebot3_gazebo multi_robot_launch.py; exec bash"

# 启动第二个终端并运行地图合成器
gnome-terminal -- bash -c "source install/setup.bash && ros2 launch merge_map merge_map_launch.py; exec bash"

# 启动第三个终端并运行图像识别
gnome-terminal -- bash -c "source install/setup.bash && ros2 run yolov5_ros2 yolo_detect_2d --ros-args -p device:=cpu; exec bash"

# 启动第四个终端并运行目标检测
gnome-terminal -- bash -c "source install/setup.bash && ros2 launch target_detect target_detect_launch.py; exec bash"

# 启动第五个终端并运行自动探图控制器
gnome-terminal -- bash -c "source install/setup.bash && ros2 run multi_robot_exploration control; exec bash"
