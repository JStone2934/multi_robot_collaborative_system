# multi_robot_collaborative_system
基于ROS的多机器人协同搜索系统
本项目使用的是ROS2 galactic，Ubuntu 20.04, gazebo11, turtlebot3
## 单机器人运行demo
在项目根目录打开终端
```bash
colcon build
```
注意每次打开终端都要运行
```bash
source install/setup.bash
```
下面的命令会在gazebo中加载地图world/room4.world，并且放入一个turtlebot3机器人
```bash
ros2 launch gazebo_tutorials turtlebot_launch.py
```
新开终端，运行rviz进行cartographer建图
```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py 
```
新开终端，进行键盘控制
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```