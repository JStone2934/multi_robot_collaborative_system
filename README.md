# 多机器人协同搜索系统

两个机器人在Gazebo仿真房屋环境下进行slam探索，通过yolov5检测目标并根据激光雷达数据解算目标坐标位置。

## 项目运行方法：

1.在根目录下运行编译命令：
```bash
colcon build
```
每次打开一个新的终端都要运行：
```bash
source install/setup.bash
```
2.第一个终端启动机器人和gazebo：
```bash
ros2 launch turtlebot3_gazebo multi_robot_launch.py
```
3.第二个终端运行地图合成器：
```bash
ros2 launch merge_map merge_map_launch.py
```
4.第三个终端运行图像识别：
```bash
ros2 run yolov5_ros2 yolo_detect_2d --ros-args -p device:=cpu 
```
5.第四个终端运行目标坐标解算：
```bash
ros2 launch target_detect target_detect_launch.py 
```
6.第五个终端运行自动探图控制器：
```bash
ros2 run multi_robot_exploration control
```

## 致谢

对项目 

https://github.com/abdulkadrtr/multiRobotExploration-RobotArmy.git 

https://github.com/fishros/yolov5_ros2.git

等进行了参考。
