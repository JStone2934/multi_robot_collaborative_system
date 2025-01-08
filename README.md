在根目录下运行

```bash
colcon build
```

然后（每次打开新的终端都需要source一次）
```bash
source src/install/setup.bash
```
终端1：启动gazebo并且放入机器人，打开激光雷达等等
```bash
ros2 launch turtlebot3_gazebo multi_robot_launch.py
```
终端2：运行地图合并程序
```bash
ros2 launch merge_map merge_map_launch.py
```
终端3：运行机器人自动搜索程序
```bash
ros2 run multi_robot_exploration control
```
