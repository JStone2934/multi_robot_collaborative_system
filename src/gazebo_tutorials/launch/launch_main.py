from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    return LaunchDescription([
        # 启动 Gazebo，并加载自定义的 world 文件
        ExecuteProcess(
            cmd=['gazebo', 
                 '--verbose', 
                 '/home/jstone/Documents/graduation_project/T7/src/gazebo_tutorials/worlds/room4/room4.world', 
                 '-s', 
                 'libgazebo_ros_init.so', 
                 '-s', 
                 'libgazebo_ros_factory.so'],
            output='screen',
            name='gazebo'
        ),
        # 启动 TurtleBot3
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join('/opt/ros/galactic/share/turtlebot3_gazebo/launch', 'turtlebot3_world.launch.py')),
            launch_arguments={
                'turtlebot3_model': 'burger',  # 根据您的模型选择
                'use_sim_time': 'true',  # 使用仿真时间
            }.items()
        ),
    ])
