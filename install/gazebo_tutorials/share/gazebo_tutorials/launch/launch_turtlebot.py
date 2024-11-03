from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node
import os

def generate_launch_description():
    return LaunchDescription([
        PushRosNamespace(namespace='turtlebot3'),
        Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_gazebo',
            output='screen',
            parameters=[{'initial_pose_x': 1.0, 'initial_pose_y': 2.0, 'initial_pose_a': 0.0}]  # 设置初始位置
        ),
    ])
