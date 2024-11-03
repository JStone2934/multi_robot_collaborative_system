from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 启动Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '/home/jstone/Documents/graduation_project/T7/src/gazebo_tutorials/worlds/room4/room4.world', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen',
            name='gazebo'
        ),
        # 启动TurtleBot3并设置初始位置
        Node(
            package='turtlebot3_gazebo',
            executable='turtlebot3_gazebo',
            output='screen',
            parameters=[{'initial_pose_x': 1.0, 'initial_pose_y': 2.0, 'initial_pose_a': 0.0}]
        ),
    ])
