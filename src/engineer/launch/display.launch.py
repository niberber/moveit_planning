import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file = '/home/yh/2026_Engineer_ws/src/engineer/urdf/engineer.urdf'
    with open(urdf_file, 'r') as f:
        robot_description_content = f.read()
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2'
        )
    ])
