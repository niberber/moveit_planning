import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    csv_file_arg = DeclareLaunchArgument(
        'csv_file',
        default_value='/home/yh/2026_Engineer_ws/trajectory.csv',
        description='Path to CSV trajectory file'
    )
    
    delay_ms_arg = DeclareLaunchArgument(
        'delay_ms',
        default_value='300',
        description='Delay per trajectory point in milliseconds'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='Loop playback'
    )
    
    urdf_file = '/home/yh/2026_Engineer_ws/src/engineer/urdf/engineer.urdf'
    
    if not os.path.exists(urdf_file):
        print(f"Warning: URDF file not found: {urdf_file}")
        urdf_content = ""
    else:
        with open(urdf_file, 'r') as f:
            urdf_content = f.read()
    
    return LaunchDescription([
        csv_file_arg,
        delay_ms_arg,
        loop_arg,
        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': urdf_content}],
            output='screen'
        ),
        
        Node(
            package='engineer_moveit2',
            executable='play_trajectory',
            name='play_trajectory',
            parameters=[{
                'csv_file': LaunchConfiguration('csv_file'),
                'delay_ms': LaunchConfiguration('delay_ms'),
                'loop': LaunchConfiguration('loop'),
                'pause_ms': 1000
            }],
            output='screen'
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['--aterror-exit-when-unexpectedly-stalled'],
            output='screen'
        )
    ])
