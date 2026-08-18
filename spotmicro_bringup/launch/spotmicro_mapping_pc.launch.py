from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spotmicro_mapping',
            executable='frontend_mapping',
            name='frontend_mapping',
            output='screen'
        ),
        Node(
            package='spotmicro_mapping',
            executable='backend_mapping',
            name='backend_mapping',
            output='screen'
        ),
        
    ])
