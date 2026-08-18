import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('spotmicro_mapping')
    rviz_config_file = os.path.join(pkg_share, 'config', 'mapping_config.rviz')

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
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        ),
    ])