import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('microspot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'spot_arm.urdf.xacro')
    
    robot_description_content = Command(['xacro ', xacro_file])
    rviz_config = os.path.join(pkg_share, 'rviz', 'config.rviz')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])