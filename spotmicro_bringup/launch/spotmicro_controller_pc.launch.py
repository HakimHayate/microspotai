import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('spotmicro_description')
    
    urdf_file = os.path.join(pkg_share, 'urdf', 'micro_v2.urdf')
    rviz_config = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')
    
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
        
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    controller_node = Node(
        package='spotmicro_controller',
        executable='controller_node',
        name='controller_node',
        output='screen'
    )
    
    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node,
        controller_node
    ])