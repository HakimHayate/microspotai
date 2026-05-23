from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_path
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit


def generate_launch_description():
    urdf_path = os.path.join(get_package_share_path('microspot_description'),
                            'urdf',
                            'microspot.urdf')
    
    ros_gz_sim_dir = get_package_share_path('ros_gz_sim')

    robot_desc = None
    with open(urdf_path, 'r') as f:
        robot_desc = f.read()
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_desc}]
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': 'empty.sdf -r'
        }.items()
    )

    robot_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        name='microspot_spawner',
        output='screen',
        arguments=[
            '-topic', 'robot_description', 
            '-name', 'microspot',          
            '-z', '0.1'                    
        ]
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
        output='screen'
    )

    raw_position_bridge_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['raw_position_bridge', '-c', 'controller_manager'],
        output='screen'
    )

    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_spawner,
            on_exit=[joint_state_broadcaster_spawner, raw_position_bridge_spawner],
        )
    )

    return LaunchDescription([
        robot_state_publisher_node,
        gazebo_launch,
        robot_spawner,
        delay_controllers
    ])

