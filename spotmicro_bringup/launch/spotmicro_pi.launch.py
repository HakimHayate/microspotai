from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='spotmicro_sensors',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),
        Node(
            package='spotmicro_sensors',
            executable='lidar_node',
            name='lidar_node',
            output='screen'
        ),
        Node(
            package='spotmicro_sensors',
            executable='imu_yaw_node',
            name='imu_yaw_node',
            output='screen'
        ),
        Node(
            package='spotmicro_sensors',
            executable='imu_filter_node',
            name='imu_filter_node',
            output='screen'
        ),

        Node(
            package='spotmicro_hardware_controller',
            executable='hardware_bridge',
            name='hardware_bridge',
            output='screen'
        ),
        Node(
            package='spotmicro_controller',
            executable='pi_gait',
            name='pi_gait',
            output='screen'
        ),
    ])