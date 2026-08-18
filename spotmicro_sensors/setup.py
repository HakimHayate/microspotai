from setuptools import find_packages, setup

package_name = 'spotmicro_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdelhakim Hayate',
    maintainer_email='hakimhayate@gmail.com',
    description='ROS 2 package containing hardware drivers and data filters for the Microspot LiDAR and IMU sensors.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lidar_node=spotmicro_sensors.lidar_node:main',
            'imu_node=spotmicro_sensors.imu_node:main',
            'imu_filter_node=spotmicro_sensors.imu_filter_node:main',
            'imu_yaw_node=spotmicro_sensors.imu_yaw_node:main',
        ],
    },
)
