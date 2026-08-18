from setuptools import find_packages, setup

package_name = 'spotmicro_controller'

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
    description='ROS 2 package for Microspot robot core control, and gait generation.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller_node=spotmicro_controller.controller_node:main',
            'real_robot_hardware_bridge=spotmicro_controller.real_robot_hardware_bridge:main',
            'pi_gait=spotmicro_controller.pi_gait:main',
        ],
    },
)
