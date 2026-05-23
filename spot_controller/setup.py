from setuptools import find_packages, setup

package_name = 'spot_controller'

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
    maintainer='hakim',
    maintainer_email='hakimhayate@gmail.com',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'walking_gait=spot_controller.walking_gait:main',
            'real_robot_hardware_bridge=spot_controller.real_robot_hardware_bridge:main'
        ],
    },
)
