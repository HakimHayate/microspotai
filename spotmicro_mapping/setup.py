from setuptools import find_packages, setup

package_name = 'spotmicro_mapping'

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
    description='ROS 2 package for mapping, ICP, and graph optimization for the Microspot robot.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    
    entry_points={
        'console_scripts': [
            'frontend_mapping=spotmicro_mapping.frontend_mapping:main',
            'backend_mapping=spotmicro_mapping.backend_mapping:main',
            'execute_path=spotmicro_mapping.execute_path:main',
        ],
    },
)
