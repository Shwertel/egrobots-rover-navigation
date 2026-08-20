import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'obstacle_avoider'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
	(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shwertel',
    maintainer_email='shwertel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'scan_subscriber = obstacle_avoider.scan_subscriber:main',
		'velocity_publisher = obstacle_avoider.velocity_publisher:main',
		'obstacle_avoider_node = obstacle_avoider.obstacle_avoider_node:main',
		'manual_controller = obstacle_avoider.manual_controller:main'
        ],
    },
)
