from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'capstone_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']) + ['scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.lua')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='kim@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    
    entry_points={
        'console_scripts': [
            'teleop_keyboard = capstone_robot.teleop_keyboard:main',
        ],
    },
)
