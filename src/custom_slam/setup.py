from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'custom_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), 
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'params'), glob('params/*.yaml')), 
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),    
        (os.path.join('lib', package_name), glob('scripts/*.py')),      
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='kim@todo.todo',
    description='Custom SLAM package with URDF configuration',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = custom_slam.navigation_node:main',
            'map_listener = custom_slam.map_listener:main',
            'go_to_pose = Nav_go_to_pose:main',
        ],
    },
)
