import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'butler_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Default files (Resource Index and Package XML)
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # 1. Install ALL Launch Files (cafe_sim.launch.py and robot_description.launch.py)
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[py]*'))),
        
        # 2. Install World Files (Your custom Gazebo world)
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),

        # 3. Install Robot Description Files (URDF/XACRO and any associated meshes/config)
        # Note: 'description' folder name matches path used in robot_description.launch.py
        (os.path.join('share', package_name, 'description'), glob(os.path.join('description', '*.*'))), 

        # 4. Install Config Files (for waypoints, map settings, etc.)
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Melvin Sajith',
    maintainer_email='melvin@todo.todo',
    description='ROS 2 Butler Robot for the French Door Cafe',
    license='Apache-2.0',
    tests_require=['pytest'],
    # **NOTE**: Add Python node executables here once you start writing the Action Server
    entry_points={
        'console_scripts': [
            # 'butler_action_server = butler_pkg.butler_server:main', 
            # 'order_client = butler_pkg.order_client:main',
        ],
    },
)