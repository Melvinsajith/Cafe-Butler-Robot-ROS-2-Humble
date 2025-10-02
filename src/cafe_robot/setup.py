from setuptools import setup

package_name = 'cafe_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=[
        'setuptools',
        'rclpy',
        'PyQt5',
        'transitions',
    ],
    zip_safe=True,
    maintainer='Melvin_Sajith',
    maintainer_email='melvinsajith20@gmail.com',
    description='Cafe Robot Order Manager and GUI',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'order_manager_node = cafe_robot.order_manager_node:main',
            'gui_dashboard = cafe_robot.gui_dashboard:main',
        ],
    },
)
