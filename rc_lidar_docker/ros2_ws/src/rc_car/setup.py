from setuptools import setup

package_name = 'rc_car'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
        ('share/' + package_name + '/web', ['rc_car/web/index.html']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rc',
    maintainer_email='user@localhost',
    description='RC car control, odometry, command manager, web bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vehicle_node = rc_car.vehicle_node:main',
            'odometry_node = rc_car.odometry_node:main',
            'command_manager = rc_car.command_manager:main',
            'web_bridge = rc_car.web_bridge:main',
        ],
    },
)
