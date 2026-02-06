import os
from glob import glob
from setuptools import setup

package_name = 'motor_control_py'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Python motor control node using pigpio.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'motor_control_node = motor_control_py.motor_control_node:main',
            'steer_control_node = motor_control_py.steer_control_node:main',
        ],
    },
)
