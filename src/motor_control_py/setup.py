# os / glob 用于收集 launch 文件并安装到 share 目录。
import os
from glob import glob
from setuptools import setup

# 功能包名称。
package_name = 'motor_control_py'

setup(
    # 包基础元信息。
    name=package_name,
    version='0.0.1',
    # 需要被安装的 Python 源码包。
    packages=[package_name],
    data_files=[
        # 注册到 ament 包索引。
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 安装 package.xml。
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 文件，方便 ros2 launch 调用。
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Python motor control node using pigpio.',
    license='Apache-2.0',
    entry_points={
        # 命令行入口：
        # motor_control_node：底盘主控制
        # steer_control_node：旧版转向 PID 节点
        # speed_web_node：速度网页监控
        'console_scripts': [
            'motor_control_node = motor_control_py.motor_control_node:main',
            'steer_control_node = motor_control_py.steer_control_node:main',
            'speed_web_node = motor_control_py.speed_web_node:main',
        ],
    },
)
