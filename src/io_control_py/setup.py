# setuptools 用于定义 Python 功能包安装信息。
from setuptools import setup

# 功能包名称，需要与目录结构一致。
package_name = 'io_control_py'

setup(
    # 包的基本元数据。
    name=package_name,
    version='0.0.1',
    # 安装 Python 源码包。
    packages=[package_name],
    data_files=[
        # 注册到 ament 索引。
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 安装 package.xml。
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='GPIO IO control node using pigpio.',
    license='Apache-2.0',
    entry_points={
        # ROS2 Python 节点入口。
        # 启动命令：
        # ros2 run io_control_py io_control_node
        'console_scripts': [
            'io_control_node = io_control_py.io_control_node:main',
        ],
    },
)
