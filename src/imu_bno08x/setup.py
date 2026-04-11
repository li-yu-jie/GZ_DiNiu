# setuptools 用于打包 ament_python 功能包。
from setuptools import setup

# Python 功能包名，需要与目录名一致。
package_name = 'imu_bno08x'

setup(
    # 包的基础元信息。
    name=package_name,
    version='0.0.1',
    # 需要安装的 Python 包目录。
    packages=[package_name],
    data_files=[
        # 注册到 ament 包索引，ROS2 才能发现该功能包。
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 安装 package.xml 到 share 目录。
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='BNO08x IMU publisher over I2C.',
    license='Apache-2.0',
    entry_points={
        # console_scripts 是 ROS2 Python 节点的启动入口。
        # 启动命令：
        # ros2 run imu_bno08x imu_bno08x_node
        'console_scripts': [
            'imu_bno08x_node = imu_bno08x.imu_bno08x_node:main',
        ],
    },
)
