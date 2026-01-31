from setuptools import setup

package_name = 'imu_bno08x'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='BNO08x IMU publisher over I2C.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_bno08x_node = imu_bno08x.imu_bno08x_node:main',
        ],
    },
)
