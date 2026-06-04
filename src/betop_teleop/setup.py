from glob import glob
import os

from setuptools import setup


package_name = "betop_teleop"


setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="北通鲲鹏20手柄遥控",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "omni_teleop = betop_teleop.omni_teleop_node:main",
            "odom_tf_bridge = betop_teleop.odom_tf_bridge:main",
            "steered_sensor_tf = betop_teleop.steered_sensor_tf:main",
        ],
    },
)
