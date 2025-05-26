from setuptools import find_packages, setup
import os
from glob import glob

package_name = "serial_bridge"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(include=[package_name, package_name + ".*"]),
    data_files=[
        # Required by ROS 2 index
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        # Include the package.xml
        (f"share/{package_name}", ["package.xml"]),
        # Install all launch files
        (f"share/{package_name}/launch", glob("launch/*.py")),
        # Install all config files
        (f"share/{package_name}/config", glob("config/*.*")),
        # Install urdf files
        (f"share/{package_name}/urdf", glob("urdf/*.*")),
        # Install map files
        (f"share/{package_name}/maps", glob("maps/*.*")),
    ],
    install_requires=[
        "setuptools",
        "rclpy",  # ROS 2 Python client
        "geometry_msgs",  # Message type dependency
        "pyserial",  # For serial port communication
    ],
    zip_safe=True,
    maintainer="cy",
    maintainer_email="1477530671@qq.com",
    description="串口桥接 ROS 2 节点包",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "serial_bridge = serial_bridge.serial_bridge_node:main",
            "serial_commander = serial_bridge.serial_commander:main",
            "wasdx_teleop = serial_bridge.wasdx_teleop:main",
            "point_move = serial_bridge.point_move:main",
            "trajectory_recorder = serial_bridge.trajectory_recorder:main",
            "trajectory_replayer = serial_bridge.trajectory_replayer:main",
        ],
    },
)
