from setuptools import find_packages, setup

package_name = "serial_bridge"

setup(
    name=package_name,
    version="0.0.0",
    # 明确 include 本包和所有子模块
    packages=find_packages(include=[package_name, package_name + ".*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/serial_bridge_launch.py",
                "launch/serial_commander_launch.py",
                "launch/wasdx_teleop_launch.py",
                "launch/slam_launch.py",
                "launch/nav_with_map_launch.py",
                "launch/navigation2.launch.py",
                "launch/nav_with_map_and_gazebo.launch.py",
            ],
        ),
        ("share/" + package_name + "/config", ["config/serial_bridge.rviz"]),
    ],
    # 加上运行时依赖
    install_requires=[
        "setuptools",
        "rclpy",  # ROS 2 Python 客户端库
        "geometry_msgs",  # Twist 消息定义
        "pyserial",  # 串口通信
    ],
    zip_safe=True,
    maintainer="cy",
    maintainer_email="1477530671@qq.com",
    description="串口桥接 ROS2 节点包",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            # 1) 原先的上行节点
            "serial_bridge = serial_bridge.serial_bridge_node:main",
            # 2) 下行控制节点
            "serial_commander = serial_bridge.serial_commander:main",
            "wasdx_teleop = serial_bridge.wasdx_teleop:main",
            "point_move = serial_bridge.point_move:main",
            "trajectory_recorder = serial_bridge.trajectory_recorder:main",
            "trajectory_replayer = serial_bridge.trajectory_replayer:main",
        ],
    },
)
