# your_ws/src/your_pkg/launch/serial_commander_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="serial_commander",
                executable="serial_commander",
                name="serial_commander",
                output="screen",
                parameters=[  # 可选，把串口参数也做成可配置
                    {"port": "/dev/ttyUSB0"},
                    {"baudrate": 115200},
                ],
            ),
        ]
    )
