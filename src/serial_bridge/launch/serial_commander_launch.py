from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="serial_bridge",
                executable="serial_commander",
                name="serial_commander",
                output="screen",
                parameters=[
                    # 用你自己 ls -l /dev/serial/by-id/ 得到的那个链接
                    {"port": "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"},
                    {"baudrate": 115200},
                ],
            ),
        ]
    )
