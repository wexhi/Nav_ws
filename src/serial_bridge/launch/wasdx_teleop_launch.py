#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription(
        [
            Node(
                package="serial_bridge",
                executable="wasdx_teleop",
                name="wasdx_teleop",
                output="screen",
                emulate_tty=True,  # ← 这一行保证给 wasdx_teleop 分配一个伪终端
            ),
        ]
    )
