from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Path to RViz config (save your rviz setup as config/serial_bridge.rviz)
    pkg_share = get_package_share_directory("serial_bridge")
    rviz_config = os.path.join(pkg_share, "config", "serial_bridge.rviz")

    return LaunchDescription(
        [
            # 1) Launch the serial_bridge node
            Node(
                package="serial_bridge",
                executable="serial_bridge",
                name="serial_bridge_node",
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
            # 2) Launch RViz with the saved configuration
            # Node(
            #     package="rviz2",
            #     executable="rviz2",
            #     name="rviz2",
            #     output="screen",
            #     arguments=["-d", rviz_config],
            # ),
        ]
    )
