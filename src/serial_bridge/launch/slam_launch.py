from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Optional RViz config path
    rviz_config_file = (
        "/home/cy/Study/Nav_ws/config/slam_toolbox_view.rviz"  # Change if needed
    )

    return LaunchDescription(
        [
            # ───── Static TF base_link → base_laser ─────
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_laser_tf_pub",
                arguments=["0", "0", "0.1", "0", "0", "0", "base_link", "base_laser"],
                output="screen",
            ),
            # ───── SLAM Toolbox ─────
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": False,
                        "scan_topic": "/scan",
                        "odom_frame": "odom",
                        "base_frame": "base_link",
                        "map_frame": "map",
                    }
                ],
            ),
            # ───── Trajectory Recorder ─────
            Node(
                package="serial_bridge",
                executable="trajectory_recorder",
                name="trajectory_recorder",
                output="screen",
            ),
            # ───── RViz2 ─────
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_file],
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
        ]
    )
