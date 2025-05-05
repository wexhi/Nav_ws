#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

MAP_FILE = "/home/cy/Study/Nav_ws/maps/B406_half_map.yaml"
PARAM_FILE = "/home/cy/Study/Nav_ws/config/nav2_params.yaml"  # 一份总参数
CONTORLER_FILE = "/home/cy/Study/Nav_ws/config/controller.yaml"  # 控制器参数
BT_XML = "/home/cy/Study/Nav_ws/config/nav_through_poses_without_wait.xml"  # 如果你运行的是 NavigateThroughPoses


def generate_launch_description():
    return LaunchDescription(
        [
            # ── base_link → base_laser ──
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_baselink_baselaser",
                arguments=[
                    "0",
                    "0",
                    "0.18",  # x y z
                    "0",
                    "0",
                    "0",  # yaw pitch roll
                    "base_link",
                    "base_laser",
                ],
                output="screen",
            ),
            # ── Map / Localization ──
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                parameters=[PARAM_FILE, {"yaml_filename": MAP_FILE}],
            ),
            Node(
                package="nav2_amcl",
                executable="amcl",
                name="amcl",
                parameters=[PARAM_FILE],
            ),
            # ── Path / Smoothing / Control ──
            Node(
                package="nav2_planner",
                executable="planner_server",
                name="planner_server",
                parameters=[PARAM_FILE],
            ),
            Node(
                package="nav2_smoother",
                executable="smoother_server",
                name="smoother_server",
                parameters=[PARAM_FILE],
            ),
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package="nav2_controller",
                        executable="controller_server",
                        name="controller_server",
                        parameters=[PARAM_FILE],
                        output="screen",
                    )
                ],
            ),
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                output="screen",
                parameters=[PARAM_FILE],
            ),
            # ── BT Navigator ──
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                parameters=[PARAM_FILE, {"default_nav_to_pose_bt_xml": BT_XML}],
            ),
            # ── Lifecycle ──
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                parameters=[
                    {
                        "autostart": True,
                        "node_names": [
                            "map_server",
                            "amcl",
                            "planner_server",
                            "smoother_server",
                            "controller_server",
                            "behavior_server",  # ← 加这一行
                            "bt_navigator",
                        ],
                    }
                ],
            ),
        ]
    )
