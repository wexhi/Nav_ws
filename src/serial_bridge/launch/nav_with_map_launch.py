#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

MAP_FILE = "/home/cy/Study/Nav_ws/maps/B406_half_map.yaml"
PARAM_FILE = "/home/cy/Study/Nav_ws/config/nav2_params.yaml"
BT_XML = "/home/cy/Study/Nav_ws/config/my_nav_to_pose_bt.xml"  # 或改为 nav_to_pose BT


def generate_launch_description():
    return LaunchDescription(
        [
            # ── base_link → base_laser ──
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="static_tf_baselink_baselaser",
                arguments=["0", "0", "0.18", "0", "0", "0", "base_link", "base_laser"],
                output="screen",
            ),
            # ── Map Server & AMCL ──
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
            # ── Planner / Smoother ──
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
            # ── 控制器延迟启动（避免 costmap 还没好） ──
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
            # ── Recovery / Behavior Tree ──
            Node(
                package="nav2_behaviors",
                executable="behavior_server",
                name="behavior_server",
                parameters=[PARAM_FILE],
                output="screen",
            ),
            Node(
                package="nav2_bt_navigator",
                executable="bt_navigator",
                name="bt_navigator",
                parameters=[PARAM_FILE, {"default_nav_to_pose_bt_xml": BT_XML}],
            ),
            # ── Local & Global Costmap ──
            Node(
                package="nav2_costmap_2d",
                executable="costmap_2d_node",
                name="local_costmap",
                namespace="local_costmap",
                parameters=[PARAM_FILE],
                output="screen",
            ),
            Node(
                package="nav2_costmap_2d",
                executable="costmap_2d_node",
                name="global_costmap",
                namespace="global_costmap",
                parameters=[PARAM_FILE],
                output="screen",
            ),
            # ── Lifecycle Manager ──
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
                            "behavior_server",
                            "bt_navigator",
                            "local_costmap",  # ✅ 添加这一行
                            "global_costmap",  # ✅ 添加这一行
                        ],
                    }
                ],
            ),
        ]
    )
