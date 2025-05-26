#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # ──────────────── Package Directories ────────────────
    serial_bridge_dir = get_package_share_directory("serial_bridge")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")

    # ──────────────── Launch Configurations ────────────────
    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    slam = LaunchConfiguration("slam", default="true")
    map_yaml = LaunchConfiguration(
        "map",
        default=PathJoinSubstitution([serial_bridge_dir, "maps", "B406_half_map.yaml"]),
    )
    params_file = LaunchConfiguration(
        "params_file",
        default=PathJoinSubstitution([serial_bridge_dir, "config", "nav2_params.yaml"]),
    )
    urdf_file = LaunchConfiguration(
        "urdf",
        default=PathJoinSubstitution(
            [serial_bridge_dir, "urdf", "sr_robot.urdf.xacro"]
        ),
    )
    rviz_config_file = PathJoinSubstitution(
        [serial_bridge_dir, "config", "navigation_view.rviz"]
    )
    bringup_launch = PathJoinSubstitution(
        [nav2_bringup_dir, "launch", "bringup_launch.py"]
    )

    # ──────────────── robot_description from Xacro ────────────────
    robot_desc = Command(["xacro ", urdf_file, " use_sim_time:=", use_sim_time])

    # ──────────────── SLAM Toolbox Node ────────────────
    slam_toolbox_node = Node(
        condition=IfCondition(slam),
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"odom_frame": "odom"},
            {"base_frame": "base_link"},
            {"map_frame": "map"},
            {"scan_topic": "/scan"},
        ],
    )

    # ──────────────── Nav2 Bringup ────────────────
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "true",
        }.items(),
    )

    # ──────────────── robot_state_publisher ────────────────
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "use_sim_time": use_sim_time}],
        output="screen",
    )

    # ──────────────── Static TFs ────────────────
    static_tf = [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_basefootprint",
            arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_baselaser",
            arguments=["0", "0", "0.18", "0", "0", "0", "base_link", "base_laser"],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_imu",
            arguments=["0", "0", "0.10", "0", "0", "0", "base_link", "imu_link"],
            output="screen",
        ),
    ]

    # ──────────────── RViz2 ────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # ──────────────── Trajectory Recorder ────────────────
    trajectory_recorder = Node(
        package="serial_bridge",
        executable="trajectory_recorder",
        name="trajectory_recorder",
        output="screen",
    )

    # ──────────────── Final Launch Description ────────────────
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "slam",
                default_value="True",
                description="Whether to run SLAM (true) or use a static map (false)",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=PathJoinSubstitution(
                    [serial_bridge_dir, "maps", "B406_half_map.yaml"]
                ),
                description="Full path to map file to load",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=PathJoinSubstitution(
                    [serial_bridge_dir, "config", "nav2_params.yaml"]
                ),
                description="Full path to the nav2 parameters file",
            ),
            DeclareLaunchArgument(
                "urdf",
                default_value=PathJoinSubstitution(
                    [serial_bridge_dir, "urdf", "sr_robot.urdf.xacro"]
                ),
                description="URDF or XACRO describing the robot model",
            ),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            rsp,
            *static_tf,
            slam_toolbox_node,
            bringup,
            rviz,
            trajectory_recorder,
        ]
    )
