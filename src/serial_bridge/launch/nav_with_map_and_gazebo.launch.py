#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    # ──────────────── 参数配置 ────────────────
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    map_yaml = LaunchConfiguration(
        "map", default="/home/cy/Study/Nav_ws/maps/B406_half_map.yaml"
    )
    params_file = LaunchConfiguration(
        "params_file", default="/home/cy/Study/Nav_ws/config/nav2_params.yaml"
    )
    urdf_file = LaunchConfiguration(
        "urdf", default="/home/cy/Study/Nav_ws/urdf/sr_robot.urdf.xacro"
    )
    world_file = LaunchConfiguration(
        "world", default="/home/cy/Study/Nav_ws/worlds/my_room.world"
    )

    # ──────────────── robot_description ────────────────
    robot_desc = Command(["xacro ", urdf_file, " use_sim_time:=", use_sim_time])

    # ──────────────── 启动 Gazebo 仿真 ────────────────
    gazebo = ExecuteProcess(
        cmd=["gazebo", "--verbose", world_file, "-s", "libgazebo_ros_factory.so"],
        output="screen",
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "robot_description", "-entity", "sweeper_bot"],
        output="screen",
    )

    # ──────────────── 启动 robot_state_publisher ────────────────
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc, "use_sim_time": use_sim_time}],
        output="screen",
    )

    # ──────────────── 静态 TF（base_link → laser, imu, footprint）──────────────
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

    # ──────────────── Nav2 bringup 入口 ────────────────
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    bringup_launch = PathJoinSubstitution(
        [nav2_bringup_dir, "launch", "bringup_launch.py"]
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            "map": map_yaml,
            "use_sim_time": use_sim_time,
            "params_file": params_file,
            "autostart": "true",
        }.items(),
    )

    # ──────────────── RViz 可视化 ────────────────
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [
                    get_package_share_directory("nav2_bringup"),
                    "rviz",
                    "nav2_default_view.rviz",
                ]
            ),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation clock if true",
            ),
            DeclareLaunchArgument(
                "map",
                default_value="/home/cy/Study/Nav_ws/maps/B406_half_map.yaml",
                description="Map file",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value="/home/cy/Study/Nav_ws/config/nav2_params.yaml",
                description="Nav2 parameters",
            ),
            DeclareLaunchArgument(
                "urdf",
                default_value="/home/cy/Study/Nav_ws/urdf/sr_robot.urdf.xacro",
                description="URDF robot model",
            ),
            DeclareLaunchArgument(
                "world",
                default_value="/home/cy/Study/Nav_ws/worlds/my_room.world",
                description="Gazebo world file",
            ),
            SetEnvironmentVariable("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
            gazebo,
            rsp,
            spawn_entity,
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_odom",
                parameters=["/home/cy/Study/Nav_ws/config/ekf_odom.yaml"],
                output="screen",
            ),
            *static_tf,
            bringup,
            rviz,
        ]
    )
