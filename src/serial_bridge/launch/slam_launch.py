from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 发布 base_link -> base_laser 的静态 TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_laser_tf_pub',
            arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'base_laser']
        ),

        # 启动 SLAM Toolbox，指定 scan_topic 参数
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'scan_topic': '/scan',           # 显式指定扫描话题
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'map_frame': 'map'
            }]
        )
    ])
