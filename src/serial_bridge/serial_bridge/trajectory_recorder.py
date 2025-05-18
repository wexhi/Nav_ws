import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__("trajectory_recorder")

        # Publisher for trajectory path
        self.path_publisher = self.create_publisher(Path, "/trajectory", 10)

        # Subscriber to /odom
        self.odom_subscriber = self.create_subscription(
            Odometry,
            "/odom",  # <- Using /odom which updates constantly
            self.odom_callback,
            10,
        )

        # Store path
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"  # Matches odom frame

    def odom_callback(self, msg: Odometry):
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(pose_stamped)

        self.path_publisher.publish(self.path_msg)
        # self.get_logger().info(f"Recorded pose. Total: {len(self.path_msg.poses)}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
