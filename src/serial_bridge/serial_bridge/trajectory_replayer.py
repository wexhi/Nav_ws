import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv
import time


class TrajectoryReplayer(Node):
    def __init__(self):
        super().__init__("trajectory_replayer")

        self.path_publisher = self.create_publisher(Path, "/trajectory_replay", 10)
        self.declare_parameter("csv_path", "trajectory.csv")

        self.timer = self.create_timer(1.0, self.publish_path)

        self.path = Path()
        self.path.header.frame_id = "odom"

        self.load_trajectory_from_csv(
            self.get_parameter("csv_path").get_parameter_value().string_value
        )

    def load_trajectory_from_csv(self, filename):
        with open(filename, newline="") as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                pose = PoseStamped()
                pose.header.frame_id = "odom"
                pose.pose.position.x = float(row["x"])
                pose.pose.position.y = float(row["y"])
                pose.pose.position.z = float(row["z"])
                pose.pose.orientation.x = float(row["qx"])
                pose.pose.orientation.y = float(row["qy"])
                pose.pose.orientation.z = float(row["qz"])
                pose.pose.orientation.w = float(row["qw"])
                self.path.poses.append(pose)
        self.get_logger().info(f"Loaded {len(self.path.poses)} poses from {filename}")

    def publish_path(self):
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_publisher.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryReplayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
