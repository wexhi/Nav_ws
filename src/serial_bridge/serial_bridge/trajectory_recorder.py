import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import math
import csv
import os


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__("trajectory_recorder")

        # Publisher for trajectory path
        self.path_publisher = self.create_publisher(Path, "/trajectory", 10)

        # Publisher for accumulated distance marker in RViz
        self.marker_publisher = self.create_publisher(Marker, "/distance_marker", 10)

        # Subscriber to odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10,
        )

        # Initialize Path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = "odom"

        # Distance tracking
        self.last_position = None
        self.total_distance = 0.0

    def odom_callback(self, msg: Odometry):
        # Extract current pose
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose

        # Append to path and publish
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(pose_stamped)
        self.path_publisher.publish(self.path_msg)

        # Calculate distance
        current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
        )

        if self.last_position is not None:
            dx = current_position[0] - self.last_position[0]
            dy = current_position[1] - self.last_position[1]
            distance = math.sqrt(dx**2 + dy**2)
            self.total_distance += distance

            # Log and visualize distance
            self.get_logger().info(f"Accumulated Distance: {self.total_distance:.2f} m")
            self.publish_distance_marker(current_position)

        self.last_position = current_position

    def publish_distance_marker(self, position):
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "distance"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = 1.0  # display above robot
        marker.scale.z = 0.3  # text size
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.text = f"Distance: {self.total_distance:.2f} m"
        self.marker_publisher.publish(marker)

    def destroy_node(self):
        # Save trajectory on shutdown
        self.save_trajectory_to_csv("trajectory_logs/trajectory.csv")
        self.get_logger().info("Trajectory saved. Shutting down node.")
        super().destroy_node()

    def save_trajectory_to_csv(self, filename):
        # Ensure directory exists
        os.makedirs(os.path.dirname(filename), exist_ok=True)

        with open(filename, "w", newline="") as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(["timestamp", "x", "y", "z", "qx", "qy", "qz", "qw"])
            for pose_stamped in self.path_msg.poses:
                pos = pose_stamped.pose.position
                ori = pose_stamped.pose.orientation
                stamp = pose_stamped.header.stamp
                timestamp = stamp.sec + stamp.nanosec * 1e-9
                writer.writerow(
                    [timestamp, pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]
                )
        self.get_logger().info(f"Trajectory saved to: {filename}")


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
