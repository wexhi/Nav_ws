#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
import math


class PointMove(Node):
    def __init__(self):
        super().__init__("point_move")
        self.publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.subscription = self.create_subscription(
            Odometry, "odom", self.odom_callback, 10
        )
        self.timer = self.create_timer(0.01, self.timer_callback)  # 高频率：更平滑控制

        # 初始状态
        self.start_x = None
        self.start_y = None
        self.start_yaw = None
        self.current_x = None
        self.current_y = None
        self.current_yaw = None
        self.phase = "forward"  # 'forward' -> 'rotate' -> 'stop'
        self.last_log_time = self.get_clock().now()

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.current_yaw = yaw

        if self.start_x is None:
            self.start_x = self.current_x
            self.start_y = self.current_y
            self.get_logger().info(
                f"✅ 起始位置记录: x = {self.start_x:.3f}, y = {self.start_y:.3f}"
            )
        if self.start_yaw is None:
            self.start_yaw = self.current_yaw
            self.get_logger().info(
                f"✅ 起始角度记录: yaw = {math.degrees(self.start_yaw):.2f}°"
            )

    def timer_callback(self):
        if None in (self.current_x, self.current_y, self.current_yaw):
            return  # 等待数据准备好

        twist = Twist()
        now = self.get_clock().now()
        log_interval = rclpy.duration.Duration(seconds=1.0)

        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        distance = math.sqrt(dx * dx + dy * dy)

        if self.phase == "forward":
            if distance < 1.0:
                twist.linear.x = 0.1
                if now - self.last_log_time > log_interval:
                    self.get_logger().info(
                        f"[前进阶段] dx={dx:.3f}, dy={dy:.3f}, 总距离={distance:.3f} m"
                    )
                    self.last_log_time = now
            else:
                twist.linear.x = 0.0
                self.phase = "rotate"
                self.get_logger().info("✅ 前进完成，开始旋转")

        elif self.phase == "rotate":
            delta_yaw = self.normalize_angle(self.current_yaw - self.start_yaw)
            angle_deg = math.degrees(delta_yaw)
            if abs(delta_yaw) < math.pi / 2:
                twist.angular.z = 0.6
                if now - self.last_log_time > log_interval:
                    self.get_logger().info(f"[旋转阶段] 当前角度变化: {angle_deg:.2f}°")
                    self.last_log_time = now
            else:
                twist.angular.z = 0.0
                self.phase = "stop"
                self.get_logger().info("✅ 旋转完成，运动结束")

        elif self.phase == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if now - self.last_log_time > log_interval:
                self.get_logger().info("🚩 已停止运动")
                self.last_log_time = now

        self.publisher.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = PointMove()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
