#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import serial
import struct
from collections import deque
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf2_ros
from math import cos, sin
from datetime import datetime
import threading
from tf_transformations import euler_from_quaternion  # ✅ 新增
from serial_bridge.crc_utils import CRC16_TABLE
import math

# —— Frame constants ——
HEADER = 0x5A
TAIL = 0xAA
FRAME_LEN = 40  # bytes
_STRUCT = struct.Struct(
    "<ff4ffff"
)  # real_vx, real_wz, quaternion (w,x,y,z), ax, ay, az

# —— CRC-16 table generation (poly 0x1021) ——
poly = 0x1021
for byte in range(256):
    crc = byte << 8
    for _ in range(8):
        if crc & 0x8000:
            crc = ((crc << 1) ^ poly) & 0xFFFF
        else:
            crc = (crc << 1) & 0xFFFF
    CRC16_TABLE.append(crc)


def crc16(data: bytes, init_crc: int = 0xFFFF) -> int:
    crc = init_crc
    for b in data:
        crc = ((crc >> 8) ^ CRC16_TABLE[(crc ^ b) & 0xFF]) & 0xFFFF
    return crc


def parse_frame(frame: bytes) -> dict:
    body = frame[1 : 1 + _STRUCT.size]
    real_vx, real_wz, w, x, y, z, ax, ay, az = _STRUCT.unpack(body)
    return {
        "timestamp": datetime.now().isoformat(timespec="milliseconds"),
        "real_vx": real_vx,
        "real_wz": real_wz,
        "q": (w, x, y, z),
        "ax": ax,
        "ay": ay,
        "az": az,
    }


def find_frames(stream: serial.Serial):
    buf = deque(maxlen=2 * FRAME_LEN)
    while True:
        data = stream.read(stream.in_waiting or 1)
        if data:
            buf.extend(data)
        while len(buf) >= FRAME_LEN:
            if buf[0] != HEADER:
                buf.popleft()
                continue
            candidate = bytes(buf.popleft() for _ in range(FRAME_LEN))
            if candidate[-1] != TAIL:
                buf.appendleft(candidate[1])
                continue
            if crc16(candidate[:-3]) != int.from_bytes(candidate[-3:-1], "little"):
                buf.appendleft(candidate[1])
                continue
            yield candidate


class SerialBridge(Node):
    def __init__(self):
        super().__init__("serial_bridge")
        self.declare_parameter(
            "port", "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
        )
        self.declare_parameter("baudrate", 115200)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baudrate").get_parameter_value().integer_value

        self.get_logger().info(f"Opening serial port {port} @{baud}")
        self.ser = serial.Serial(port, baud, timeout=0.02)

        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.imu_pub = self.create_publisher(Imu, "imu/data_raw", 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.x = self.y = self.yaw = 0.0
        self.prev_time = self.get_clock().now()
        self.last_transform = None

        threading.Thread(target=self._serial_loop, daemon=True).start()

    def _serial_loop(self):
        for frame in find_frames(self.ser):
            self._process_frame(frame)

    def _process_frame(self, frame: bytes):
        now = self.get_clock().now()
        dt = (now - self.prev_time).nanoseconds * 1e-9
        self.prev_time = now
        msg = parse_frame(frame)
        vx, wz = -msg["real_vx"], -msg["real_wz"]
        w, x, y, z = msg["q"]
        ax, ay, az = msg["ax"], msg["ay"], msg["az"]

        # ✅ 关键修改：用四元数提取 yaw 来积分位姿
        _, _, yaw = euler_from_quaternion((x, y, z, w))  # 注意顺序
        self.yaw = yaw
        self.x += vx * dt * cos(self.yaw)
        self.y += vx * dt * sin(self.yaw)

        # publish Odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = x
        odom.pose.pose.orientation.y = y
        odom.pose.pose.orientation.z = z
        odom.pose.pose.orientation.w = w
        odom.twist.twist.linear.x = vx
        odom.twist.twist.angular.z = wz
        self.odom_pub.publish(odom)

        # publish IMU
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = "imu_link"
        imu.orientation.x = x
        imu.orientation.y = y
        imu.orientation.z = z
        imu.orientation.w = w
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = ay
        imu.linear_acceleration.z = az
        self.imu_pub.publish(imu)

        # broadcast TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.x = x
        t.transform.rotation.y = y
        t.transform.rotation.z = z
        t.transform.rotation.w = w
        self.last_transform = t
        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(
            f"[ODOM] x={self.x:.3f}, y={self.y:.3f}, vx={vx:.3f}, dt={dt:.3f}, yaw={math.degrees(self.yaw):.1f}"
        )

    def _publish_tf(self):
        if self.last_transform:
            t = self.last_transform
            t.header.stamp = self.get_clock().now().to_msg()
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    node.create_timer(0.02, node._publish_tf)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
