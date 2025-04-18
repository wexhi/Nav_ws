#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
from serial_bridge.crc_utils import CRC16_TABLE  # 和你之前接收帧用同一张表

CMD_HEADER = 0xA5
CMD_TAIL = 0x5A


def calc_crc16(data: bytes, init_crc=0xFFFF) -> int:
    crc = init_crc
    for b in data:
        crc = ((crc >> 8) ^ CRC16_TABLE[(crc ^ b) & 0xFF]) & 0xFFFF
    return crc


class SerialCommander(Node):
    def __init__(self):
        super().__init__("serial_commander")

        # 1. 声明参数及默认值
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        # 2. 读取参数
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baudrate").get_parameter_value().integer_value

        # 3. 打日志并打开串口
        self.get_logger().info(f"Opening serial port {port} @{baud}")
        self.ser = serial.Serial(port, baud, timeout=0.1)

        # 订阅 cmd_vel
        self.sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_cb, 10)
        self.get_logger().info("SerialCommander 初始化完成")

    def cmd_vel_cb(self, msg: Twist):
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)

        # 打包 payload
        payload = struct.pack("<ff", vx, wz)
        crc = calc_crc16(bytes([CMD_HEADER]) + payload)

        # 组帧
        frame = (
            bytearray([CMD_HEADER])
            + payload
            + struct.pack("<H", crc)
            + bytearray([CMD_TAIL])
        )

        # 发送
        self.ser.write(frame)
        self.get_logger().debug(
            f"发出帧: header=0x{CMD_HEADER:X}, vx={vx}, wz={wz}, crc=0x{crc:X}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SerialCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
