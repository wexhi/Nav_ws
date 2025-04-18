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
        # 根据实际串口号和波特率打开
        self.ser = serial.Serial("/dev/ttyUSB0", 115200, timeout=0.1)
        self.sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_cb, 10)
        self.get_logger().info("SerialCommander 初始化完成")

    def cmd_vel_cb(self, msg: Twist):
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)
        # 按小端打包两个 float
        payload = struct.pack("<ff", vx, wz)
        # 计算 CRC（对 header 之后到 payload 末尾）
        crc = calc_crc16(bytes([CMD_HEADER]) + payload)
        frame = bytearray()
        frame.append(CMD_HEADER)
        frame += payload
        frame += struct.pack("<H", crc)
        frame.append(CMD_TAIL)
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
    node.destroy_node()
    rclpy.shutdown()
