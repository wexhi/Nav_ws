#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
from serial_bridge.crc_utils import CRC16_TABLE
import traceback
from datetime import datetime

CMD_HEADER = 0xA5
CMD_TAIL = 0x5A


def calc_crc16(data: bytes, init_crc=0xFFFF) -> int:
    crc = init_crc
    for i, b in enumerate(data):
        crc = ((crc >> 8) ^ CRC16_TABLE[(crc ^ b) & 0xFF]) & 0xFFFF
    return crc


class SerialCommander(Node):
    def __init__(self):
        super().__init__("serial_commander")

        # 声明参数
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)

        # 获取参数
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baudrate").get_parameter_value().integer_value

        try:
            self.get_logger().info(f"尝试打开串口: {port} @ {baud}...")
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info("✅ 串口打开成功")
        except Exception as e:
            self.get_logger().error(f"❌ 串口打开失败: {e}")
            self.get_logger().error(traceback.format_exc())
            raise e

        # 订阅 cmd_vel
        self.sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_cb, 10)
        self.get_logger().info("📡 SerialCommander 初始化完成，等待 cmd_vel 指令...")

    def cmd_vel_cb(self, msg: Twist):
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)

        self.get_logger().info(f"[{timestamp}] 接收到 cmd_vel: vx={vx:.3f}, wz={wz:.3f}")

        # 打包 payload
        payload = struct.pack("<ff", vx, wz)
        crc_data = bytes([CMD_HEADER]) + payload
        crc = calc_crc16(crc_data)

        # 组帧
        frame = (
            bytearray([CMD_HEADER]) +
            payload +
            struct.pack("<H", crc) +
            bytearray([CMD_TAIL])
        )

        # 打印帧的十六进制表示
        hex_str = ' '.join([f"{b:02X}" for b in frame])
        self.get_logger().debug(f"📦 发送帧内容 (hex): {hex_str}")
        self.get_logger().debug(f"🧮 CRC 校验值: 0x{crc:04X}")

        try:
            self.ser.write(frame)
            self.get_logger().info("🚀 指令已发送")
        except Exception as e:
            self.get_logger().error(f"❌ 串口写入失败: {e}")
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = SerialCommander()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 接收到 Ctrl+C，准备退出...")
    except Exception as e:
        node.get_logger().error(f"未捕获异常: {e}")
        node.get_logger().error(traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ ROS 节点关闭完成")


if __name__ == "__main__":
    main()
