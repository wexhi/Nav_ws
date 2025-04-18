#!/usr/bin/env python3
import os, termios, tty, select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 速度步进和最大值
VX_STEP, WZ_STEP = 0.05, 0.3
VX_MAX, WZ_MAX = 0.2, 1.0


def getch(timeout=0.1):
    """
    从 /dev/tty 读一个字符，超时返回空串。
    如果读到 Ctrl‑C(\x03)，抛出 KeyboardInterrupt。
    """
    # 直接打开 /dev/tty，确保拿到一个真实的终端设备
    fd = os.open("/dev/tty", os.O_RDONLY)
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)  # 置为 raw 模式
        rlist, _, _ = select.select([fd], [], [], timeout)
        if rlist:
            ch = os.read(fd, 1).decode()
            if ch == "\x03":
                # 用户按下了 Ctrl‑C
                raise KeyboardInterrupt
            return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        os.close(fd)
    return ""


class WasdxTeleop(Node):
    def __init__(self):
        super().__init__("wasdx_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info(
            "WASDX Teleop 启动：w/s 前后累加，a/d 左右累加，x 停车，Ctrl‑C 退出"
        )
        self.vx = 0.0
        self.wz = 0.0
        self.create_timer(0.1, self.timer_cb)

    def timer_cb(self):
        try:
            key = getch()
        except KeyboardInterrupt:
            # 收到 Ctrl‑C，优雅退出
            self.get_logger().info("Detected Ctrl‑C, shutting down...")
            rclpy.shutdown()
            return

        if key == "w":
            self.vx = min(self.vx + VX_STEP, VX_MAX)
        elif key == "s":
            self.vx = max(self.vx - VX_STEP, -VX_MAX)
        elif key == "a":
            self.wz = min(self.wz - WZ_STEP, WZ_MAX)
        elif key == "d":
            self.wz = max(self.wz + WZ_STEP, -WZ_MAX)
        elif key == "x":
            self.vx = self.wz = 0.0

        if key in ["w", "s", "a", "d", "x"]:
            self.get_logger().info(f"按键[{key}] → vx={self.vx:.2f}, wz={self.wz:.2f}")

        # 发布速度
        msg = Twist()
        msg.linear.x = self.vx
        msg.angular.z = self.wz
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WasdxTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # 兜底：如果外部还能捕到的话
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
