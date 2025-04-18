#!/usr/bin/env python3
import sys, termios, tty, select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# 速度步进和最大值（可按需调整）
VX_STEP = 0.05  # 每按一次 w/s 改变 0.05 m/s
WZ_STEP = 0.1  # 每按一次 a/d 改变 0.1 rad/s
VX_MAX = 0.5  # 最大线速度
WZ_MAX = 1.5  # 最大角速度


def getch(timeout=0.1):
    """在终端上读一个字符，超时返回空串"""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([fd], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ""


class WasdxTeleop(Node):
    def __init__(self):
        super().__init__("wasdx_teleop")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.get_logger().info(
            "WASDX Teleop 启动：w/s 前后累加，a/d 左右累加，z 立即停止，Ctrl-C 退出"
        )
        # 当前目标速度
        self.vx = 0.0
        self.wz = 0.0
        # 定时发布
        self.create_timer(0.1, self.timer_cb)

    def timer_cb(self):
        key = getch()
        if key == "w":  # 前进加速
            self.vx = min(self.vx + VX_STEP, VX_MAX)
        elif key == "s":  # 后退加速（负向）
            self.vx = max(self.vx - VX_STEP, -VX_MAX)
        elif key == "a":  # 左转加速
            self.wz = min(self.wz + WZ_STEP, WZ_MAX)
        elif key == "d":  # 右转加速（负向）
            self.wz = max(self.wz - WZ_STEP, -WZ_MAX)
        elif key == "z":  # 立即停车
            self.vx = 0.0
            self.wz = 0.0

        # 打印当前目标
        if key in ["w", "s", "a", "d", "z"]:
            self.get_logger().info(
                f"按键[{key}] → vx={self.vx:.2f} m/s, wz={self.wz:.2f} rad/s"
            )

        # 发布 Twist
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
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
