#!/usr/bin/env python3
"""
Simple ROS 2 (Foxy) keyboard teleop publisher for /cmd_vel

Usage: source ROS 2 (and workspace) then run:
  python3 scripts/teleop_keyboard.py [--linear 0.2] [--angular 0.5]

Keys:
  w: forward
  s: backward
  a: turn left
  d: turn right
  x or SPACE: stop (zero velocities)
  q or Ctrl-C: quit

This script continually publishes the last commanded Twist at 10 Hz.
"""

import sys
import select
import termios
import tty
import threading
import argparse

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TeleopKeyboard(Node):
    def __init__(
        self, linear_speed: float, angular_speed: float, publish_hz: float = 10.0
    ):
        super().__init__("teleop_keyboard")
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed
        self.twist = Twist()
        self._lock = threading.Lock()

        timer_period = 1.0 / float(publish_hz)
        self.create_timer(timer_period, self._publish)

    def _publish(self):
        with self._lock:
            msg = Twist()
            msg.linear.x = float(self.twist.linear.x)
            msg.linear.y = float(self.twist.linear.y)
            msg.linear.z = float(self.twist.linear.z)
            msg.angular.x = float(self.twist.angular.x)
            msg.angular.y = float(self.twist.angular.y)
            msg.angular.z = float(self.twist.angular.z)
        self.pub.publish(msg)

    def set_command(self, linear: float, angular: float):
        with self._lock:
            self.twist.linear.x = float(linear)
            self.twist.angular.z = float(angular)


def print_instructions(linear, angular):
    msg = f"""
Keyboard Teleop

Controls:
  w: forward  (linear {linear} m/s)
  s: backward (linear -{linear} m/s)
  a: turn left  (angular {angular} rad/s)
  d: turn right (angular -{angular} rad/s)
  x or SPACE: stop
  q or Ctrl-C: quit

Publishing to: /cmd_vel
"""
    print(msg)


def getchar(timeout=0.1):
    """Read a single character from stdin with a timeout. Returns '' on timeout."""
    dr, _, _ = select.select([sys.stdin], [], [], timeout)
    if dr:
        return sys.stdin.read(1)
    return ""


def keyboard_thread_fn(node: TeleopKeyboard, linear_speed: float, angular_speed: float):
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())
        print_instructions(linear_speed, angular_speed)
        while rclpy.ok():
            c = getchar(0.1)
            if not c:
                continue
            c = c.lower()
            if c == "w":
                node.set_command(linear_speed, 0.0)
                node.get_logger().debug("cmd: forward")
            elif c == "s":
                node.set_command(-linear_speed, 0.0)
                node.get_logger().debug("cmd: backward")
            elif c == "a":
                node.set_command(0.0, angular_speed)
                node.get_logger().debug("cmd: left")
            elif c == "d":
                node.set_command(0.0, -angular_speed)
                node.get_logger().debug("cmd: right")
            elif c == "x" or c == " ":
                node.set_command(0.0, 0.0)
                node.get_logger().debug("cmd: stop")
            elif c == "q":
                node.get_logger().info("quitting")
                rclpy.shutdown()
                break
            else:
                # ignore other keys
                pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Keyboard teleop publisher for /cmd_vel"
    )
    parser.add_argument(
        "--linear", "-l", type=float, default=0.2, help="Linear speed (m/s)"
    )
    parser.add_argument(
        "--angular", "-a", type=float, default=0.5, help="Angular speed (rad/s)"
    )
    args = parser.parse_args(argv)

    rclpy.init()
    node = TeleopKeyboard(args.linear, args.angular)

    kb_thread = threading.Thread(
        target=keyboard_thread_fn, args=(node, args.linear, args.angular), daemon=True
    )
    kb_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("shutting down node")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
