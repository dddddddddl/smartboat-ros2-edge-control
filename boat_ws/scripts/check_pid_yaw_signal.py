#!/usr/bin/env python3
"""Legacy archived-PID diagnostic; not used by the maintained edge launch."""

import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from std_msgs.msg import String


class PidYawSignalCheck(Node):
    def __init__(self):
        super().__init__('check_pid_yaw_signal')
        self.last_yaw = None
        self.last_yaw_time = None
        self.last_status = None
        self.last_status_time = None
        self.last_cmd = None
        self.last_cmd_time = None
        self.start_time = time.monotonic()
        self.done = False
        self.success = False

        self.create_subscription(
            Float32,
            '/smartboat/localization/yaw',
            self.yaw_callback,
            10,
        )
        self.create_subscription(
            String,
            '/smartboat/control/avoidance_status',
            self.status_callback,
            10,
        )
        self.create_subscription(
            Twist,
            '/smartboat/control/cmd_vel',
            self.cmd_callback,
            10,
        )
        self.create_timer(1.0, self.report)

    def yaw_callback(self, msg):
        self.last_yaw = float(msg.data)
        self.last_yaw_time = time.monotonic()

    def status_callback(self, msg):
        self.last_status = msg.data
        self.last_status_time = time.monotonic()

    def cmd_callback(self, msg):
        self.last_cmd = msg
        self.last_cmd_time = time.monotonic()

    @staticmethod
    def age(now, stamp):
        if stamp is None:
            return None
        return now - stamp

    def report(self):
        now = time.monotonic()
        yaw_age = self.age(now, self.last_yaw_time)
        status_age = self.age(now, self.last_status_time)
        cmd_age = self.age(now, self.last_cmd_time)

        yaw_ok = yaw_age is not None and yaw_age < 1.0
        status_ok = status_age is not None and status_age < 1.5
        cmd_ok = cmd_age is not None and cmd_age < 1.5

        cmd_text = 'none'
        if self.last_cmd is not None:
            cmd_text = (
                f'linear.x={self.last_cmd.linear.x:.3f}, '
                f'angular.z={self.last_cmd.angular.z:.3f}'
            )

        status_text = self.last_status or 'none'
        print(
            'PX4 yaw -> PID check | '
            f'yaw_ok={yaw_ok} yaw_deg={self.last_yaw} | '
            f'pid_status_ok={status_ok} | '
            f'cmd_vel_ok={cmd_ok} {cmd_text} | '
            f'status="{status_text}"',
            flush=True,
        )

        if now - self.start_time >= 10.0:
            if yaw_ok and status_ok and cmd_ok:
                print('PASS: avoidance_pid is receiving yaw and publishing cmd_vel.')
                self.success = True
            else:
                print('FAIL: yaw, PID status, or cmd_vel is missing/stale.')
                self.success = False
            self.done = True


def main():
    rclpy.init()
    node = PidYawSignalCheck()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        return 0 if node.success else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
