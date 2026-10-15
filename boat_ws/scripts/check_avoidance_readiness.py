#!/usr/bin/env python3
"""Check the maintained recommendation-only avoidance data path."""

import json
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from smartboat_msgs.msg import FusionDetection, RadarStatus


class AvoidanceReadinessCheck(Node):
    """Observe formal typed inputs and recommendation-only outputs."""

    def __init__(self):
        super().__init__('check_avoidance_readiness')
        self.start_time = time.monotonic()
        self.done = False
        self.success = False
        self.samples = {}
        self.sample_times = {}

        self.create_subscription(
            FusionDetection,
            '/smartboat/perception/fusion/detection',
            self._store('fusion'),
            10,
        )
        self.create_subscription(
            RadarStatus,
            '/smartboat/perception/radar/typed_status',
            self._store('radar'),
            10,
        )
        self.create_subscription(
            Float32,
            '/smartboat/localization/yaw',
            self._store('yaw'),
            10,
        )
        self.create_subscription(
            Float32,
            '/smartboat/localization/ground_speed',
            self._store('ground_speed'),
            10,
        )
        self.create_subscription(
            String,
            '/smartboat/navigation/avoidance_supervisor_status',
            self._store('supervisor_status'),
            10,
        )
        self.create_subscription(
            Twist,
            '/smartboat/navigation/avoidance_cmd_vel',
            self._store('recommendation'),
            10,
        )
        self.create_timer(1.0, self.report)

    def _store(self, key):
        def callback(msg):
            self.samples[key] = msg
            self.sample_times[key] = time.monotonic()

        return callback

    def recent(self, key, now, timeout=2.0):
        timestamp = self.sample_times.get(key)
        return timestamp is not None and now - timestamp < timeout

    @staticmethod
    def _parse_status(msg):
        if msg is None:
            return {}
        try:
            parsed = json.loads(msg.data)
        except (AttributeError, TypeError, ValueError, json.JSONDecodeError):
            return {}
        return parsed if isinstance(parsed, dict) else {}

    def report(self):
        now = time.monotonic()
        fusion = self.samples.get('fusion')
        radar = self.samples.get('radar')
        yaw = self.samples.get('yaw')
        speed = self.samples.get('ground_speed')
        recommendation = self.samples.get('recommendation')
        status = self._parse_status(self.samples.get('supervisor_status'))

        fusion_ok = (
            self.recent('fusion', now)
            and fusion is not None
            and fusion.sensor_ready
        )
        radar_ok = (
            self.recent('radar', now)
            and radar is not None
            and radar.connected
            and radar.data_fresh
        )
        yaw_ok = self.recent('yaw', now)
        speed_ok = self.recent('ground_speed', now)
        status_ok = self.recent('supervisor_status', now)
        recommendation_ok = self.recent('recommendation', now)
        recommendation_only = status.get('recommendation_only') is True

        print('--- Smartboat recommendation readiness ---', flush=True)
        print(
            f'fusion_ok={fusion_ok} '
            f'sensor_ready={getattr(fusion, "sensor_ready", None)} '
            f'obstacle={getattr(fusion, "obstacle_detected", None)}',
            flush=True,
        )
        print(
            f'radar_ok={radar_ok} '
            f'connected={getattr(radar, "connected", None)} '
            f'data_fresh={getattr(radar, "data_fresh", None)}',
            flush=True,
        )
        print(
            f'localization_ok={yaw_ok and speed_ok} '
            f'yaw={getattr(yaw, "data", None)} '
            f'ground_speed={getattr(speed, "data", None)}',
            flush=True,
        )
        print(
            f'supervisor_ok={status_ok and recommendation_ok} '
            f'recommendation_only={recommendation_only} '
            f'state={status.get("state")} '
            f'linear={getattr(getattr(recommendation, "linear", None), "x", None)} '
            f'angular={getattr(getattr(recommendation, "angular", None), "z", None)}',
            flush=True,
        )

        if now - self.start_time < 12.0:
            return
        checks = (
            fusion_ok,
            radar_ok,
            yaw_ok,
            speed_ok,
            status_ok,
            recommendation_ok,
            recommendation_only,
        )
        self.success = all(checks)
        if self.success:
            print(
                'PASS: formal perception, localization, radar, and '
                'recommendation-only supervisor data are fresh.',
                flush=True,
            )
        else:
            print(
                'FAIL: one or more maintained recommendation inputs/outputs '
                'are missing, stale, or not ready.',
                flush=True,
            )
        self.done = True


def main():
    rclpy.init()
    node = AvoidanceReadinessCheck()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
        return 0 if node.success else 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
