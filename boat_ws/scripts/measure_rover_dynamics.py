#!/usr/bin/env python3
"""Measure smartboat turn and braking dynamics from ROS 2 telemetry.

Passive mode is the default: an operator drives with the RC transmitter while
the script detects and measures the manoeuvre. Active mode publishes Twist
commands only after an explicit confirmation token and several safety checks.
It never arms the vehicle or changes the PX4 flight mode.
"""

import argparse
import math
import statistics
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32, String


CONFIRMATION_TOKEN = "I_HAVE_RC_ESTOP"


def signed_angle_delta_deg(current, previous):
    return (current - previous + 180.0) % 360.0 - 180.0


class DynamicsTestNode(Node):
    def __init__(self, command_topic):
        super().__init__("measure_rover_dynamics")
        self.command_topic = command_topic
        self.yaw_deg = None
        self.ground_speed = None
        self.yaw_time = None
        self.speed_time = None
        self.bridge_status = None
        self.bridge_status_time = None

        self.create_subscription(
            Float32,
            "/smartboat/localization/yaw",
            self._yaw_callback,
            10,
        )
        self.create_subscription(
            Float32,
            "/smartboat/localization/ground_speed",
            self._speed_callback,
            10,
        )
        self.create_subscription(
            String,
            "/smartboat/px4_bridge/status",
            self._bridge_callback,
            10,
        )
        self.command_pub = self.create_publisher(Twist, command_topic, 10)

    def _yaw_callback(self, msg):
        self.yaw_deg = float(msg.data)
        self.yaw_time = time.monotonic()

    def _speed_callback(self, msg):
        self.ground_speed = max(0.0, float(msg.data))
        self.speed_time = time.monotonic()

    def _bridge_callback(self, msg):
        self.bridge_status = str(msg.data)
        self.bridge_status_time = time.monotonic()

    @staticmethod
    def _recent(stamp, timeout=1.0):
        return stamp is not None and (time.monotonic() - stamp) <= timeout

    def telemetry_fresh(self):
        return (
            self.yaw_deg is not None
            and self.ground_speed is not None
            and self._recent(self.yaw_time)
            and self._recent(self.speed_time)
        )

    def wait_for_telemetry(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.telemetry_fresh():
                return True
        return False

    def competing_command_publishers(self):
        names = {
            endpoint.node_name
            for endpoint in self.get_publishers_info_by_topic(self.command_topic)
        }
        names.discard(self.get_name())
        return sorted(names)

    def active_bridge_ready(self):
        if not self._recent(self.bridge_status_time, timeout=2.0):
            return False
        status = self.bridge_status or ""
        return "output_enabled=True" in status and "mode_allowed=True" in status

    def wait_for_active_bridge(self, timeout=3.0):
        """Publish neutral commands so the bridge emits a fresh status update."""
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.active_bridge_ready():
                return True
        return False

    def publish_command(self, speed, yaw_rate):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(yaw_rate)
        self.command_pub.publish(msg)

    def publish_neutral(self, duration=1.0, rate_hz=20.0):
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_command(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=1.0 / rate_hz)


def sample(node, previous_time, previous_yaw):
    rclpy.spin_once(node, timeout_sec=0.05)
    now = time.monotonic()
    if not node.telemetry_fresh():
        raise RuntimeError("yaw or ground-speed telemetry became stale")

    dt = now - previous_time
    delta_deg = signed_angle_delta_deg(node.yaw_deg, previous_yaw)
    yaw_rate = math.radians(delta_deg) / dt if dt > 1e-4 else 0.0
    return now, node.yaw_deg, node.ground_speed, delta_deg, yaw_rate


def wait_until_speed(node, target_speed, timeout, active_speed=None):
    print(f"Waiting for ground speed >= {target_speed:.2f} m/s ...", flush=True)
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        if active_speed is not None:
            node.publish_command(active_speed, 0.0)
        rclpy.spin_once(node, timeout_sec=0.05)
        if not node.telemetry_fresh():
            continue
        if node.ground_speed >= target_speed:
            return
    raise RuntimeError(f"did not reach {target_speed:.2f} m/s within {timeout:.1f}s")


def stop_with_reverse(node, reverse_speed, stop_threshold, timeout):
    start = time.monotonic()
    previous_time = start
    previous_speed = node.ground_speed
    distance = 0.0
    below_since = None

    while rclpy.ok() and (time.monotonic() - start) < timeout:
        node.publish_command(-abs(reverse_speed), 0.0)
        rclpy.spin_once(node, timeout_sec=0.05)
        if not node.telemetry_fresh():
            raise RuntimeError("telemetry became stale during active reverse braking")

        now = time.monotonic()
        dt = now - previous_time
        distance += 0.5 * (previous_speed + node.ground_speed) * dt
        previous_time = now
        previous_speed = node.ground_speed

        if node.ground_speed <= stop_threshold:
            if below_since is None:
                below_since = now
            elif now - below_since >= 0.3:
                node.publish_neutral()
                return now - start, distance
        else:
            below_since = None

    raise RuntimeError("active reverse braking did not stop the boat before timeout")


def run_turn_test(node, args):
    # PX4 yaw is expressed in NED: increasing yaw turns clockwise/right.
    measured_direction_sign = -1.0 if args.direction == "left" else 1.0
    positive_command_direction = args.positive_command_turns
    command_sign = 1.0 if args.direction == positive_command_direction else -1.0
    trigger_yaw_rate = math.radians(args.turn_trigger_deg_s)

    if args.active:
        wait_until_speed(
            node,
            args.test_speed * 0.9,
            args.acceleration_timeout,
            active_speed=args.test_speed,
        )
        print(
            f"Starting active {args.direction} turn: speed={args.test_speed:.2f} m/s, "
            f"yaw command={command_sign * args.yaw_rate_command:.2f} rad/s",
            flush=True,
        )
    else:
        print(
            f"RC operator: accelerate to about {args.test_speed:.2f} m/s, then hold a "
            f"steady {args.direction} turn until the boat turns {args.turn_angle_deg:.0f} deg.",
            flush=True,
        )

    deadline = time.monotonic() + args.test_timeout
    previous_time = time.monotonic()
    previous_yaw = node.yaw_deg
    started = args.active
    cumulative_angle = 0.0
    speeds = []
    yaw_rates = []
    started_at = None

    while rclpy.ok() and time.monotonic() < deadline:
        if args.active:
            node.publish_command(
                args.test_speed,
                command_sign * args.yaw_rate_command,
            )

        now, yaw, speed, delta_deg, yaw_rate = sample(
            node,
            previous_time,
            previous_yaw,
        )
        previous_time = now
        previous_yaw = yaw

        direction_matches = yaw_rate * measured_direction_sign > 0.0
        if not started:
            moving_fast_enough = speed >= args.test_speed * 0.7
            turning_fast_enough = abs(yaw_rate) >= trigger_yaw_rate
            if moving_fast_enough and direction_matches and turning_fast_enough:
                started = True

        if not started:
            continue
        if started_at is None:
            started_at = now

        cumulative_angle += delta_deg * measured_direction_sign
        if args.active and cumulative_angle <= -20.0:
            raise RuntimeError(
                "boat turned opposite to the requested direction; stop and correct "
                "--positive-command-turns before retrying"
            )
        if direction_matches and speed >= args.minimum_measure_speed:
            speeds.append(speed)
            yaw_rates.append(abs(yaw_rate))

        if cumulative_angle >= args.turn_angle_deg:
            break
    else:
        raise RuntimeError("turn test timed out before reaching the requested heading change")

    if not speeds or not yaw_rates:
        raise RuntimeError("not enough valid samples to calculate turn dynamics")

    average_speed = statistics.fmean(speeds)
    elapsed = time.monotonic() - started_at
    average_yaw_rate = math.radians(cumulative_angle) / elapsed
    radius = average_speed / average_yaw_rate if average_yaw_rate > 1e-3 else math.inf
    sorted_yaw_rates = sorted(yaw_rates)
    p95_index = min(len(sorted_yaw_rates) - 1, int(0.95 * len(sorted_yaw_rates)))
    representative_max_yaw_rate = sorted_yaw_rates[p95_index]

    if args.active:
        brake_time, brake_distance = stop_with_reverse(
            node,
            args.reverse_speed,
            args.stop_threshold,
            args.brake_timeout,
        )
        print(
            f"Active post-turn stop: time={brake_time:.2f}s, "
            f"estimated distance={brake_distance:.2f}m",
            flush=True,
        )

    print("\nTURN RESULT", flush=True)
    print(f"direction: {args.direction}", flush=True)
    print(f"heading change: {cumulative_angle:.1f} deg", flush=True)
    print(f"measurement time: {elapsed:.2f} s", flush=True)
    print(f"average speed: {average_speed:.3f} m/s", flush=True)
    print(f"average yaw rate: {math.degrees(average_yaw_rate):.2f} deg/s", flush=True)
    print(
        f"95th-percentile yaw rate: "
        f"{math.degrees(representative_max_yaw_rate):.2f} deg/s",
        flush=True,
    )
    print(f"estimated turn radius: {radius:.3f} m", flush=True)


def run_brake_test(node, args):
    if args.active:
        wait_until_speed(
            node,
            args.test_speed * 0.9,
            args.acceleration_timeout,
            active_speed=args.test_speed,
        )
        print("Target speed reached; applying active reverse braking.", flush=True)
        initial_speed = node.ground_speed
        brake_time, distance = stop_with_reverse(
            node,
            args.reverse_speed,
            args.stop_threshold,
            args.brake_timeout,
        )
    else:
        print(
            f"RC operator: accelerate to at least {args.test_speed:.2f} m/s. When the "
            "script announces target speed, immediately apply full reverse until stopped.",
            flush=True,
        )
        wait_until_speed(node, args.test_speed, args.acceleration_timeout)
        initial_speed = node.ground_speed
        print("TARGET REACHED -- APPLY FULL REVERSE NOW", flush=True)

        start = time.monotonic()
        previous_time = start
        previous_speed = initial_speed
        distance = 0.0
        below_since = None
        while rclpy.ok() and (time.monotonic() - start) < args.brake_timeout:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not node.telemetry_fresh():
                raise RuntimeError("telemetry became stale during braking measurement")
            now = time.monotonic()
            dt = now - previous_time
            distance += 0.5 * (previous_speed + node.ground_speed) * dt
            previous_time = now
            previous_speed = node.ground_speed
            if node.ground_speed <= args.stop_threshold:
                if below_since is None:
                    below_since = now
                elif now - below_since >= 0.3:
                    break
            else:
                below_since = None
        else:
            raise RuntimeError("braking test timed out before the boat stopped")
        brake_time = time.monotonic() - start

    deceleration = initial_speed / brake_time if brake_time > 1e-3 else math.inf
    print("\nBRAKE RESULT", flush=True)
    print(f"initial speed: {initial_speed:.3f} m/s", flush=True)
    print(f"time to stop: {brake_time:.3f} s", flush=True)
    print(f"estimated stopping distance: {distance:.3f} m", flush=True)
    print(f"average deceleration: {deceleration:.3f} m/s^2", flush=True)


def build_parser():
    parser = argparse.ArgumentParser(
        description="Measure smartboat turn radius, yaw rate, and stopping distance."
    )
    parser.add_argument("test", choices=("turn", "brake"))
    parser.add_argument("--direction", choices=("left", "right"), default="left")
    parser.add_argument("--test-speed", type=float, default=0.7)
    parser.add_argument("--turn-angle-deg", type=float, default=90.0)
    parser.add_argument("--turn-trigger-deg-s", type=float, default=5.0)
    parser.add_argument("--minimum-measure-speed", type=float, default=0.2)
    parser.add_argument("--yaw-rate-command", type=float, default=0.35)
    parser.add_argument(
        "--positive-command-turns",
        choices=("left", "right"),
        default="right",
        help=(
            "Physical direction produced by positive angular.z through the current "
            "bridge; verify at very low speed before active tests."
        ),
    )
    parser.add_argument("--reverse-speed", type=float, default=0.4)
    parser.add_argument("--stop-threshold", type=float, default=0.12)
    parser.add_argument("--acceleration-timeout", type=float, default=20.0)
    parser.add_argument("--test-timeout", type=float, default=25.0)
    parser.add_argument("--brake-timeout", type=float, default=8.0)
    parser.add_argument("--telemetry-timeout", type=float, default=8.0)
    parser.add_argument(
        "--command-topic",
        default="/smartboat/control/cmd_vel",
    )
    parser.add_argument(
        "--active",
        action="store_true",
        help="Publish commands instead of measuring RC-driven manoeuvres.",
    )
    parser.add_argument(
        "--confirm",
        default="",
        help=f"Active mode requires the exact token {CONFIRMATION_TOKEN}.",
    )
    return parser


def validate_args(args):
    if not 0.1 <= args.test_speed <= 1.5:
        raise ValueError("--test-speed must be between 0.1 and 1.5 m/s")
    if not 15.0 <= args.turn_angle_deg <= 180.0:
        raise ValueError("--turn-angle-deg must be between 15 and 180 degrees")
    if not 0.05 <= args.yaw_rate_command <= 0.7:
        raise ValueError("--yaw-rate-command must be between 0.05 and 0.7 rad/s")
    if args.active and args.confirm != CONFIRMATION_TOKEN:
        raise ValueError(
            f"active mode requires --confirm {CONFIRMATION_TOKEN}"
        )


def main():
    args = build_parser().parse_args()
    try:
        validate_args(args)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    rclpy.init()
    node = DynamicsTestNode(args.command_topic)
    active_commands_started = False
    try:
        if not node.wait_for_telemetry(args.telemetry_timeout):
            raise RuntimeError(
                "no fresh yaw and ground-speed telemetry; start yaw_from_mavlink first"
            )

        if args.active:
            competitors = node.competing_command_publishers()
            if competitors:
                raise RuntimeError(
                    "active test refused because command topic has other publishers: "
                    + ", ".join(competitors)
                )
            active_commands_started = True
            if not node.wait_for_active_bridge():
                raise RuntimeError(
                    "active test refused: PX4 bridge is stale, output is disabled, "
                    "or current mode is not allowed"
                )
            print(
                "ACTIVE WATER TEST ENABLED. Keep the RC emergency-stop control ready.",
                flush=True,
            )
            node.publish_neutral(duration=1.0)

        if args.test == "turn":
            run_turn_test(node, args)
        else:
            run_brake_test(node, args)
        return 0
    except (KeyboardInterrupt, RuntimeError) as exc:
        print(f"TEST ABORTED: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if active_commands_started:
            try:
                node.publish_neutral(duration=1.0)
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
