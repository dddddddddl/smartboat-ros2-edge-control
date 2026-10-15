import math
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Bool, Float32

try:
    from pymavlink import mavutil
except ImportError:
    mavutil = None


MAV_AUTOPILOT_PX4 = 12
MAV_COMP_ID_AUTOPILOT1 = 1


def is_px4_autopilot_heartbeat(msg, target_system_id=0):
    if msg is None or msg.get_type() != 'HEARTBEAT':
        return False
    source_system = int(msg.get_srcSystem())
    if source_system <= 0:
        return False
    if target_system_id > 0 and source_system != int(target_system_id):
        return False
    if int(msg.get_srcComponent()) != MAV_COMP_ID_AUTOPILOT1:
        return False
    return int(getattr(msg, 'autopilot', -1)) == MAV_AUTOPILOT_PX4


def ground_speed_is_plausible(speed_mps, maximum_speed_mps):
    speed = float(speed_mps)
    return math.isfinite(speed) and 0.0 <= speed <= float(maximum_speed_mps)


def ground_speed_change_is_plausible(
    previous_speed_mps,
    speed_mps,
    elapsed_sec,
    maximum_change_mps_per_sec,
    jump_tolerance_mps,
):
    if previous_speed_mps is None:
        return True
    values = (
        previous_speed_mps,
        speed_mps,
        elapsed_sec,
        maximum_change_mps_per_sec,
        jump_tolerance_mps,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return False
    elapsed = max(0.0, float(elapsed_sec))
    allowed = (
        float(jump_tolerance_mps)
        + float(maximum_change_mps_per_sec) * elapsed
    )
    return abs(float(speed_mps) - float(previous_speed_mps)) <= allowed


class YawFromMavlinkNode(Node):
    def __init__(self):
        super().__init__('yaw_from_mavlink')

        self.declare_parameter('mavlink_url', 'tcp:127.0.0.1:5760')
        self.declare_parameter('target_system_id', 0)
        self.declare_parameter('output_yaw_topic', '/smartboat/localization/yaw')
        self.declare_parameter(
            'output_ground_speed_topic',
            '/smartboat/localization/ground_speed'
        )
        self.declare_parameter(
            'output_ground_speed_valid_topic',
            '/smartboat/localization/ground_speed_valid',
        )
        self.declare_parameter('output_rate_hz', 30.0)
        self.declare_parameter('publish_last_on_disconnect', True)
        self.declare_parameter('heartbeat_timeout_sec', 5.0)
        self.declare_parameter('recv_timeout_sec', 1.0)
        self.declare_parameter('attitude_timeout_sec', 3.0)
        self.declare_parameter('ground_speed_timeout_sec', 1.5)
        self.declare_parameter('require_gps_fix_for_ground_speed', True)
        self.declare_parameter('minimum_gps_fix_type', 3)
        self.declare_parameter('gps_fix_timeout_sec', 1.5)
        self.declare_parameter('maximum_ground_speed_mps', 5.0)
        self.declare_parameter(
            'maximum_ground_speed_change_mps_per_sec',
            2.0,
        )
        self.declare_parameter('ground_speed_jump_tolerance_mps', 0.25)
        self.declare_parameter('request_message_intervals', True)
        self.declare_parameter('requested_message_rate_hz', 10.0)
        self.declare_parameter('reconnect_delay_sec', 2.0)
        self.declare_parameter('log_period_sec', 1.0)

        self.mavlink_url = str(self.get_parameter('mavlink_url').value)
        self.target_system_id = int(
            self.get_parameter('target_system_id').value
        )
        self.output_yaw_topic = str(self.get_parameter('output_yaw_topic').value)
        self.output_ground_speed_topic = str(
            self.get_parameter('output_ground_speed_topic').value
        )
        self.output_ground_speed_valid_topic = str(
            self.get_parameter('output_ground_speed_valid_topic').value
        )
        self.output_rate_hz = float(self.get_parameter('output_rate_hz').value)
        self.publish_last_on_disconnect = bool(
            self.get_parameter('publish_last_on_disconnect').value
        )
        self.heartbeat_timeout_sec = float(
            self.get_parameter('heartbeat_timeout_sec').value
        )
        self.recv_timeout_sec = float(
            self.get_parameter('recv_timeout_sec').value
        )
        self.attitude_timeout_sec = float(
            self.get_parameter('attitude_timeout_sec').value
        )
        self.ground_speed_timeout_sec = float(
            self.get_parameter('ground_speed_timeout_sec').value
        )
        self.require_gps_fix_for_ground_speed = bool(
            self.get_parameter('require_gps_fix_for_ground_speed').value
        )
        self.minimum_gps_fix_type = int(
            self.get_parameter('minimum_gps_fix_type').value
        )
        self.gps_fix_timeout_sec = float(
            self.get_parameter('gps_fix_timeout_sec').value
        )
        self.maximum_ground_speed_mps = float(
            self.get_parameter('maximum_ground_speed_mps').value
        )
        self.maximum_ground_speed_change_mps_per_sec = float(
            self.get_parameter(
                'maximum_ground_speed_change_mps_per_sec'
            ).value
        )
        self.ground_speed_jump_tolerance_mps = float(
            self.get_parameter('ground_speed_jump_tolerance_mps').value
        )
        self.request_message_intervals = bool(
            self.get_parameter('request_message_intervals').value
        )
        self.requested_message_rate_hz = float(
            self.get_parameter('requested_message_rate_hz').value
        )
        self.reconnect_delay_sec = float(
            self.get_parameter('reconnect_delay_sec').value
        )
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)

        self.yaw_pub = self.create_publisher(
            Float32,
            self.output_yaw_topic,
            10
        )
        self.ground_speed_pub = self.create_publisher(
            Float32,
            self.output_ground_speed_topic,
            10
        )
        self.ground_speed_valid_pub = self.create_publisher(
            Bool,
            self.output_ground_speed_valid_topic,
            10,
        )

        if self.target_system_id < 0 or self.target_system_id > 255:
            raise ValueError('target_system_id must be in the range 0..255')
        positive_parameters = {
            'output_rate_hz': self.output_rate_hz,
            'heartbeat_timeout_sec': self.heartbeat_timeout_sec,
            'recv_timeout_sec': self.recv_timeout_sec,
            'attitude_timeout_sec': self.attitude_timeout_sec,
            'ground_speed_timeout_sec': self.ground_speed_timeout_sec,
            'gps_fix_timeout_sec': self.gps_fix_timeout_sec,
            'maximum_ground_speed_mps': self.maximum_ground_speed_mps,
            'maximum_ground_speed_change_mps_per_sec': (
                self.maximum_ground_speed_change_mps_per_sec
            ),
            'requested_message_rate_hz': self.requested_message_rate_hz,
            'reconnect_delay_sec': self.reconnect_delay_sec,
            'log_period_sec': self.log_period_sec,
        }
        for name, value in positive_parameters.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be finite and positive')
        if self.minimum_gps_fix_type < 0 or self.minimum_gps_fix_type > 8:
            raise ValueError('minimum_gps_fix_type must be in the range 0..8')
        if self.maximum_ground_speed_mps <= 0.0:
            raise ValueError('maximum_ground_speed_mps must be positive')
        if self.maximum_ground_speed_change_mps_per_sec <= 0.0:
            raise ValueError(
                'maximum_ground_speed_change_mps_per_sec must be positive'
            )
        if (
            not math.isfinite(self.ground_speed_jump_tolerance_mps)
            or self.ground_speed_jump_tolerance_mps < 0.0
        ):
            raise ValueError('ground_speed_jump_tolerance_mps cannot be negative')

        self._stop_event = threading.Event()
        self._lock = threading.Lock()
        self._connection = None
        self._connected = False
        self._target_system = 0
        self._target_component = 0
        self._last_yaw_deg = None
        self._last_yaw_rad = None
        self._last_attitude_time = None
        self._last_ground_speed = None
        self._last_ground_speed_time = None
        self._ground_speed_fault_latched = False
        self._ground_speed_sequence = 0
        self._published_ground_speed_sequence = -1
        self._gps_fix_type = 0
        self._last_gps_fix_time = None
        self._last_log_time = 0.0
        self._last_waiting_log_time = 0.0
        self._reader_thread = threading.Thread(
            target=self._reader_loop,
            name='yaw_from_mavlink_reader',
            daemon=True,
        )
        self._reader_thread.start()
        self.create_timer(
            1.0 / self.output_rate_hz,
            self._publish_cached_localization
        )

        self.get_logger().info(
            f'YawFromMavlink started | input={self.mavlink_url} | '
            f'output={self.output_yaw_topic} deg | '
            f'ground_speed={self.output_ground_speed_topic} m/s | '
            f'rate={self.output_rate_hz:.1f} Hz'
        )

    def destroy_node(self):
        self._stop_event.set()
        self._close_connection()
        if self._reader_thread.is_alive():
            self._reader_thread.join(timeout=2.0)
        super().destroy_node()

    def _close_connection(self):
        connection = self._connection
        self._connection = None
        with self._lock:
            self._connected = False
            self._target_system = 0
            self._target_component = 0
            self._gps_fix_type = 0
            self._last_gps_fix_time = None
            self._last_ground_speed = None
            self._last_ground_speed_time = None
            self._ground_speed_fault_latched = True
        if connection is None:
            return
        try:
            connection.close()
        except Exception:
            pass

    def _sleep_until_retry(self):
        deadline = time.monotonic() + self.reconnect_delay_sec
        while not self._stop_event.is_set() and time.monotonic() < deadline:
            time.sleep(0.1)

    def _connect(self):
        if mavutil is None:
            raise RuntimeError(
                'pymavlink is not installed; install python3-pymavlink '
                'inside the ROS2 container.'
            )

        connection = mavutil.mavlink_connection(
            self.mavlink_url,
            autoreconnect=False,
        )
        deadline = time.monotonic() + self.heartbeat_timeout_sec
        heartbeat = None
        while time.monotonic() < deadline and not self._stop_event.is_set():
            candidate = connection.recv_match(
                type='HEARTBEAT',
                blocking=True,
                timeout=0.5,
            )
            if is_px4_autopilot_heartbeat(candidate, self.target_system_id):
                heartbeat = candidate
                break
        if heartbeat is None:
            try:
                connection.close()
            except Exception:
                pass
            raise TimeoutError(
                f'no MAVLink HEARTBEAT from {self.mavlink_url} within '
                f'{self.heartbeat_timeout_sec:.1f}s'
            )

        target_system = int(heartbeat.get_srcSystem())
        target_component = int(heartbeat.get_srcComponent())
        connection.target_system = target_system
        connection.target_component = target_component

        self._connection = connection
        with self._lock:
            self._connected = True
            self._target_system = target_system
            self._target_component = target_component
        self.get_logger().info(
            f'MAVLink connected | input={self.mavlink_url} | '
            f'target_system={target_system} | '
            f'target_component={target_component}'
        )
        self._request_message_intervals(connection)
        return connection

    def _request_message_intervals(self, connection):
        if not self.request_message_intervals:
            return
        if self.requested_message_rate_hz <= 0.0:
            return

        interval_us = int(1_000_000.0 / self.requested_message_rate_hz)
        message_ids = [
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
        ]

        for message_id in message_ids:
            try:
                connection.mav.command_long_send(
                    connection.target_system,
                    connection.target_component,
                    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                    0,
                    message_id,
                    interval_us,
                    0,
                    0,
                    0,
                    0,
                    0,
                )
            except Exception as exc:
                self.get_logger().warning(
                    f'failed to request MAVLink message {message_id}: {exc}'
                )

    def _reader_loop(self):
        while not self._stop_event.is_set():
            try:
                connection = self._connect()
                self._read_localization_loop(connection)
            except Exception as exc:
                if not self._stop_event.is_set():
                    self.get_logger().warning(
                        f'MAVLink yaw reader disconnected: {exc}; '
                        f'retrying in {self.reconnect_delay_sec:.1f}s'
                    )
            finally:
                self._close_connection()

            self._sleep_until_retry()

    def _read_localization_loop(self, connection):
        while not self._stop_event.is_set():
            msg = connection.recv_match(
                type=[
                    'ATTITUDE',
                    'GLOBAL_POSITION_INT',
                    'GPS_RAW_INT',
                ],
                blocking=True,
                timeout=self.recv_timeout_sec,
            )
            if msg is None:
                last_attitude_time = self._last_attitude_time
                if (
                    last_attitude_time is not None
                    and (time.monotonic() - last_attitude_time)
                    > self.attitude_timeout_sec
                ):
                    raise TimeoutError(
                        'no ATTITUDE message for '
                        f'{self.attitude_timeout_sec:.1f}s'
                )
                continue

            with self._lock:
                target_system = self._target_system
            if int(msg.get_srcSystem()) != target_system:
                continue

            msg_type = msg.get_type()
            now = time.monotonic()

            if msg_type == 'GPS_RAW_INT':
                with self._lock:
                    self._gps_fix_type = int(msg.fix_type)
                    self._last_gps_fix_time = now
            elif msg_type == 'ATTITUDE':
                self._handle_attitude(msg, now)
            elif msg_type == 'GLOBAL_POSITION_INT':
                vx_mps = float(msg.vx) / 100.0
                vy_mps = float(msg.vy) / 100.0
                self._handle_ground_speed(math.hypot(vx_mps, vy_mps), now)

    def _handle_attitude(self, msg, now):
        yaw_rad = float(msg.yaw)
        yaw_deg = math.degrees(yaw_rad)

        with self._lock:
            self._last_yaw_deg = yaw_deg
            self._last_yaw_rad = yaw_rad
            self._last_attitude_time = now

        if now - self._last_log_time >= self.log_period_sec:
            self._last_log_time = now
            self.get_logger().info(
                f'received yaw={yaw_deg:.1f} deg '
                f'({yaw_rad:.3f} rad)'
            )

    def _handle_ground_speed(self, ground_speed, now):
        ground_speed = float(ground_speed)
        if not self._ground_speed_source_valid(now):
            return
        if not ground_speed_is_plausible(
            ground_speed,
            self.maximum_ground_speed_mps,
        ):
            with self._lock:
                self._ground_speed_fault_latched = True
            if now - self._last_waiting_log_time >= self.log_period_sec:
                self._last_waiting_log_time = now
                self.get_logger().warning(
                    f'ignored implausible ground_speed={ground_speed:.2f} m/s'
                )
            return
        with self._lock:
            previous_speed = self._last_ground_speed
            previous_time = self._last_ground_speed_time
        elapsed = (
            now - previous_time
            if previous_time is not None
            else 0.0
        )
        if not ground_speed_change_is_plausible(
            previous_speed,
            ground_speed,
            elapsed,
            self.maximum_ground_speed_change_mps_per_sec,
            self.ground_speed_jump_tolerance_mps,
        ):
            with self._lock:
                self._ground_speed_fault_latched = True
            if now - self._last_waiting_log_time >= self.log_period_sec:
                self._last_waiting_log_time = now
                self.get_logger().warning(
                    'ignored implausible ground-speed jump '
                    f'{previous_speed:.2f}->{ground_speed:.2f} m/s'
                )
            return
        with self._lock:
            self._last_ground_speed = ground_speed
            self._last_ground_speed_time = now
            self._ground_speed_fault_latched = False
            self._ground_speed_sequence += 1

        if now - self._last_log_time >= self.log_period_sec:
            self._last_log_time = now
            self.get_logger().info(
                f'received ground_speed={ground_speed:.2f} m/s'
            )

    def _ground_speed_source_valid(self, now):
        if not self.require_gps_fix_for_ground_speed:
            return True
        with self._lock:
            fix_type = self._gps_fix_type
            fix_time = self._last_gps_fix_time
        return (
            fix_type >= self.minimum_gps_fix_type
            and fix_time is not None
            and (now - fix_time) <= self.gps_fix_timeout_sec
        )

    def _publish_cached_localization(self):
        now = time.monotonic()
        with self._lock:
            yaw_deg = self._last_yaw_deg
            yaw_rad = self._last_yaw_rad
            ground_speed = self._last_ground_speed
            connected = self._connected
            last_attitude_time = self._last_attitude_time
            last_ground_speed_time = self._last_ground_speed_time
            ground_speed_sequence = self._ground_speed_sequence
            ground_speed_fault_latched = self._ground_speed_fault_latched

        if yaw_deg is None:
            if now - self._last_waiting_log_time >= self.log_period_sec:
                self._last_waiting_log_time = now
                self.get_logger().warning(
                    'waiting for first MAVLink ATTITUDE yaw'
                )
            return

        if (
            not self.publish_last_on_disconnect
            and last_attitude_time is not None
            and (now - last_attitude_time) > self.attitude_timeout_sec
        ):
            return

        out = Float32()
        out.data = float(yaw_deg)
        self.yaw_pub.publish(out)

        ground_speed_age = None
        speed_valid = False
        if ground_speed is not None and last_ground_speed_time is not None:
            ground_speed_age = now - last_ground_speed_time
            speed_valid = (
                ground_speed_age <= self.ground_speed_timeout_sec
                and self._ground_speed_source_valid(now)
                and not ground_speed_fault_latched
            )
            if (
                speed_valid
                and ground_speed_sequence != self._published_ground_speed_sequence
            ):
                speed_out = Float32()
                speed_out.data = float(ground_speed)
                self.ground_speed_pub.publish(speed_out)
                self._published_ground_speed_sequence = ground_speed_sequence
        self.ground_speed_valid_pub.publish(Bool(data=speed_valid))

        if now - self._last_log_time >= self.log_period_sec:
            self._last_log_time = now
            age = now - last_attitude_time if last_attitude_time else 0.0
            speed_text = 'none'
            if ground_speed is not None:
                speed_age = ground_speed_age if ground_speed_age is not None else -1.0
                speed_text = f'{ground_speed:.2f} m/s age={speed_age:.2f}s'
            self.get_logger().info(
                f'published yaw={yaw_deg:.1f} deg '
                f'({yaw_rad:.3f} rad) | connected={connected} | '
                f'age={age:.2f}s | ground_speed={speed_text}'
            )


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = YawFromMavlinkNode()
        rclpy.spin(node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass

    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
