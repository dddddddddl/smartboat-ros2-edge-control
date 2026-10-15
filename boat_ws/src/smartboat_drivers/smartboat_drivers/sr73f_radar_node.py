"""ROS 2 SocketCAN driver for the forward SR73F millimetre-wave radar."""

from collections import deque
import errno
import json
import os
import socket
import struct
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

from smartboat_msgs.msg import RadarStatus, RadarTarget, RadarTargetArray
from smartboat_drivers.sr73f_protocol import (
    RADAR_OBJECT_CAN_ID,
    RADAR_STATUS_CAN_ID,
    distance_from_bow,
    object_in_detection_window,
    parse_object_general,
    recent_hits_confirm_target,
)


CAN_EFF_FLAG = 0x80000000
CAN_EFF_MASK = 0x1FFFFFFF
CAN_SFF_MASK = 0x000007FF


class Sr73fRadarNode(Node):
    def __init__(self):
        super().__init__('sr73f_radar')

        self.declare_parameter('can_interface', '')
        self.declare_parameter('can_bitrate', 500000)
        self.declare_parameter('auto_bringup_interface', True)
        self.declare_parameter('minimum_distance_m', 3.0)
        self.declare_parameter('maximum_distance_m', 10.0)
        self.declare_parameter('field_of_view_deg', 112.0)
        self.declare_parameter('minimum_rcs', -64.0)
        self.declare_parameter('radar_to_bow_m', 0.45)
        self.declare_parameter('object_timeout_sec', 0.5)
        self.declare_parameter('minimum_confirmation_frames', 3)
        self.declare_parameter('confirmation_window_sec', 0.5)
        self.declare_parameter('scan_timeout_sec', 0.5)
        self.declare_parameter('reconnect_delay_sec', 2.0)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.can_interface = str(
            self.get_parameter('can_interface').value
        )
        self.can_bitrate = int(self.get_parameter('can_bitrate').value)
        self.auto_bringup = bool(
            self.get_parameter('auto_bringup_interface').value
        )
        self.minimum_distance_m = self._float_param('minimum_distance_m')
        self.maximum_distance_m = self._float_param('maximum_distance_m')
        self.field_of_view_deg = self._float_param('field_of_view_deg')
        self.minimum_rcs = self._float_param('minimum_rcs')
        self.radar_to_bow_m = self._float_param('radar_to_bow_m')
        self.object_timeout_sec = self._float_param('object_timeout_sec')
        self.minimum_confirmation_frames = int(
            self.get_parameter('minimum_confirmation_frames').value
        )
        self.confirmation_window_sec = self._float_param(
            'confirmation_window_sec'
        )
        self.scan_timeout_sec = self._float_param('scan_timeout_sec')
        self.reconnect_delay_sec = self._float_param('reconnect_delay_sec')

        if self.minimum_distance_m >= self.maximum_distance_m:
            raise ValueError(
                'minimum_distance_m must be below maximum_distance_m'
            )
        if self.minimum_confirmation_frames <= 0:
            raise ValueError('minimum_confirmation_frames must be positive')
        if self.confirmation_window_sec <= 0.0:
            raise ValueError('confirmation_window_sec must be positive')

        self.sock = None
        self.connected_interface = None
        self.last_connect_attempt = 0.0
        self.object_cache = {}
        self.object_hit_times = {}
        self.received_frames = 0
        self.received_status_frames = 0
        self.received_object_frames = 0
        self.last_frame_time = None

        self.closest_pub = self.create_publisher(
            Float32,
            '/smartboat/perception/radar/closest_obstacle_distance',
            10,
        )
        self.objects_pub = self.create_publisher(
            String,
            '/smartboat/perception/radar/objects',
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            '/smartboat/perception/radar/status',
            10,
        )
        self.targets_pub = self.create_publisher(
            RadarTargetArray,
            '/smartboat/perception/radar/targets',
            10,
        )
        self.typed_status_pub = self.create_publisher(
            RadarStatus,
            '/smartboat/perception/radar/typed_status',
            10,
        )

        rate_hz = self._float_param('publish_rate_hz')
        self.create_timer(1.0 / rate_hz, self._timer_callback)
        self.get_logger().info(
            f'SR73F radar node started | raw range='
            f'{self.minimum_distance_m:.1f}-{self.maximum_distance_m:.1f}m | '
            f'fov={self.field_of_view_deg:.1f}deg | '
            f'radar_to_bow={self.radar_to_bow_m:.2f}m'
        )

    def _float_param(self, name):
        return float(self.get_parameter(name).value)

    @staticmethod
    def _operstate(interface):
        path = os.path.join('/sys/class/net', interface, 'operstate')
        try:
            with open(path, 'r', encoding='ascii') as file:
                return file.read().strip().lower()
        except OSError:
            return ''

    @classmethod
    def _find_interfaces(cls):
        sys_net = '/sys/class/net'
        if not os.path.isdir(sys_net):
            return []
        candidates = []
        for name in sorted(os.listdir(sys_net)):
            if not name.startswith('can'):
                continue
            path = os.path.join(sys_net, name)
            target = os.path.realpath(path)
            score = 0
            if '/virtual/net/' in target:
                score += 20
            if name == 'can2':
                score += 10
            if cls._operstate(name) == 'up':
                score += 5
            candidates.append((score, name))
        return [name for _, name in sorted(candidates, reverse=True)]

    def _select_interface(self):
        if self.can_interface:
            return self.can_interface
        candidates = self._find_interfaces()
        if not candidates:
            raise RuntimeError('no SocketCAN interface found')
        return candidates[0]

    def _run_ip(self, *arguments):
        command = ['ip', *arguments]
        if os.geteuid() != 0:
            # The maintained container runs ROS as an unprivileged user but
            # grants passwordless sudo. CAP_NET_ADMIN is needed only for the
            # short interface setup commands, not for the radar node itself.
            command = ['sudo', '-n', *command]
        result = subprocess.run(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
        if result.returncode != 0:
            message = result.stderr.strip() or result.stdout.strip()
            raise RuntimeError(message or f'ip command failed: {arguments}')

    def _bring_up(self, interface):
        if self._operstate(interface) == 'up':
            return
        self._run_ip('link', 'set', interface, 'down')
        self._run_ip(
            'link',
            'set',
            interface,
            'type',
            'can',
            'bitrate',
            str(self.can_bitrate),
        )
        self._run_ip('link', 'set', interface, 'up')

    def _connect(self):
        now = time.monotonic()
        if now - self.last_connect_attempt < self.reconnect_delay_sec:
            return
        self.last_connect_attempt = now

        try:
            interface = self._select_interface()
            if self.auto_bringup:
                self._bring_up(interface)
            sock = socket.socket(
                socket.AF_CAN,
                socket.SOCK_RAW,
                socket.CAN_RAW,
            )
            can_filter = b''.join((
                struct.pack('=II', RADAR_STATUS_CAN_ID, CAN_SFF_MASK),
                struct.pack('=II', RADAR_OBJECT_CAN_ID, CAN_SFF_MASK),
            ))
            sock.setsockopt(
                socket.SOL_CAN_RAW,
                socket.CAN_RAW_FILTER,
                can_filter,
            )
            sock.bind((interface,))
            sock.setblocking(False)
        except (OSError, RuntimeError) as exc:
            self.get_logger().warning(f'SR73F connection failed: {exc}')
            return

        self.sock = sock
        self.connected_interface = interface
        self.get_logger().info(f'SR73F connected on {interface}')

    def _disconnect(self, reason):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self.connected_interface = None
        self.object_cache.clear()
        self.object_hit_times.clear()
        self.get_logger().warning(f'SR73F disconnected: {reason}')

    @staticmethod
    def _decode_can_frame(frame):
        if len(frame) < 16:
            return None, None
        can_id_raw, can_dlc, data = struct.unpack('=IB3x8s', frame[:16])
        if can_id_raw & CAN_EFF_FLAG:
            frame_id = can_id_raw & CAN_EFF_MASK
        else:
            frame_id = can_id_raw & CAN_SFF_MASK
        return frame_id, list(data[:can_dlc])

    def _read_available_frames(self):
        if self.sock is None:
            return
        while True:
            try:
                frame = self.sock.recv(16)
            except BlockingIOError:
                return
            except OSError as exc:
                if exc.errno in (errno.ENETDOWN, errno.ENODEV, errno.EBADF):
                    self._disconnect(exc)
                    return
                self._disconnect(exc)
                return

            frame_id, payload = self._decode_can_frame(frame)
            if frame_id not in (RADAR_STATUS_CAN_ID, RADAR_OBJECT_CAN_ID):
                continue
            obj = parse_object_general(frame_id, payload or [])
            self.received_frames += 1
            self.last_frame_time = time.monotonic()
            if frame_id == RADAR_STATUS_CAN_ID:
                self.received_status_frames += 1
                continue
            self.received_object_frames += 1
            if not object_in_detection_window(
                obj,
                minimum_distance_m=self.minimum_distance_m,
                maximum_distance_m=self.maximum_distance_m,
                field_of_view_deg=self.field_of_view_deg,
                minimum_rcs=self.minimum_rcs,
            ):
                if obj is not None:
                    self.object_cache.pop(obj['id'], None)
                    self.object_hit_times.pop(obj['id'], None)
                continue

            seen_at = time.monotonic()
            obj['distance_from_bow'] = distance_from_bow(
                obj,
                self.radar_to_bow_m,
            )
            obj['closing_speed'] = max(0.0, -obj['vrel_long'])
            if obj['closing_speed'] > 0.05:
                obj['ttc_sec'] = (
                    obj['distance_from_bow'] / obj['closing_speed']
                )
            else:
                obj['ttc_sec'] = None
            self.object_cache[obj['id']] = (obj, seen_at)
            hits = self.object_hit_times.setdefault(obj['id'], deque())
            hits.append(seen_at)
            cutoff = seen_at - self.confirmation_window_sec
            while hits and hits[0] < cutoff:
                hits.popleft()

    def _fresh_objects(self):
        now = time.monotonic()
        stale_ids = [
            object_id
            for object_id, (_, stamp) in self.object_cache.items()
            if now - stamp > self.object_timeout_sec
        ]
        for object_id in stale_ids:
            del self.object_cache[object_id]
            self.object_hit_times.pop(object_id, None)
        confirmed = []
        cutoff = now - self.confirmation_window_sec
        for object_id, (obj, _) in self.object_cache.items():
            hits = self.object_hit_times.get(object_id, ())
            while hits and hits[0] < cutoff:
                hits.popleft()
            if recent_hits_confirm_target(
                hits,
                now_sec=now,
                minimum_frames=self.minimum_confirmation_frames,
                confirmation_window_sec=self.confirmation_window_sec,
            ):
                confirmed.append(obj)
        return confirmed

    def _publish(self):
        now = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        data_fresh = (
            self.last_frame_time is not None
            and (now - self.last_frame_time) <= self.scan_timeout_sec
        )
        objects = sorted(
            self._fresh_objects(),
            key=lambda item: item['distance_from_bow'],
        )
        if data_fresh:
            closest = objects[0]['distance_from_bow'] if objects else -1.0
            self.closest_pub.publish(Float32(data=float(closest)))
            objects_json = json.dumps(objects, sort_keys=True)
            self.objects_pub.publish(String(data=objects_json))
        status = {
            'connected': self.sock is not None,
            'data_fresh': data_fresh,
            'interface': self.connected_interface,
            'raw_min_distance_m': self.minimum_distance_m,
            'raw_max_distance_m': self.maximum_distance_m,
            'minimum_confirmation_frames': (
                self.minimum_confirmation_frames
            ),
            'target_count': len(objects),
            'received_frames': self.received_frames,
            'received_status_frames': self.received_status_frames,
            'received_object_frames': self.received_object_frames,
        }
        status_json = json.dumps(status, sort_keys=True)
        self.status_pub.publish(String(data=status_json))

        targets_msg = RadarTargetArray()
        targets_msg.header.stamp = stamp
        targets_msg.header.frame_id = 'sr73f_radar'
        if data_fresh:
            for obj in objects:
                target = RadarTarget()
                target.id = int(obj['id'])
                target.longitudinal_m = float(obj['dist_long'])
                target.lateral_m = float(obj['dist_lat'])
                target.distance_m = float(obj['distance'])
                target.distance_from_bow_m = float(obj['distance_from_bow'])
                target.bearing_deg = float(obj['bearing_deg'])
                target.relative_longitudinal_mps = float(obj['vrel_long'])
                target.relative_lateral_mps = float(obj['vrel_lat'])
                target.rcs_db = float(obj['rcs'])
                target.closing_speed_mps = float(obj['closing_speed'])
                target.ttc_valid = obj['ttc_sec'] is not None
                target.ttc_sec = (
                    float(obj['ttc_sec']) if target.ttc_valid else 0.0
                )
                targets_msg.targets.append(target)
        self.targets_pub.publish(targets_msg)

        typed_status = RadarStatus()
        typed_status.header.stamp = stamp
        typed_status.header.frame_id = 'sr73f_radar'
        typed_status.connected = bool(status['connected'])
        typed_status.data_fresh = bool(status['data_fresh'])
        typed_status.interface = status['interface'] or ''
        typed_status.raw_min_distance_m = float(status['raw_min_distance_m'])
        typed_status.raw_max_distance_m = float(status['raw_max_distance_m'])
        typed_status.target_count = int(status['target_count'])
        typed_status.received_frames = int(status['received_frames'])
        typed_status.received_status_frames = int(
            status['received_status_frames']
        )
        typed_status.received_object_frames = int(
            status['received_object_frames']
        )
        self.typed_status_pub.publish(typed_status)

    def _timer_callback(self):
        if self.sock is None:
            self._connect()
        self._read_available_frames()
        self._publish()

    def destroy_node(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
            self.sock = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Sr73fRadarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            # ros2 launch forwards SIGINT to child processes while the node is
            # already unwinding from rclpy.spin(). Treat a second interrupt
            # during entity cleanup as a normal shutdown.
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
