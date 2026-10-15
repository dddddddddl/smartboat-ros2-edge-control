import fcntl
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from smartboat_px4_bridge.safety_event_recorder import SafetyEventRecorder

try:
    from pymavlink import mavparm, mavutil
except ImportError:
    mavparm = None
    mavutil = None


def clamp(value, min_value, max_value):
    if not all(math.isfinite(float(item)) for item in (
        value,
        min_value,
        max_value,
    )):
        raise ValueError('cannot clamp non-finite values')
    return max(min_value, min(max_value, value))


def acquire_exclusive_output_lock(path):
    """Prevent more than one hardware-output bridge in this container."""
    lock_path = str(path).strip()
    if not lock_path:
        raise ValueError('output_lock_path cannot be empty')
    lock_file = open(lock_path, 'a+', encoding='utf-8')
    try:
        fcntl.flock(lock_file.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError as exc:
        lock_file.close()
        raise RuntimeError(
            f'another PX4 hardware-output bridge holds {lock_path}'
        ) from exc
    lock_file.seek(0)
    lock_file.truncate()
    lock_file.write(f'{os.getpid()}\n')
    lock_file.flush()
    return lock_file


class SpeedController:
    """Conservative PI speed loop that never commands opposite thrust."""

    def __init__(
        self,
        *,
        kp,
        ki,
        integral_limit,
        maximum_effort,
        overspeed_tolerance_mps,
        target_deadband_mps,
    ):
        self.kp = float(kp)
        self.ki = float(ki)
        self.integral_limit = float(integral_limit)
        self.maximum_effort = float(maximum_effort)
        self.overspeed_tolerance_mps = float(overspeed_tolerance_mps)
        self.target_deadband_mps = float(target_deadband_mps)
        if self.kp < 0.0 or self.ki < 0.0:
            raise ValueError('speed controller gains cannot be negative')
        if self.integral_limit <= 0.0 or self.maximum_effort <= 0.0:
            raise ValueError('speed controller limits must be positive')
        if self.maximum_effort > 1.0:
            raise ValueError('maximum speed-controller effort cannot exceed 1')
        if self.overspeed_tolerance_mps < 0.0:
            raise ValueError('overspeed tolerance cannot be negative')
        if self.target_deadband_mps < 0.0:
            raise ValueError('target deadband cannot be negative')
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.last_time = None
        self.last_direction = 0.0

    def update(self, target_speed_mps, ground_speed_mps, now_sec):
        target = float(target_speed_mps)
        speed = float(ground_speed_mps)
        now = float(now_sec)
        if not all(math.isfinite(value) for value in (target, speed, now)):
            self.reset()
            return None
        if speed < 0.0:
            self.reset()
            return None
        if abs(target) <= self.target_deadband_mps:
            self.reset()
            return 0.0

        direction = 1.0 if target > 0.0 else -1.0
        if self.last_direction not in (0.0, direction):
            self.reset()
        dt = 0.0 if self.last_time is None else clamp(
            now - self.last_time,
            0.0,
            0.25,
        )
        self.last_time = now
        self.last_direction = direction

        desired = abs(target)
        if speed > desired + self.overspeed_tolerance_mps:
            self.integral = 0.0
            return 0.0

        error = desired - speed
        self.integral = clamp(
            self.integral + error * dt,
            0.0,
            self.integral_limit,
        )
        effort = clamp(
            self.kp * max(0.0, error) + self.ki * self.integral,
            0.0,
            self.maximum_effort,
        )
        return direction * effort


MAV_AUTOPILOT_PX4 = 12
MAV_COMP_ID_AUTOPILOT1 = 1
MAV_MODE_FLAG_SAFETY_ARMED = 128
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_FRAME_BODY_NED = 8
MAV_ARM_DISARM_FORCE_MAGIC = 21196.0
PX4_MAIN_MODE_MANUAL = 1
PX4_MAIN_MODE_AUTO = 4
PX4_MAIN_MODE_OFFBOARD = 6
PX4_AUTO_SUB_MODE_LOITER = 3
PX4_AUTO_SUB_MODE_MISSION = 4
SAFE_RC_INPUT_MODE = 2

# SET_POSITION_TARGET_LOCAL_NED: ignore position, acceleration/force, yaw and
# yaw-rate; use the body-frame vx/vy direction. PX4 differential/Ackermann
# Rover derives steering from the velocity vector and ignores yaw/yaw-rate.
OFFBOARD_VELOCITY_TYPE_MASK = (
    (1 << 0)
    | (1 << 1)
    | (1 << 2)
    | (1 << 6)
    | (1 << 7)
    | (1 << 8)
    | (1 << 10)
    | (1 << 11)
)


PX4_MAIN_MODES = {
    1: 'MANUAL',
    2: 'ALTCTL',
    3: 'POSCTL',
    4: 'AUTO',
    5: 'ACRO',
    6: 'OFFBOARD',
    7: 'STABILIZED',
    8: 'RATTITUDE',
    10: 'TERMINATION',
}

PX4_AUTO_SUB_MODES = {
    1: 'READY',
    2: 'TAKEOFF',
    3: 'LOITER',
    4: 'MISSION',
    5: 'RTL',
    6: 'LAND',
    7: 'RTGS',
    8: 'FOLLOW_TARGET',
    9: 'PRECLAND',
}


def decode_px4_custom_mode(custom_mode):
    main_mode = (int(custom_mode) >> 16) & 0xFF
    sub_mode = (int(custom_mode) >> 24) & 0xFF
    main_name = PX4_MAIN_MODES.get(main_mode, f'MAIN_{main_mode}')

    if main_name == 'AUTO':
        sub_name = PX4_AUTO_SUB_MODES.get(sub_mode, f'SUB_{sub_mode}')
        return f'AUTO.{sub_name}'

    return main_name


def mode_string_from_heartbeat(msg):
    if mavutil is not None:
        try:
            mode_name = mavutil.mode_string_px4(msg)
            normalized_mode_name = str(mode_name).strip()
            if (
                normalized_mode_name
                and normalized_mode_name.upper() != 'UNKNOWN'
                and not normalized_mode_name.startswith('Mode(')
            ):
                return normalized_mode_name
        except Exception:
            pass

    return decode_px4_custom_mode(getattr(msg, 'custom_mode', 0))


def is_px4_autopilot_heartbeat(msg, target_system_id=0):
    if msg is None or msg.get_type() != 'HEARTBEAT':
        return False

    source_system = int(msg.get_srcSystem())
    source_component = int(msg.get_srcComponent())
    if source_system <= 0:
        return False
    if target_system_id > 0 and source_system != int(target_system_id):
        return False
    if source_component != MAV_COMP_ID_AUTOPILOT1:
        return False
    return int(getattr(msg, 'autopilot', -1)) == MAV_AUTOPILOT_PX4


def heartbeat_is_armed(msg):
    base_mode = int(getattr(msg, 'base_mode', 0))
    return bool(base_mode & MAV_MODE_FLAG_SAFETY_ARMED)


def flight_mode_requests_emergency_kill(flight_mode):
    return str(flight_mode).upper() == 'TERMINATION'


def px4_kill_switch_state_from_statustext(text_value):
    """Parse PX4's explicit kill-switch state announcements."""
    normalized = ' '.join(
        str(text_value).strip().lower().replace('-', ' ').split()
    )
    if 'kill switch disengaged' in normalized:
        return False
    if 'kill switch engaged' in normalized:
        return True
    return None


def physical_kill_state_from_rc_pwm(
    *,
    current_engaged,
    pwm,
    trigger_pwm,
    release_pwm,
):
    """Return only RC Kill state transitions, with hysteresis."""
    value = int(pwm)
    if value == 65535 or value < 800 or value > 2200:
        return None
    if current_engaged:
        return False if value <= int(release_pwm) else None
    return True if value >= int(trigger_pwm) else None


def encode_px4_custom_mode(main_mode, sub_mode=0):
    """Encode PX4 main/sub modes for MAVLink SET_MODE."""
    return (int(main_mode) << 16) | (int(sub_mode) << 24)


def make_offboard_velocity_setpoint(
    *,
    linear_x,
    angular_z,
    maximum_linear_speed,
    maximum_yaw_rate,
    yaw_to_mavlink_sign,
    heading_lookahead_sec,
    maximum_heading_offset_rad,
):
    """Return bounded body-NED vx/vy for PX4 differential-rover steering."""
    values = (
        linear_x,
        angular_z,
        maximum_linear_speed,
        maximum_yaw_rate,
        yaw_to_mavlink_sign,
        heading_lookahead_sec,
        maximum_heading_offset_rad,
    )
    if not all(math.isfinite(float(value)) for value in values):
        return 0.0, 0.0
    if float(maximum_linear_speed) <= 0.0 or float(maximum_yaw_rate) <= 0.0:
        return 0.0, 0.0
    if float(yaw_to_mavlink_sign) not in (-1.0, 1.0):
        return 0.0, 0.0
    if (
        float(heading_lookahead_sec) <= 0.0
        or not 0.0 < float(maximum_heading_offset_rad) < (math.pi / 2.0)
    ):
        return 0.0, 0.0
    speed = clamp(
        float(linear_x),
        -float(maximum_linear_speed),
        float(maximum_linear_speed),
    )
    yaw_rate = clamp(
        float(angular_z),
        -float(maximum_yaw_rate),
        float(maximum_yaw_rate),
    ) * float(yaw_to_mavlink_sign)
    heading_offset = clamp(
        yaw_rate * float(heading_lookahead_sec),
        -float(maximum_heading_offset_rad),
        float(maximum_heading_offset_rad),
    )
    forward_speed = speed * math.cos(heading_offset)
    lateral_speed = abs(speed) * math.sin(heading_offset)
    return forward_speed, lateral_speed


def activation_timeout_requests_control(
    flight_mode,
    fail_closed_on_activation_timeout,
    manual_release_on_activation_timeout,
):
    if (
        str(flight_mode).upper() == 'MANUAL'
        and manual_release_on_activation_timeout
    ):
        return False
    return bool(fail_closed_on_activation_timeout)


def rc_channels_ready_for_handoff(
    *,
    now_sec,
    last_message_time_sec,
    rssi,
    channels,
    timeout_sec,
    minimum_rssi,
    throttle_channel,
    steering_channel,
    neutral_pwm,
    neutral_tolerance_pwm,
    allow_unknown_rssi=True,
    throttle_neutral_pwm=None,
    steering_neutral_pwm=None,
):
    """Require a fresh live RC link and neutral critical sticks."""
    if last_message_time_sec is None:
        return False
    if not all(math.isfinite(float(value)) for value in (
        now_sec,
        last_message_time_sec,
        timeout_sec,
    )):
        return False
    age = float(now_sec) - float(last_message_time_sec)
    if age < 0.0 or age > float(timeout_sec):
        return False
    # MAVLink reserves 255 for "RSSI unknown". The installed receiver reports
    # 255 even with a healthy full-strength link, so freshness and valid
    # channel values remain authoritative when unknown RSSI is allowed.
    rssi_value = int(rssi)
    if rssi_value == 255 and not bool(allow_unknown_rssi):
        return False
    if rssi_value != 255 and rssi_value < int(minimum_rssi):
        return False

    channel_centers = (
        (
            throttle_channel,
            neutral_pwm
            if throttle_neutral_pwm is None
            else throttle_neutral_pwm,
        ),
        (
            steering_channel,
            neutral_pwm
            if steering_neutral_pwm is None
            else steering_neutral_pwm,
        ),
    )
    for one_based_channel, channel_center in channel_centers:
        index = int(one_based_channel) - 1
        if index < 0 or index >= len(channels):
            return False
        pwm = int(channels[index])
        if pwm < 800 or pwm > 2200:
            return False
        if abs(pwm - int(channel_center)) > int(neutral_tolerance_pwm):
            return False
    return True


def offboard_command_requests_stop(
    cmd,
    command_active,
    linear_deadband_mps,
):
    """Return whether a fresh rover command requests zero XY velocity."""
    if not command_active or cmd is None:
        return False
    try:
        linear_x = float(cmd.linear.x)
        deadband = float(linear_deadband_mps)
    except (AttributeError, TypeError, ValueError):
        return False
    if not math.isfinite(linear_x) or not math.isfinite(deadband):
        return False
    if deadband < 0.0:
        return False
    # Differential-rover steering is encoded in the direction of the XY
    # velocity vector and therefore collapses to zero when linear.x is zero.
    return abs(linear_x) <= deadband


def make_manual_control_values(
    linear_x,
    angular_z,
    max_linear_speed,
    max_yaw_rate,
    throttle_center,
    throttle_scale,
    max_throttle_opening_percent,
    steering_scale,
    yaw_to_mavlink_sign,
    steering_mavlink_axis,
):
    numeric_values = (
        linear_x,
        angular_z,
        max_linear_speed,
        max_yaw_rate,
        throttle_center,
        throttle_scale,
        max_throttle_opening_percent,
        steering_scale,
        yaw_to_mavlink_sign,
    )
    if not all(math.isfinite(float(value)) for value in numeric_values):
        return 0, 0, int(throttle_center), 0
    linear = clamp(linear_x / max_linear_speed, -1.0, 1.0)
    yaw = clamp(angular_z / max_yaw_rate, -1.0, 1.0)

    opening_fraction = clamp(
        float(max_throttle_opening_percent) / 100.0,
        0.0,
        1.0,
    )
    minimum_throttle = throttle_center * (1.0 - opening_fraction)
    maximum_throttle = throttle_center + (
        (1000 - throttle_center) * opening_fraction
    )
    throttle = int(clamp(
        throttle_center + linear * throttle_scale,
        minimum_throttle,
        maximum_throttle,
    ))
    steering = int(clamp(
        yaw * steering_scale * yaw_to_mavlink_sign,
        -1000,
        1000,
    ))
    if steering_mavlink_axis == 'y':
        return 0, steering, throttle, 0
    return 0, 0, throttle, steering


class CmdVelToMavlinkNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_mavlink')

        self.declare_parameter('mavlink_url', 'tcp:127.0.0.1:5760')
        self.declare_parameter(
            'input_cmd_vel_topic',
            '/smartboat/control/cmd_vel',
        )
        self.declare_parameter(
            'activation_topic',
            '/smartboat/control/avoidance_active',
        )
        self.declare_parameter('status_topic', '/smartboat/px4_bridge/status')
        self.declare_parameter(
            'flight_mode_topic',
            '/smartboat/px4_bridge/flight_mode',
        )
        self.declare_parameter('armed_topic', '/smartboat/px4_bridge/armed')
        self.declare_parameter('output_enabled', False)
        self.declare_parameter(
            'output_lock_path',
            '/tmp/smartboat_px4_bridge_output.lock',
        )
        self.declare_parameter('target_system_id', 0)
        self.declare_parameter(
            'allowed_control_modes',
            ['AUTO.MISSION', 'AUTO.LOITER', 'MANUAL', 'OFFBOARD'],
        )
        self.declare_parameter('require_armed', True)
        self.declare_parameter('require_activation', True)
        self.declare_parameter('activation_timeout_sec', 0.8)
        self.declare_parameter('fail_closed_on_activation_timeout', True)
        self.declare_parameter(
            'manual_release_on_activation_timeout',
            True,
        )
        self.declare_parameter('source_switching_enabled', True)
        self.declare_parameter('mavlink_control_input_mode', 2)
        self.declare_parameter('restore_control_input_mode', 2)
        self.declare_parameter('source_switch_retry_sec', 1.0)
        self.declare_parameter('release_neutral_sec', 0.5)
        self.declare_parameter('manual_release_neutral_sec', 0.0)
        self.declare_parameter('offboard_warmup_sec', 1.1)
        self.declare_parameter('offboard_release_sec', 0.5)
        self.declare_parameter('offboard_mode_retry_sec', 1.0)
        self.declare_parameter('offboard_entry_timeout_sec', 4.0)
        self.declare_parameter('offboard_heading_lookahead_sec', 1.0)
        self.declare_parameter('offboard_max_heading_offset_rad', 0.7)
        self.declare_parameter('hold_before_offboard', True)
        self.declare_parameter('manual_avoidance_enabled', True)
        self.declare_parameter('mode_timeout_sec', 2.0)
        self.declare_parameter('command_rate_hz', 10.0)
        self.declare_parameter('heartbeat_timeout_sec', 5.0)
        self.declare_parameter('reconnect_delay_sec', 2.0)
        self.declare_parameter('cmd_timeout_sec', 0.5)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_yaw_rate', 0.7)
        self.declare_parameter('throttle_center', 500)
        self.declare_parameter('throttle_scale', 500)
        self.declare_parameter('max_throttle_opening_percent', 100.0)
        self.declare_parameter('steering_scale', 1000)
        self.declare_parameter('yaw_to_mavlink_sign', -1.0)
        self.declare_parameter('steering_mavlink_axis', 'y')
        self.declare_parameter('neutral_on_timeout', True)
        self.declare_parameter('speed_closed_loop_enabled', True)
        self.declare_parameter(
            'ground_speed_topic',
            '/smartboat/localization/ground_speed',
        )
        self.declare_parameter(
            'ground_speed_valid_topic',
            '/smartboat/localization/ground_speed_valid',
        )
        self.declare_parameter('ground_speed_input_timeout_sec', 0.5)
        self.declare_parameter('speed_controller_kp', 0.8)
        self.declare_parameter('speed_controller_ki', 0.25)
        self.declare_parameter('speed_integral_limit', 2.0)
        self.declare_parameter('speed_maximum_effort', 0.5)
        self.declare_parameter('speed_overspeed_tolerance_mps', 0.05)
        self.declare_parameter('speed_target_deadband_mps', 0.02)
        self.declare_parameter(
            'manual_fallback_topic',
            '/smartboat/control/manual_fallback_requested',
        )
        self.declare_parameter(
            'manual_handoff_complete_topic',
            '/smartboat/px4_bridge/manual_handoff_complete',
        )
        self.declare_parameter(
            'emergency_kill_topic',
            '/smartboat/control/emergency_kill',
        )
        self.declare_parameter('fallback_neutral_sec', 1.0)
        self.declare_parameter('fallback_mode_retry_sec', 1.0)
        self.declare_parameter('rc_channel_timeout_sec', 0.8)
        self.declare_parameter('rc_minimum_rssi', 1)
        self.declare_parameter('rc_throttle_channel', 3)
        self.declare_parameter('rc_steering_channel', 1)
        self.declare_parameter('rc_neutral_pwm', 1500)
        self.declare_parameter('rc_throttle_neutral_pwm', 1500)
        self.declare_parameter('rc_steering_neutral_pwm', 1500)
        self.declare_parameter('rc_neutral_tolerance_pwm', 100)
        self.declare_parameter('rc_takeover_threshold_pwm', 80)
        self.declare_parameter('rc_entry_neutral_sec', 0.3)
        self.declare_parameter('allow_unknown_rc_rssi', True)
        self.declare_parameter('stop_speed_threshold_mps', 0.2)
        self.declare_parameter('stop_command_deadband_mps', 0.02)
        self.declare_parameter('physical_kill_channel', 0)
        self.declare_parameter('physical_kill_trigger_pwm', 1850)
        self.declare_parameter('physical_kill_release_pwm', 1700)
        self.declare_parameter('event_log_enabled', True)
        self.declare_parameter(
            'event_log_path',
            '/workspace/boat_ws/log/smartboat_px4_bridge_events.jsonl',
        )
        self.declare_parameter('log_period_sec', 1.0)

        self.mavlink_url = str(self.get_parameter('mavlink_url').value)
        self.input_cmd_vel_topic = str(
            self.get_parameter('input_cmd_vel_topic').value
        )
        self.activation_topic = str(
            self.get_parameter('activation_topic').value
        )
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.flight_mode_topic = str(
            self.get_parameter('flight_mode_topic').value
        )
        self.armed_topic = str(self.get_parameter('armed_topic').value)
        self.output_enabled = bool(self.get_parameter('output_enabled').value)
        self.output_lock_path = str(
            self.get_parameter('output_lock_path').value
        )
        self.target_system_id = int(
            self.get_parameter('target_system_id').value
        )
        self.allowed_control_modes = set(
            str(mode).upper()
            for mode in self.get_parameter('allowed_control_modes').value
        )
        self.require_armed = bool(self.get_parameter('require_armed').value)
        self.require_activation = bool(
            self.get_parameter('require_activation').value
        )
        self.activation_timeout_sec = float(
            self.get_parameter('activation_timeout_sec').value
        )
        self.fail_closed_on_activation_timeout = bool(
            self.get_parameter('fail_closed_on_activation_timeout').value
        )
        self.manual_release_on_activation_timeout = bool(
            self.get_parameter('manual_release_on_activation_timeout').value
        )
        self.source_switching_enabled = bool(
            self.get_parameter('source_switching_enabled').value
        )
        self.mavlink_control_input_mode = int(
            self.get_parameter('mavlink_control_input_mode').value
        )
        self.restore_control_input_mode = int(
            self.get_parameter('restore_control_input_mode').value
        )
        self.source_switch_retry_sec = float(
            self.get_parameter('source_switch_retry_sec').value
        )
        self.release_neutral_sec = float(
            self.get_parameter('release_neutral_sec').value
        )
        self.manual_release_neutral_sec = float(
            self.get_parameter('manual_release_neutral_sec').value
        )
        self.offboard_warmup_sec = float(
            self.get_parameter('offboard_warmup_sec').value
        )
        self.offboard_release_sec = float(
            self.get_parameter('offboard_release_sec').value
        )
        self.offboard_mode_retry_sec = float(
            self.get_parameter('offboard_mode_retry_sec').value
        )
        self.offboard_entry_timeout_sec = float(
            self.get_parameter('offboard_entry_timeout_sec').value
        )
        self.offboard_heading_lookahead_sec = float(
            self.get_parameter('offboard_heading_lookahead_sec').value
        )
        self.offboard_max_heading_offset_rad = float(
            self.get_parameter('offboard_max_heading_offset_rad').value
        )
        self.hold_before_offboard = bool(
            self.get_parameter('hold_before_offboard').value
        )
        self.manual_avoidance_enabled = bool(
            self.get_parameter('manual_avoidance_enabled').value
        )
        self.mode_timeout_sec = float(
            self.get_parameter('mode_timeout_sec').value
        )
        self.command_rate_hz = float(
            self.get_parameter('command_rate_hz').value
        )
        self.heartbeat_timeout_sec = float(
            self.get_parameter('heartbeat_timeout_sec').value
        )
        self.reconnect_delay_sec = float(
            self.get_parameter('reconnect_delay_sec').value
        )
        self.cmd_timeout_sec = float(
            self.get_parameter('cmd_timeout_sec').value
        )
        self.max_linear_speed = float(
            self.get_parameter('max_linear_speed').value
        )
        self.max_yaw_rate = float(self.get_parameter('max_yaw_rate').value)
        self.throttle_center = int(self.get_parameter('throttle_center').value)
        self.throttle_scale = int(self.get_parameter('throttle_scale').value)
        self.max_throttle_opening_percent = float(
            self.get_parameter('max_throttle_opening_percent').value
        )
        self.steering_scale = int(self.get_parameter('steering_scale').value)
        self.yaw_to_mavlink_sign = float(
            self.get_parameter('yaw_to_mavlink_sign').value
        )
        self.steering_mavlink_axis = str(
            self.get_parameter('steering_mavlink_axis').value
        ).lower()
        self.neutral_on_timeout = bool(
            self.get_parameter('neutral_on_timeout').value
        )
        self.speed_closed_loop_enabled = bool(
            self.get_parameter('speed_closed_loop_enabled').value
        )
        self.ground_speed_topic = str(
            self.get_parameter('ground_speed_topic').value
        )
        self.ground_speed_valid_topic = str(
            self.get_parameter('ground_speed_valid_topic').value
        )
        self.ground_speed_input_timeout_sec = float(
            self.get_parameter('ground_speed_input_timeout_sec').value
        )
        self.manual_fallback_topic = str(
            self.get_parameter('manual_fallback_topic').value
        )
        self.manual_handoff_complete_topic = str(
            self.get_parameter('manual_handoff_complete_topic').value
        )
        self.emergency_kill_topic = str(
            self.get_parameter('emergency_kill_topic').value
        )
        self.fallback_neutral_sec = float(
            self.get_parameter('fallback_neutral_sec').value
        )
        self.fallback_mode_retry_sec = float(
            self.get_parameter('fallback_mode_retry_sec').value
        )
        self.rc_channel_timeout_sec = float(
            self.get_parameter('rc_channel_timeout_sec').value
        )
        self.rc_minimum_rssi = int(
            self.get_parameter('rc_minimum_rssi').value
        )
        self.rc_throttle_channel = int(
            self.get_parameter('rc_throttle_channel').value
        )
        self.rc_steering_channel = int(
            self.get_parameter('rc_steering_channel').value
        )
        self.rc_neutral_pwm = int(self.get_parameter('rc_neutral_pwm').value)
        self.rc_throttle_neutral_pwm = int(
            self.get_parameter('rc_throttle_neutral_pwm').value
        )
        self.rc_steering_neutral_pwm = int(
            self.get_parameter('rc_steering_neutral_pwm').value
        )
        self.rc_neutral_tolerance_pwm = int(
            self.get_parameter('rc_neutral_tolerance_pwm').value
        )
        self.rc_takeover_threshold_pwm = int(
            self.get_parameter('rc_takeover_threshold_pwm').value
        )
        self.rc_entry_neutral_sec = float(
            self.get_parameter('rc_entry_neutral_sec').value
        )
        self.allow_unknown_rc_rssi = bool(
            self.get_parameter('allow_unknown_rc_rssi').value
        )
        self.stop_speed_threshold_mps = float(
            self.get_parameter('stop_speed_threshold_mps').value
        )
        self.stop_command_deadband_mps = float(
            self.get_parameter('stop_command_deadband_mps').value
        )
        self.physical_kill_channel = int(
            self.get_parameter('physical_kill_channel').value
        )
        self.physical_kill_trigger_pwm = int(
            self.get_parameter('physical_kill_trigger_pwm').value
        )
        self.physical_kill_release_pwm = int(
            self.get_parameter('physical_kill_release_pwm').value
        )
        self.event_log_enabled = bool(
            self.get_parameter('event_log_enabled').value
        )
        self.event_log_path = str(
            self.get_parameter('event_log_path').value
        )
        self.log_period_sec = float(self.get_parameter('log_period_sec').value)

        self.speed_controller = SpeedController(
            kp=float(self.get_parameter('speed_controller_kp').value),
            ki=float(self.get_parameter('speed_controller_ki').value),
            integral_limit=float(
                self.get_parameter('speed_integral_limit').value
            ),
            maximum_effort=float(
                self.get_parameter('speed_maximum_effort').value
            ),
            overspeed_tolerance_mps=float(
                self.get_parameter('speed_overspeed_tolerance_mps').value
            ),
            target_deadband_mps=float(
                self.get_parameter('speed_target_deadband_mps').value
            ),
        )

        if self.target_system_id < 0 or self.target_system_id > 255:
            raise ValueError('target_system_id must be in the range 0..255')
        finite_parameters = {
            'activation_timeout_sec': self.activation_timeout_sec,
            'source_switch_retry_sec': self.source_switch_retry_sec,
            'release_neutral_sec': self.release_neutral_sec,
            'manual_release_neutral_sec': self.manual_release_neutral_sec,
            'offboard_warmup_sec': self.offboard_warmup_sec,
            'offboard_release_sec': self.offboard_release_sec,
            'offboard_mode_retry_sec': self.offboard_mode_retry_sec,
            'offboard_entry_timeout_sec': self.offboard_entry_timeout_sec,
            'offboard_heading_lookahead_sec': (
                self.offboard_heading_lookahead_sec
            ),
            'offboard_max_heading_offset_rad': (
                self.offboard_max_heading_offset_rad
            ),
            'mode_timeout_sec': self.mode_timeout_sec,
            'command_rate_hz': self.command_rate_hz,
            'heartbeat_timeout_sec': self.heartbeat_timeout_sec,
            'reconnect_delay_sec': self.reconnect_delay_sec,
            'cmd_timeout_sec': self.cmd_timeout_sec,
            'max_linear_speed': self.max_linear_speed,
            'max_yaw_rate': self.max_yaw_rate,
            'max_throttle_opening_percent': (
                self.max_throttle_opening_percent
            ),
            'yaw_to_mavlink_sign': self.yaw_to_mavlink_sign,
            'ground_speed_input_timeout_sec': (
                self.ground_speed_input_timeout_sec
            ),
            'fallback_neutral_sec': self.fallback_neutral_sec,
            'fallback_mode_retry_sec': self.fallback_mode_retry_sec,
            'rc_channel_timeout_sec': self.rc_channel_timeout_sec,
            'rc_entry_neutral_sec': self.rc_entry_neutral_sec,
            'stop_speed_threshold_mps': self.stop_speed_threshold_mps,
            'stop_command_deadband_mps': self.stop_command_deadband_mps,
            'log_period_sec': self.log_period_sec,
        }
        for name, value in finite_parameters.items():
            if not math.isfinite(float(value)):
                raise ValueError(f'{name} must be finite')
        for name, value in (
            ('mode_timeout_sec', self.mode_timeout_sec),
            ('command_rate_hz', self.command_rate_hz),
            ('heartbeat_timeout_sec', self.heartbeat_timeout_sec),
            ('reconnect_delay_sec', self.reconnect_delay_sec),
            ('cmd_timeout_sec', self.cmd_timeout_sec),
            ('log_period_sec', self.log_period_sec),
        ):
            if value <= 0.0:
                raise ValueError(f'{name} must be greater than zero')
        if self.max_linear_speed <= 0.0:
            raise ValueError('max_linear_speed must be greater than zero')
        if self.max_yaw_rate <= 0.0:
            raise ValueError('max_yaw_rate must be greater than zero')
        if not 0.0 < self.max_throttle_opening_percent <= 100.0:
            raise ValueError(
                'max_throttle_opening_percent must be in the range (0, 100]'
            )
        if self.yaw_to_mavlink_sign not in (-1.0, 1.0):
            raise ValueError('yaw_to_mavlink_sign must be -1.0 or 1.0')
        if self.steering_mavlink_axis not in ('y', 'r'):
            raise ValueError('steering_mavlink_axis must be "y" or "r"')
        if self.activation_timeout_sec <= 0.0:
            raise ValueError(
                'activation_timeout_sec must be greater than zero'
            )
        if self.source_switch_retry_sec <= 0.0:
            raise ValueError(
                'source_switch_retry_sec must be greater than zero'
            )
        if self.release_neutral_sec < 0.0:
            raise ValueError('release_neutral_sec cannot be negative')
        if self.manual_release_neutral_sec < 0.0:
            raise ValueError('manual_release_neutral_sec cannot be negative')
        if self.offboard_warmup_sec < 1.0:
            raise ValueError('offboard_warmup_sec must be at least 1 second')
        if self.offboard_release_sec < 0.0:
            raise ValueError('offboard_release_sec cannot be negative')
        if self.offboard_mode_retry_sec <= 0.0:
            raise ValueError('offboard_mode_retry_sec must be positive')
        if self.offboard_entry_timeout_sec <= self.offboard_warmup_sec:
            raise ValueError(
                'offboard_entry_timeout_sec must exceed offboard_warmup_sec'
            )
        if self.offboard_heading_lookahead_sec <= 0.0:
            raise ValueError('offboard_heading_lookahead_sec must be positive')
        if not 0.0 < self.offboard_max_heading_offset_rad < (math.pi / 2.0):
            raise ValueError(
                'offboard_max_heading_offset_rad must be between 0 and pi/2'
            )
        if self.ground_speed_input_timeout_sec <= 0.0:
            raise ValueError('ground_speed_input_timeout_sec must be positive')
        if self.fallback_neutral_sec < 0.0:
            raise ValueError('fallback_neutral_sec cannot be negative')
        if self.fallback_mode_retry_sec <= 0.0:
            raise ValueError('fallback_mode_retry_sec must be positive')
        if self.rc_channel_timeout_sec <= 0.0:
            raise ValueError('rc_channel_timeout_sec must be positive')
        for name, channel in (
            ('rc_throttle_channel', self.rc_throttle_channel),
            ('rc_steering_channel', self.rc_steering_channel),
        ):
            if channel < 1 or channel > 18:
                raise ValueError(f'{name} must be in the range 1..18')
        if (
            self.physical_kill_channel < 0
            or self.physical_kill_channel > 18
        ):
            raise ValueError(
                'physical_kill_channel must be in the range 0..18'
            )
        for name, pwm in (
            ('physical_kill_trigger_pwm', self.physical_kill_trigger_pwm),
            ('physical_kill_release_pwm', self.physical_kill_release_pwm),
        ):
            if pwm < 800 or pwm > 2200:
                raise ValueError(f'{name} must be in the range 800..2200')
        if self.physical_kill_release_pwm >= self.physical_kill_trigger_pwm:
            raise ValueError(
                'physical_kill_release_pwm must be below '
                'physical_kill_trigger_pwm'
            )
        if self.rc_neutral_tolerance_pwm < 0:
            raise ValueError('rc_neutral_tolerance_pwm cannot be negative')
        if self.rc_takeover_threshold_pwm <= 0:
            raise ValueError('rc_takeover_threshold_pwm must be positive')
        if self.rc_takeover_threshold_pwm > self.rc_neutral_tolerance_pwm:
            raise ValueError(
                'rc_takeover_threshold_pwm cannot exceed '
                'rc_neutral_tolerance_pwm'
            )
        if self.rc_entry_neutral_sec < 0.0:
            raise ValueError('rc_entry_neutral_sec cannot be negative')
        if self.stop_speed_threshold_mps <= 0.0:
            raise ValueError('stop_speed_threshold_mps must be positive')
        if self.stop_command_deadband_mps < 0.0:
            raise ValueError('stop_command_deadband_mps cannot be negative')
        if self.rc_minimum_rssi < 0 or self.rc_minimum_rssi > 254:
            raise ValueError('rc_minimum_rssi must be in the range 0..254')
        for name, pwm in (
            ('rc_neutral_pwm', self.rc_neutral_pwm),
            ('rc_throttle_neutral_pwm', self.rc_throttle_neutral_pwm),
            ('rc_steering_neutral_pwm', self.rc_steering_neutral_pwm),
        ):
            if pwm < 800 or pwm > 2200:
                raise ValueError(f'{name} must be in the range 800..2200')
        if self.throttle_center < 0 or self.throttle_center > 1000:
            raise ValueError('throttle_center must be in the range 0..1000')
        if self.throttle_scale <= 0 or self.steering_scale <= 0:
            raise ValueError('throttle and steering scales must be positive')
        for name, value in (
            ('mavlink_control_input_mode', self.mavlink_control_input_mode),
            ('restore_control_input_mode', self.restore_control_input_mode),
        ):
            if value != SAFE_RC_INPUT_MODE:
                raise ValueError(
                    f'{name} must remain {SAFE_RC_INPUT_MODE}; '
                    'the bridge is not allowed to disable physical RC input'
                )

        self._lock = threading.Lock()
        self._output_lock_file = None
        if self.output_enabled:
            self._output_lock_file = acquire_exclusive_output_lock(
                self.output_lock_path
            )
        self._stop_event = threading.Event()
        self._connection = None
        self._connected = False
        self._target_system = 0
        self._target_component = 0
        self._flight_mode = 'UNKNOWN'
        self._armed = False
        self._last_heartbeat_time = None
        self._last_cmd = None
        self._last_cmd_time = None
        self._avoidance_active = False
        self._last_activation_time = None
        self._ground_speed = 0.0
        self._last_ground_speed_time = None
        self._ground_speed_valid = False
        self._last_ground_speed_valid_time = None
        self._manual_fallback_requested = False
        self._manual_fallback_latched = False
        self._emergency_kill_requested = False
        self._emergency_kill_latched = False
        self._physical_kill_switch_engaged = False
        self._physical_kill_pwm = None
        self._physical_kill_mission_escape_latched = False
        self._last_physical_kill_disarm_request_time = None
        self._last_physical_kill_manual_request_time = None
        self._termination_active = False
        self._fallback_phase = 'IDLE'
        self._fallback_phase_started_time = None
        self._last_fallback_mode_request_time = None
        self._manual_handoff_complete = False
        self._rc_handoff_watchdog_active = False
        self._last_rc_channels_time = None
        self._rc_rssi = 0
        self._rc_channels = ()
        self._rc_centered_since = None
        self._offboard_phase = 'IDLE'
        self._offboard_origin_mode = None
        self._offboard_phase_started_time = None
        self._last_offboard_mode_request_time = None
        self._last_hold_mode_request_time = None
        self._operator_takeover_latched = False
        self._last_timer_flight_mode = 'UNKNOWN'
        self._release_started_time = None
        self._source_desired = False
        self._source_acquired = False
        self._source_status = 'disabled'
        self._source_wakeup = threading.Event()
        self._last_log_time = 0.0
        self._last_status = ''
        self._last_safety_event_state = None
        self._event_recorder = SafetyEventRecorder(
            self.event_log_path,
            enabled=self.event_log_enabled,
        )

        self.create_subscription(
            Twist,
            self.input_cmd_vel_topic,
            self.cmd_vel_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.activation_topic,
            self.activation_callback,
            10,
        )
        self.create_subscription(
            Float32,
            self.ground_speed_topic,
            self.ground_speed_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.ground_speed_valid_topic,
            self.ground_speed_valid_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.manual_fallback_topic,
            self.manual_fallback_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self.emergency_kill_topic,
            self.emergency_kill_callback,
            10,
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.flight_mode_pub = self.create_publisher(
            String,
            self.flight_mode_topic,
            10,
        )
        self.armed_pub = self.create_publisher(Bool, self.armed_topic, 10)
        self.manual_handoff_pub = self.create_publisher(
            Bool,
            self.manual_handoff_complete_topic,
            10,
        )
        self.create_timer(1.0 / self.command_rate_hz, self.command_timer)

        self._connect_thread = threading.Thread(
            target=self.connect_loop,
            name='cmd_vel_to_mavlink_connect',
            daemon=True,
        )
        self._connect_thread.start()
        self._source_thread = None
        if self.output_enabled and self.source_switching_enabled:
            self._source_thread = threading.Thread(
                target=self.source_manager_loop,
                name='cmd_vel_to_mavlink_source_manager',
                daemon=True,
            )
            self._source_thread.start()

        self.get_logger().info(
            f'CmdVelToMavlink started | input={self.input_cmd_vel_topic} | '
            f'mavlink={self.mavlink_url} | '
            f'output_enabled={self.output_enabled} | '
            f'target_system_id={self.target_system_id or "auto"} | '
            f'require_activation={self.require_activation} | '
            f'source_switching={self.source_switching_enabled} | '
            f'max_throttle_opening={self.max_throttle_opening_percent:.1f}% | '
            f'steering_mavlink_axis={self.steering_mavlink_axis} | '
            f'yaw_to_mavlink_sign={self.yaw_to_mavlink_sign:+.0f} | '
            f'allowed_control_modes={sorted(self.allowed_control_modes)}'
        )
        if self.output_enabled:
            self.get_logger().warning(
                'PX4 hardware output is ENABLED; control remains gated by '
                'heartbeat freshness, flight mode, arming state, avoidance '
                'activation, command timeout, and input-source acquisition.'
            )

    def destroy_node(self):
        self._set_source_desired(False)
        self._stop_event.set()
        self._source_wakeup.set()
        self.close_connection()
        if self._connect_thread.is_alive():
            self._connect_thread.join(timeout=2.0)
        if self._source_thread is not None and self._source_thread.is_alive():
            self._source_thread.join(timeout=3.0)
        self._event_recorder.close()
        if self._output_lock_file is not None:
            self._output_lock_file.close()
            self._output_lock_file = None
        super().destroy_node()

    def close_connection(self):
        with self._lock:
            connection = self._connection
            if self._offboard_phase != 'IDLE':
                # A transport failure during an automated session must never
                # reconnect and silently resume control.
                self._operator_takeover_latched = True
                self._offboard_phase = 'IDLE'
                self._offboard_origin_mode = None
                self._offboard_phase_started_time = None
                self._last_offboard_mode_request_time = None
                self._last_hold_mode_request_time = None
            self._connection = None
            self._connected = False
            self._target_system = 0
            self._target_component = 0
            self._flight_mode = 'UNKNOWN'
            self._armed = False
            self._last_heartbeat_time = None
            self._last_rc_channels_time = None
            self._rc_rssi = 0
            self._rc_channels = ()
            self._rc_centered_since = None
            self._physical_kill_pwm = None
        if connection is not None:
            try:
                connection.close()
            except Exception:
                pass

    def cmd_vel_callback(self, msg):
        with self._lock:
            self._last_cmd = msg
            self._last_cmd_time = time.monotonic()

    def activation_callback(self, msg):
        with self._lock:
            self._avoidance_active = bool(msg.data)
            self._last_activation_time = time.monotonic()

    def ground_speed_callback(self, msg):
        value = float(msg.data)
        now = time.monotonic()
        with self._lock:
            if not math.isfinite(value) or value < 0.0:
                self._ground_speed_valid = False
                self._last_ground_speed_valid_time = now
                return
            self._ground_speed = value
            self._last_ground_speed_time = now

    def ground_speed_valid_callback(self, msg):
        with self._lock:
            self._ground_speed_valid = bool(msg.data)
            self._last_ground_speed_valid_time = time.monotonic()

    def manual_fallback_callback(self, msg):
        requested = bool(msg.data)
        with self._lock:
            self._manual_fallback_requested = requested
            if requested:
                self._manual_fallback_latched = True
            else:
                self._finish_fallback_if_safe_locked()

    def emergency_kill_callback(self, msg):
        requested = bool(msg.data)
        with self._lock:
            self._emergency_kill_requested = requested
            if requested:
                self._emergency_kill_latched = True
                self._manual_fallback_latched = True
            else:
                self._finish_fallback_if_safe_locked()

    def _set_source_desired(self, desired):
        # Production safety invariant: this node may repair/retain the
        # RC-capable input mode, but it must never request MAVLink-only input.
        # Offboard control is independent from COM_RC_IN_MODE.
        desired = False
        changed = False
        with self._lock:
            if self._source_desired != desired:
                self._source_desired = desired
                changed = True
        if changed:
            self._source_wakeup.set()

    def source_manager_loop(self):
        connection = None
        initialized = False
        while not self._stop_event.is_set():
            try:
                if mavutil is None or mavparm is None:
                    raise RuntimeError(
                        'pymavlink parameter support unavailable'
                    )
                if connection is None:
                    connection = mavutil.mavlink_connection(
                        self.mavlink_url,
                        source_system=246,
                        source_component=190,
                        autoreconnect=False,
                    )
                    heartbeat = self._wait_for_px4_heartbeat(connection)
                    connection.target_system = int(heartbeat.get_srcSystem())
                    connection.target_component = int(
                        heartbeat.get_srcComponent()
                    )
                    initialized = False

                if not initialized:
                    self._set_source_status('restoring_startup_input_source')
                    if not self._set_rc_input_mode(
                        connection,
                        SAFE_RC_INPUT_MODE,
                    ):
                        raise RuntimeError(
                            'startup input-source restore failed'
                        )
                    with self._lock:
                        self._source_acquired = False
                    self._set_source_status('operator_input_restored')
                    initialized = True

                with self._lock:
                    acquired = self._source_acquired
                if acquired:
                    self._set_source_status('restoring_input_source')
                    if not self._set_rc_input_mode(
                        connection,
                        SAFE_RC_INPUT_MODE,
                    ):
                        raise RuntimeError('input-source restore failed')
                    with self._lock:
                        self._source_acquired = False
                    self._set_source_status('operator_input_restored')

                self._source_wakeup.wait(timeout=0.2)
                self._source_wakeup.clear()
            except Exception as exc:
                with self._lock:
                    self._source_acquired = False
                self._set_source_status(f'source_manager_error:{exc}')
                if connection is not None:
                    try:
                        connection.close()
                    except Exception:
                        pass
                    connection = None
                self._source_wakeup.wait(timeout=self.source_switch_retry_sec)
                self._source_wakeup.clear()

        if connection is not None:
            try:
                self._set_rc_input_mode(
                    connection,
                    SAFE_RC_INPUT_MODE,
                )
            except Exception:
                pass
            try:
                connection.close()
            except Exception:
                pass
        with self._lock:
            self._source_acquired = False

    def _set_rc_input_mode(self, connection, value):
        if int(value) != SAFE_RC_INPUT_MODE:
            raise ValueError(
                'refusing to set COM_RC_IN_MODE to anything other than 2'
            )
        parameters = mavparm.MAVParmDict()
        return bool(parameters.mavset(
            connection,
            'COM_RC_IN_MODE',
            SAFE_RC_INPUT_MODE,
            retries=3,
            parm_type=mavutil.mavlink.MAV_PARAM_TYPE_INT32,
        ))

    def _set_source_status(self, status):
        with self._lock:
            self._source_status = str(status)

    def connect_loop(self):
        while not self._stop_event.is_set():
            if mavutil is None:
                self.publish_status('pymavlink_missing')
                time.sleep(self.reconnect_delay_sec)
                continue

            try:
                connection = mavutil.mavlink_connection(
                    self.mavlink_url,
                    autoreconnect=False,
                )
                heartbeat = self._wait_for_px4_heartbeat(connection)
                target_system = int(heartbeat.get_srcSystem())
                target_component = int(heartbeat.get_srcComponent())

                with self._lock:
                    self._connection = connection
                    self._connected = True
                    self._target_system = target_system
                    self._target_component = target_component
                    self._update_heartbeat_state_locked(heartbeat)

                self.get_logger().info(
                    f'MAVLink connected | target_system={target_system} | '
                    f'target_component={target_component} | '
                    f'flight_mode={self._flight_mode} | armed={self._armed}'
                )
                self._request_rc_channels(connection)

                while not self._stop_event.is_set():
                    try:
                        msg = connection.recv_match(
                            type=[
                                'HEARTBEAT',
                                'RC_CHANNELS',
                                'STATUSTEXT',
                            ],
                            blocking=True,
                            timeout=0.5,
                        )
                    except Exception:
                        break
                    if is_px4_autopilot_heartbeat(msg, target_system):
                        with self._lock:
                            self._update_heartbeat_state_locked(msg)
                    elif (
                        msg is not None
                        and msg.get_type() == 'RC_CHANNELS'
                        and int(msg.get_srcSystem()) == target_system
                    ):
                        with self._lock:
                            kill_state = self._update_rc_state_locked(msg)
                        if kill_state:
                            # Do not wait for STATUSTEXT: it is not guaranteed
                            # to be forwarded to every MAVLink endpoint.
                            self._set_source_desired(False)
                    elif (
                        msg is not None
                        and msg.get_type() == 'STATUSTEXT'
                        and int(msg.get_srcSystem()) == target_system
                    ):
                        kill_state = px4_kill_switch_state_from_statustext(
                            getattr(msg, 'text', '')
                        )
                        if kill_state is not None:
                            with self._lock:
                                self._update_physical_kill_state_locked(
                                    kill_state
                                )
                            if kill_state:
                                # Wake the source manager immediately instead
                                # of waiting for the next command timer tick.
                                self._set_source_desired(False)

            except Exception as exc:
                self.publish_status(f'disconnected: {exc}')
                self.get_logger().warning(
                    f'MAVLink command bridge disconnected: {exc}; '
                    f'retrying in {self.reconnect_delay_sec:.1f}s'
                )
            finally:
                self.close_connection()

            deadline = time.monotonic() + self.reconnect_delay_sec
            while (
                not self._stop_event.is_set()
                and time.monotonic() < deadline
            ):
                time.sleep(0.1)

    def _wait_for_px4_heartbeat(self, connection):
        deadline = time.monotonic() + self.heartbeat_timeout_sec
        while not self._stop_event.is_set():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                target_text = (
                    str(self.target_system_id)
                    if self.target_system_id > 0
                    else 'auto'
                )
                raise TimeoutError(
                    'no valid PX4 autopilot HEARTBEAT for '
                    f'target_system_id={target_text} within '
                    f'{self.heartbeat_timeout_sec:.1f}s'
                )
            heartbeat = connection.recv_match(
                type='HEARTBEAT',
                blocking=True,
                timeout=min(remaining, 0.5),
            )
            if is_px4_autopilot_heartbeat(
                heartbeat,
                self.target_system_id,
            ):
                return heartbeat

        raise RuntimeError('stopped while waiting for PX4 HEARTBEAT')

    def _update_heartbeat_state_locked(self, msg):
        self._flight_mode = mode_string_from_heartbeat(msg).upper()
        self._armed = heartbeat_is_armed(msg)
        self._last_heartbeat_time = time.monotonic()
        if (
            self._physical_kill_switch_engaged
            and (
                self._flight_mode == 'AUTO.MISSION'
                or (
                    self._flight_mode == 'OFFBOARD'
                    and self._offboard_origin_mode == 'AUTO.MISSION'
                )
            )
        ):
            self._physical_kill_mission_escape_latched = True
        termination_active = flight_mode_requests_emergency_kill(
            self._flight_mode
        )
        if termination_active:
            self._termination_active = True
            self._emergency_kill_latched = True
            self._manual_fallback_latched = True
        elif self._termination_active:
            self._termination_active = False
            self._finish_fallback_if_safe_locked()

    def _update_physical_kill_state_locked(self, engaged):
        self._physical_kill_switch_engaged = bool(engaged)
        if engaged:
            # PX4 already stops the actuators. Do not enter the MAVLink-only
            # fallback path here: COM_RC_IN_MODE=1 disables RC processing and
            # can hide the same switch's subsequent disengage position.
            self._emergency_kill_latched = True
            if (
                self._flight_mode == 'AUTO.MISSION'
                or (
                    self._flight_mode == 'OFFBOARD'
                    and self._offboard_origin_mode == 'AUTO.MISSION'
                )
            ):
                self._physical_kill_mission_escape_latched = True
                self._operator_takeover_latched = True
            return
        self._finish_fallback_if_safe_locked()

    def _emergency_source_active_locked(self):
        return (
            self._emergency_kill_requested
            or self._physical_kill_switch_engaged
            or self._termination_active
        )

    def _finish_fallback_if_safe_locked(self):
        """Clear completed latches regardless of event arrival order."""
        if self._emergency_source_active_locked():
            return False

        if self._manual_fallback_latched:
            if (
                self._manual_fallback_requested
                or not self._manual_handoff_complete
            ):
                return False
            self._manual_fallback_latched = False
            self._manual_handoff_complete = False
            self._fallback_phase = 'IDLE'
            self._fallback_phase_started_time = None
            self._last_fallback_mode_request_time = None

        if self._physical_kill_mission_escape_latched:
            if (
                self._armed
                or self._flight_mode != 'MANUAL'
                or self._source_acquired
            ):
                return False
            self._physical_kill_mission_escape_latched = False
            self._last_physical_kill_disarm_request_time = None
            self._last_physical_kill_manual_request_time = None

        # A physical Kill never acquires MAVLink control. Keep its software
        # latch only until COM_RC_IN_MODE has actually been restored.
        if self._source_acquired:
            return False

        self._emergency_kill_latched = False
        return True

    def _update_rc_state_locked(self, msg):
        channel_count = max(0, min(18, int(getattr(msg, 'chancount', 0))))
        channels = tuple(
            int(getattr(msg, f'chan{index}_raw', 65535))
            for index in range(1, channel_count + 1)
        )
        self._rc_channels = channels
        self._rc_rssi = int(getattr(msg, 'rssi', 0))
        self._last_rc_channels_time = time.monotonic()
        if (
            self.physical_kill_channel <= 0
            or len(channels) < self.physical_kill_channel
        ):
            return None

        pwm = channels[self.physical_kill_channel - 1]
        self._physical_kill_pwm = None if pwm == 65535 else pwm
        kill_state = physical_kill_state_from_rc_pwm(
            current_engaged=self._physical_kill_switch_engaged,
            pwm=pwm,
            trigger_pwm=self.physical_kill_trigger_pwm,
            release_pwm=self.physical_kill_release_pwm,
        )
        if kill_state is not None:
            self._update_physical_kill_state_locked(kill_state)
        return kill_state

    def _request_rc_channels(self, connection):
        try:
            connection.mav.command_long_send(
                connection.target_system or self._target_system,
                connection.target_component or self._target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_RC_CHANNELS,
                100_000,
                0,
                0,
                0,
                0,
                0,
            )
        except Exception as exc:
            self.get_logger().warning(
                f'failed to request RC_CHANNELS stream: {exc}'
            )

    def mode_allows_control(self, now, flight_mode, last_heartbeat_time):
        if last_heartbeat_time is None:
            return False
        if (now - last_heartbeat_time) > self.mode_timeout_sec:
            return False
        return flight_mode in self.allowed_control_modes

    def make_manual_control(
        self,
        cmd,
        fresh,
        now,
        ground_speed,
        ground_speed_fresh,
    ):
        if not fresh or cmd is None:
            self.speed_controller.reset()
            return 0, 0, self.throttle_center, 0

        linear_x = float(cmd.linear.x)
        angular_z = float(cmd.angular.z)
        if not math.isfinite(linear_x) or not math.isfinite(angular_z):
            self.speed_controller.reset()
            return 0, 0, self.throttle_center, 0

        if self.speed_closed_loop_enabled:
            if not ground_speed_fresh:
                self.speed_controller.reset()
                return 0, 0, self.throttle_center, 0
            effort = self.speed_controller.update(
                linear_x,
                ground_speed,
                now,
            )
            if effort is None:
                return 0, 0, self.throttle_center, 0
            linear_x = effort * self.max_linear_speed
        else:
            self.speed_controller.reset()

        return make_manual_control_values(
            linear_x=linear_x,
            angular_z=angular_z,
            max_linear_speed=self.max_linear_speed,
            max_yaw_rate=self.max_yaw_rate,
            throttle_center=self.throttle_center,
            throttle_scale=self.throttle_scale,
            max_throttle_opening_percent=(
                self.max_throttle_opening_percent
            ),
            steering_scale=self.steering_scale,
            yaw_to_mavlink_sign=self.yaw_to_mavlink_sign,
            steering_mavlink_axis=self.steering_mavlink_axis,
        )

    def _request_manual_mode(self, connection, target_system):
        try:
            connection.mav.set_mode_send(
                target_system,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                encode_px4_custom_mode(PX4_MAIN_MODE_MANUAL),
            )
            return True
        except Exception as exc:
            self.get_logger().warning(
                f'failed to request PX4 MANUAL mode: {exc}'
            )
            return False

    def _request_offboard_mode(self, connection, target_system):
        try:
            connection.mav.set_mode_send(
                target_system,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                encode_px4_custom_mode(PX4_MAIN_MODE_OFFBOARD),
            )
            return True
        except Exception as exc:
            self.get_logger().warning(
                f'failed to request PX4 OFFBOARD mode: {exc}'
            )
            return False

    def _request_stopped_mode(
        self,
        connection,
        target_system,
        offboard_origin_mode,
    ):
        if offboard_origin_mode == 'AUTO.MISSION':
            return self._request_auto_mode(
                connection,
                target_system,
                PX4_AUTO_SUB_MODE_LOITER,
            )
        return self._request_manual_mode(connection, target_system)

    def _request_auto_mode(self, connection, target_system, sub_mode):
        try:
            connection.mav.set_mode_send(
                target_system,
                MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                encode_px4_custom_mode(PX4_MAIN_MODE_AUTO, sub_mode),
            )
            return True
        except Exception as exc:
            self.get_logger().warning(
                f'failed to request PX4 AUTO sub-mode {sub_mode}: {exc}'
            )
            return False

    def _send_offboard_velocity(
        self,
        connection,
        target_system,
        target_component,
        cmd,
        command_active,
    ):
        linear_x = 0.0
        angular_z = 0.0
        if command_active and cmd is not None:
            linear_x = float(cmd.linear.x)
            angular_z = float(cmd.angular.z)
        forward_speed, lateral_speed = make_offboard_velocity_setpoint(
            linear_x=linear_x,
            angular_z=angular_z,
            maximum_linear_speed=self.max_linear_speed,
            maximum_yaw_rate=self.max_yaw_rate,
            yaw_to_mavlink_sign=self.yaw_to_mavlink_sign,
            heading_lookahead_sec=self.offboard_heading_lookahead_sec,
            maximum_heading_offset_rad=(
                self.offboard_max_heading_offset_rad
            ),
        )
        connection.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000.0) & 0xFFFFFFFF,
            target_system,
            target_component or MAV_COMP_ID_AUTOPILOT1,
            MAV_FRAME_BODY_NED,
            OFFBOARD_VELOCITY_TYPE_MASK,
            0.0,
            0.0,
            0.0,
            forward_speed,
            lateral_speed,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        return forward_speed, lateral_speed

    def _reset_offboard_session(self, *, operator_takeover=False):
        with self._lock:
            self._offboard_phase = 'IDLE'
            self._offboard_origin_mode = None
            self._offboard_phase_started_time = None
            self._last_offboard_mode_request_time = None
            self._last_hold_mode_request_time = None
            if operator_takeover:
                self._operator_takeover_latched = True

    def _request_disarm(self, connection, target_system, target_component):
        try:
            connection.mav.command_long_send(
                target_system,
                target_component or MAV_COMP_ID_AUTOPILOT1,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0,
                MAV_ARM_DISARM_FORCE_MAGIC,
                0,
                0,
                0,
                0,
                0,
            )
            return True
        except Exception as exc:
            self.get_logger().warning(
                f'failed to request PX4 disarm: {exc}'
            )
            return False

    def _physical_kill_mission_escape_step(
        self,
        *,
        now,
        connection,
        target_system,
        target_component,
        flight_mode,
        armed,
    ):
        """Force a killed mission into disarmed MANUAL before releasing it."""
        with self._lock:
            last_disarm_request = (
                self._last_physical_kill_disarm_request_time
            )
            last_manual_request = (
                self._last_physical_kill_manual_request_time
            )

        if armed and (
            last_disarm_request is None
            or (now - last_disarm_request) >= self.fallback_mode_retry_sec
        ):
            self._request_disarm(
                connection,
                target_system,
                target_component,
            )
            with self._lock:
                self._last_physical_kill_disarm_request_time = now

        if flight_mode != 'MANUAL' and (
            last_manual_request is None
            or (now - last_manual_request) >= self.fallback_mode_retry_sec
        ):
            self._request_manual_mode(connection, target_system)
            with self._lock:
                self._last_physical_kill_manual_request_time = now

    def _reset_manual_fallback(self):
        with self._lock:
            self._manual_fallback_requested = False
            self._manual_fallback_latched = False
            if not self._emergency_source_active_locked():
                self._emergency_kill_latched = False
            self._fallback_phase = 'IDLE'
            self._fallback_phase_started_time = None
            self._last_fallback_mode_request_time = None
            self._manual_handoff_complete = False
            self._rc_handoff_watchdog_active = False
        self.speed_controller.reset()

    def command_timer(self):
        now = time.monotonic()
        with self._lock:
            self._finish_fallback_if_safe_locked()
            connection = self._connection
            connected = self._connected
            target_system = self._target_system
            target_component = self._target_component
            flight_mode = self._flight_mode
            armed = self._armed
            last_heartbeat_time = self._last_heartbeat_time
            cmd = self._last_cmd
            last_cmd_time = self._last_cmd_time
            avoidance_active = self._avoidance_active
            last_activation_time = self._last_activation_time
            ground_speed = self._ground_speed
            last_ground_speed_time = self._last_ground_speed_time
            ground_speed_valid = self._ground_speed_valid
            last_ground_speed_valid_time = (
                self._last_ground_speed_valid_time
            )
            source_acquired = self._source_acquired
            source_status = self._source_status
            manual_fallback_latched = self._manual_fallback_latched
            emergency_kill_requested = self._emergency_kill_requested
            emergency_kill_latched = self._emergency_kill_latched
            physical_kill_switch_engaged = (
                self._physical_kill_switch_engaged
            )
            physical_kill_pwm = self._physical_kill_pwm
            physical_kill_mission_escape_latched = (
                self._physical_kill_mission_escape_latched
            )
            termination_active = self._termination_active
            fallback_phase = self._fallback_phase
            last_rc_channels_time = self._last_rc_channels_time
            rc_rssi = self._rc_rssi
            rc_channels = self._rc_channels
            offboard_phase = self._offboard_phase
            offboard_origin_mode = self._offboard_origin_mode
            offboard_phase_started_time = self._offboard_phase_started_time
            last_offboard_mode_request_time = (
                self._last_offboard_mode_request_time
            )
            last_hold_mode_request_time = self._last_hold_mode_request_time
            operator_takeover_latched = self._operator_takeover_latched
            last_timer_flight_mode = self._last_timer_flight_mode

        cmd_fresh = (
            last_cmd_time is not None
            and (now - last_cmd_time) <= self.cmd_timeout_sec
        )
        activation_fresh = (
            last_activation_time is not None
            and (now - last_activation_time) <= self.activation_timeout_sec
        )
        heartbeat_fresh = (
            last_heartbeat_time is not None
            and (now - last_heartbeat_time) <= self.mode_timeout_sec
        )
        ground_speed_fresh = (
            ground_speed_valid
            and last_ground_speed_time is not None
            and last_ground_speed_valid_time is not None
            and (now - last_ground_speed_time)
            <= self.ground_speed_input_timeout_sec
            and (now - last_ground_speed_valid_time)
            <= self.ground_speed_input_timeout_sec
            and math.isfinite(float(ground_speed))
            and float(ground_speed) >= 0.0
        )
        rc_centered_now = rc_channels_ready_for_handoff(
            now_sec=now,
            last_message_time_sec=last_rc_channels_time,
            rssi=rc_rssi,
            channels=rc_channels,
            timeout_sec=self.rc_channel_timeout_sec,
            minimum_rssi=self.rc_minimum_rssi,
            throttle_channel=self.rc_throttle_channel,
            steering_channel=self.rc_steering_channel,
            neutral_pwm=self.rc_neutral_pwm,
            neutral_tolerance_pwm=self.rc_neutral_tolerance_pwm,
            allow_unknown_rssi=self.allow_unknown_rc_rssi,
            throttle_neutral_pwm=self.rc_throttle_neutral_pwm,
            steering_neutral_pwm=self.rc_steering_neutral_pwm,
        )
        rc_takeover_clear = rc_channels_ready_for_handoff(
            now_sec=now,
            last_message_time_sec=last_rc_channels_time,
            rssi=rc_rssi,
            channels=rc_channels,
            timeout_sec=self.rc_channel_timeout_sec,
            minimum_rssi=self.rc_minimum_rssi,
            throttle_channel=self.rc_throttle_channel,
            steering_channel=self.rc_steering_channel,
            neutral_pwm=self.rc_neutral_pwm,
            neutral_tolerance_pwm=self.rc_takeover_threshold_pwm,
            allow_unknown_rssi=self.allow_unknown_rc_rssi,
            throttle_neutral_pwm=self.rc_throttle_neutral_pwm,
            steering_neutral_pwm=self.rc_steering_neutral_pwm,
        )
        # A deliberately wide tolerance reuses the same freshness, RSSI and
        # channel-range checks without treating intentional stick motion as a
        # lost RC link.
        rc_link_valid = rc_channels_ready_for_handoff(
            now_sec=now,
            last_message_time_sec=last_rc_channels_time,
            rssi=rc_rssi,
            channels=rc_channels,
            timeout_sec=self.rc_channel_timeout_sec,
            minimum_rssi=self.rc_minimum_rssi,
            throttle_channel=self.rc_throttle_channel,
            steering_channel=self.rc_steering_channel,
            neutral_pwm=self.rc_neutral_pwm,
            neutral_tolerance_pwm=1400,
            allow_unknown_rssi=self.allow_unknown_rc_rssi,
            throttle_neutral_pwm=self.rc_throttle_neutral_pwm,
            steering_neutral_pwm=self.rc_steering_neutral_pwm,
        )
        with self._lock:
            if rc_centered_now:
                if self._rc_centered_since is None:
                    self._rc_centered_since = now
            else:
                self._rc_centered_since = None
            rc_centered_since = self._rc_centered_since
        rc_ready = (
            rc_centered_now
            and rc_centered_since is not None
            and (now - rc_centered_since) >= self.rc_entry_neutral_sec
        )
        rc_stick_takeover_requested = (
            rc_link_valid and not rc_takeover_clear
        )
        rc_link_lost = not rc_link_valid
        rc_takeover_requested = (
            rc_stick_takeover_requested or rc_link_lost
        )
        if self.require_activation:
            active_requested = activation_fresh and avoidance_active
            command_active = (
                activation_fresh and avoidance_active and cmd_fresh
            )
        else:
            active_requested = True
            command_active = cmd_fresh
        stop_command_requested = offboard_command_requests_stop(
            cmd,
            command_active,
            self.stop_command_deadband_mps,
        )
        stop_speed_reached = (
            stop_command_requested
            and ground_speed_fresh
            and float(ground_speed) < self.stop_speed_threshold_mps
        )

        armed_allowed = armed or not self.require_armed
        transport_ready = (
            connected
            and connection is not None
            and target_system > 0
            and self.output_enabled
            and armed_allowed
            and heartbeat_fresh
        )
        emergency_control_ready = (
            connected
            and connection is not None
            and target_system > 0
            and self.output_enabled
        )
        operator_source_ready = (
            not self.source_switching_enabled
            or (
                not source_acquired
                and source_status == 'operator_input_restored'
            )
        )

        # COM_RC_IN_MODE is never switched to MAVLink-only. The manager may
        # repair a stale value of 1 left by an older build, but its desired
        # state remains operator/RC input for the entire process lifetime.
        self._set_source_desired(False)

        if not armed and not emergency_kill_latched:
            self._reset_manual_fallback()
            manual_fallback_latched = False
            fallback_phase = 'IDLE'
            with self._lock:
                self._rc_handoff_watchdog_active = False
            if offboard_phase != 'IDLE':
                self._reset_offboard_session()
                offboard_phase = 'IDLE'
                offboard_origin_mode = None

        # Once the operator has taken over, a disarm/rearm alone must not
        # restart automation. Only an explicit MANUAL -> AUTO.MISSION mode
        # selection clears the lockout and authorizes a new mission session.
        if (
            heartbeat_fresh
            and flight_mode == 'AUTO.MISSION'
            and last_timer_flight_mode == 'MANUAL'
            and not emergency_kill_latched
            and not manual_fallback_latched
            and offboard_phase == 'IDLE'
        ):
            with self._lock:
                self._operator_takeover_latched = False
            operator_takeover_latched = False

        # A MANUAL transition during an AUTO-originated Offboard session is an
        # explicit operator takeover. Never resume the task automatically.
        if (
            heartbeat_fresh
            and flight_mode == 'MANUAL'
            and last_timer_flight_mode != 'MANUAL'
            and offboard_phase != 'IDLE'
            and offboard_origin_mode != 'MANUAL'
        ):
            self._reset_offboard_session(operator_takeover=True)
            offboard_phase = 'IDLE'
            offboard_origin_mode = None
            operator_takeover_latched = True

        # RC always outranks either MANUAL- or AUTO-originated assistance.
        # Valid primary-stick motion requests MANUAL immediately. A genuinely
        # stale/invalid RC stream cannot safely hand control to a missing
        # operator, so AUTO-originated assistance falls back to HOLD instead.
        if offboard_phase != 'IDLE' and rc_takeover_requested:
            if emergency_control_ready:
                if rc_link_lost and offboard_origin_mode == 'AUTO.MISSION':
                    self._request_auto_mode(
                        connection,
                        target_system,
                        PX4_AUTO_SUB_MODE_LOITER,
                    )
                elif flight_mode != 'MANUAL':
                    self._request_manual_mode(connection, target_system)
            self._reset_offboard_session(operator_takeover=True)
            offboard_phase = 'IDLE'
            offboard_origin_mode = None
            operator_takeover_latched = True

        fallback_flight_mode = (
            flight_mode if heartbeat_fresh else 'UNKNOWN'
        )
        physical_kill_recovery = (
            emergency_kill_latched and not manual_fallback_latched
        )
        physical_kill_blocked = (
            physical_kill_switch_engaged
            or physical_kill_recovery
            or physical_kill_mission_escape_latched
        )
        offboard_sent = False
        offboard_forward_speed = 0.0
        offboard_lateral_speed = 0.0

        if physical_kill_blocked:
            if offboard_phase != 'IDLE':
                self._reset_offboard_session()
                offboard_phase = 'IDLE'
                offboard_origin_mode = None
            if (
                physical_kill_mission_escape_latched
                and emergency_control_ready
            ):
                self._physical_kill_mission_escape_step(
                    now=now,
                    connection=connection,
                    target_system=target_system,
                    target_component=target_component,
                    flight_mode=fallback_flight_mode,
                    armed=armed,
                )
        elif emergency_kill_requested or termination_active:
            if offboard_phase != 'IDLE':
                self._reset_offboard_session()
                offboard_phase = 'IDLE'
                offboard_origin_mode = None
            fallback_phase = 'EMERGENCY_DISARM_MANUAL'
            with self._lock:
                self._fallback_phase = fallback_phase
            if emergency_control_ready:
                self._physical_kill_mission_escape_step(
                    now=now,
                    connection=connection,
                    target_system=target_system,
                    target_component=target_component,
                    flight_mode=fallback_flight_mode,
                    armed=armed,
                )
        elif manual_fallback_latched:
            if offboard_phase != 'IDLE':
                self._reset_offboard_session()
                offboard_phase = 'IDLE'
                offboard_origin_mode = None
            if heartbeat_fresh and flight_mode == 'MANUAL':
                fallback_phase = 'COMPLETE'
                with self._lock:
                    self._manual_handoff_complete = True
                    self._fallback_phase = fallback_phase
                    self._operator_takeover_latched = True
                operator_takeover_latched = True
            elif emergency_control_ready:
                fallback_phase = 'REQUESTING_MANUAL'
                with self._lock:
                    last_request = self._last_fallback_mode_request_time
                if (
                    last_request is None
                    or (now - last_request) >= self.fallback_mode_retry_sec
                ):
                    self._request_manual_mode(connection, target_system)
                    with self._lock:
                        self._last_fallback_mode_request_time = now
                        self._fallback_phase = fallback_phase
        else:
            activation_failed_closed = (
                self.require_activation
                and not activation_fresh
                and self.fail_closed_on_activation_timeout
            )
            if activation_failed_closed:
                # A dead executor must not leave a stale Offboard command or
                # silently resume the mission. Request HOLD and let RC mode
                # selection remain available throughout.
                if (
                    offboard_phase != 'IDLE'
                    and offboard_origin_mode == 'MANUAL'
                ):
                    if emergency_control_ready and flight_mode != 'MANUAL':
                        self._request_manual_mode(connection, target_system)
                    self._reset_offboard_session(operator_takeover=True)
                    offboard_phase = 'IDLE'
                    offboard_origin_mode = None
                    operator_takeover_latched = True
                elif offboard_phase != 'IDLE' and transport_ready:
                    try:
                        offboard_forward_speed, offboard_lateral_speed = (
                            self._send_offboard_velocity(
                                connection,
                                target_system,
                                target_component,
                                None,
                                False,
                            )
                        )
                        offboard_sent = True
                    except Exception as exc:
                        self.publish_status(f'offboard_send_failed: {exc}')
                        self.close_connection()
                        return
                if emergency_control_ready and (
                    last_hold_mode_request_time is None
                    or (now - last_hold_mode_request_time)
                    >= self.offboard_mode_retry_sec
                ):
                    if (
                        flight_mode == 'AUTO.MISSION'
                        or (
                            flight_mode == 'OFFBOARD'
                            and offboard_origin_mode == 'AUTO.MISSION'
                        )
                    ):
                        self._request_auto_mode(
                            connection,
                            target_system,
                            PX4_AUTO_SUB_MODE_LOITER,
                        )
                    elif (
                        flight_mode == 'OFFBOARD'
                        and offboard_origin_mode == 'MANUAL'
                    ):
                        self._request_manual_mode(
                            connection,
                            target_system,
                        )
                    with self._lock:
                        self._last_hold_mode_request_time = now
                if flight_mode == 'AUTO.LOITER' and offboard_phase != 'IDLE':
                    self._reset_offboard_session()
                    offboard_phase = 'IDLE'
                    offboard_origin_mode = None
            else:
                can_start_manual_assist = (
                    flight_mode == 'MANUAL'
                    and self.manual_avoidance_enabled
                    and not operator_takeover_latched
                    and rc_ready
                )
                can_start_auto_avoidance = (
                    flight_mode == 'AUTO.MISSION'
                    and not operator_takeover_latched
                    and rc_ready
                )
                if (
                    offboard_phase == 'IDLE'
                    and active_requested
                    and command_active
                    and transport_ready
                    and operator_source_ready
                    and (can_start_auto_avoidance or can_start_manual_assist)
                ):
                    offboard_phase = (
                        'STOPPED' if stop_speed_reached else 'WARMUP'
                    )
                    offboard_origin_mode = flight_mode
                    offboard_phase_started_time = now
                    with self._lock:
                        self._offboard_phase = offboard_phase
                        self._offboard_origin_mode = offboard_origin_mode
                        self._offboard_phase_started_time = now
                        self._last_offboard_mode_request_time = None
                        self._last_hold_mode_request_time = None

                expected_modes = {
                    offboard_origin_mode,
                    'AUTO.LOITER',
                    'OFFBOARD',
                }
                if (
                    offboard_phase != 'IDLE'
                    and heartbeat_fresh
                    and flight_mode not in expected_modes
                ):
                    self._reset_offboard_session(
                        operator_takeover=(flight_mode == 'MANUAL')
                    )
                    offboard_phase = 'IDLE'
                    offboard_origin_mode = None
                    operator_takeover_latched = (
                        operator_takeover_latched
                        or flight_mode == 'MANUAL'
                    )

                entry_timed_out = (
                    offboard_phase in ('WARMUP', 'REQUESTING_OFFBOARD')
                    and offboard_phase_started_time is not None
                    and (now - offboard_phase_started_time)
                    >= self.offboard_entry_timeout_sec
                    and flight_mode != 'OFFBOARD'
                )
                if entry_timed_out:
                    if emergency_control_ready:
                        if offboard_origin_mode == 'AUTO.MISSION':
                            self._request_auto_mode(
                                connection,
                                target_system,
                                PX4_AUTO_SUB_MODE_LOITER,
                            )
                        else:
                            self._request_manual_mode(
                                connection,
                                target_system,
                            )
                    self._reset_offboard_session(operator_takeover=True)
                    offboard_phase = 'IDLE'
                    offboard_origin_mode = None
                    operator_takeover_latched = True

                stopped_phase_handled = False
                if (
                    offboard_phase != 'IDLE'
                    and active_requested
                    and stop_speed_reached
                    and offboard_phase != 'STOPPED'
                ):
                    offboard_phase = 'STOPPED'
                    offboard_phase_started_time = now
                    last_offboard_mode_request_time = None
                    with self._lock:
                        self._offboard_phase = offboard_phase
                        self._offboard_phase_started_time = now
                        self._last_offboard_mode_request_time = None

                if offboard_phase == 'STOPPED' and active_requested:
                    if command_active and not stop_command_requested:
                        # A new non-zero avoidance command may reacquire
                        # Offboard, while retaining the original mode context.
                        offboard_phase = 'WARMUP'
                        offboard_phase_started_time = now
                        last_offboard_mode_request_time = None
                        last_hold_mode_request_time = None
                        with self._lock:
                            self._offboard_phase = offboard_phase
                            self._offboard_phase_started_time = now
                            self._last_offboard_mode_request_time = None
                            self._last_hold_mode_request_time = None
                    else:
                        stopped_phase_handled = True
                        request_due = (
                            last_offboard_mode_request_time is None
                            or (now - last_offboard_mode_request_time)
                            >= self.offboard_mode_retry_sec
                        )
                        if emergency_control_ready and request_due:
                            self._request_stopped_mode(
                                connection,
                                target_system,
                                offboard_origin_mode,
                            )
                            last_offboard_mode_request_time = now
                            with self._lock:
                                self._last_offboard_mode_request_time = now
                        if transport_ready and flight_mode == 'OFFBOARD':
                            try:
                                (
                                    offboard_forward_speed,
                                    offboard_lateral_speed,
                                ) = self._send_offboard_velocity(
                                    connection,
                                    target_system,
                                    target_component,
                                    None,
                                    False,
                                )
                                offboard_sent = True
                            except Exception as exc:
                                self.publish_status(
                                    f'offboard_send_failed: {exc}'
                                )
                                self.close_connection()
                                return

                if offboard_phase != 'IDLE' and transport_ready:
                    if active_requested:
                        if not stopped_phase_handled:
                            try:
                                (
                                    offboard_forward_speed,
                                    offboard_lateral_speed,
                                ) = self._send_offboard_velocity(
                                    connection,
                                    target_system,
                                    target_component,
                                    cmd,
                                    command_active,
                                )
                                offboard_sent = True
                            except Exception as exc:
                                self.publish_status(
                                    f'offboard_send_failed: {exc}'
                                )
                                self.close_connection()
                                return

                            if flight_mode == 'OFFBOARD':
                                offboard_phase = 'ACTIVE'
                                with self._lock:
                                    self._offboard_phase = offboard_phase
                            else:
                                if (
                                    self.hold_before_offboard
                                    and offboard_origin_mode == 'AUTO.MISSION'
                                    and flight_mode == 'AUTO.MISSION'
                                    and (
                                        last_hold_mode_request_time is None
                                        or (now - last_hold_mode_request_time)
                                        >= self.offboard_mode_retry_sec
                                    )
                                ):
                                    self._request_auto_mode(
                                        connection,
                                        target_system,
                                        PX4_AUTO_SUB_MODE_LOITER,
                                    )
                                    last_hold_mode_request_time = now
                                    with self._lock:
                                        self._last_hold_mode_request_time = now
                                warmup_elapsed = (
                                    now - offboard_phase_started_time
                                    if offboard_phase_started_time is not None
                                    else 0.0
                                )
                                if (
                                    warmup_elapsed >= self.offboard_warmup_sec
                                    and (
                                        last_offboard_mode_request_time is None
                                        or (
                                            now
                                            - last_offboard_mode_request_time
                                        ) >= self.offboard_mode_retry_sec
                                    )
                                ):
                                    self._request_offboard_mode(
                                        connection,
                                        target_system,
                                    )
                                    last_offboard_mode_request_time = now
                                    offboard_phase = 'REQUESTING_OFFBOARD'
                                    with self._lock:
                                        setattr(
                                            self,
                                            '_last_offboard_mode_request_time',
                                            now,
                                        )
                                        self._offboard_phase = offboard_phase
                    else:
                        if offboard_phase != 'RELEASING':
                            offboard_phase = 'RELEASING'
                            offboard_phase_started_time = now
                            with self._lock:
                                self._offboard_phase = offboard_phase
                                self._offboard_phase_started_time = now
                        if flight_mode == 'OFFBOARD':
                            try:
                                (
                                    offboard_forward_speed,
                                    offboard_lateral_speed,
                                ) = self._send_offboard_velocity(
                                    connection,
                                    target_system,
                                    target_component,
                                    None,
                                    False,
                                )
                                offboard_sent = True
                            except Exception as exc:
                                self.publish_status(
                                    f'offboard_send_failed: {exc}'
                                )
                                self.close_connection()
                                return
                        release_elapsed = now - offboard_phase_started_time
                        release_request_due = (
                            last_offboard_mode_request_time is None
                            or (
                                now - last_offboard_mode_request_time
                            ) >= self.offboard_mode_retry_sec
                        )
                        if (
                            release_elapsed >= self.offboard_release_sec
                            and release_request_due
                        ):
                            if offboard_origin_mode == 'AUTO.MISSION':
                                self._request_auto_mode(
                                    connection,
                                    target_system,
                                    PX4_AUTO_SUB_MODE_MISSION,
                                )
                            elif offboard_origin_mode == 'MANUAL':
                                self._request_manual_mode(
                                    connection,
                                    target_system,
                                )
                            last_offboard_mode_request_time = now
                            with self._lock:
                                self._last_offboard_mode_request_time = now
                        if flight_mode == offboard_origin_mode:
                            self._reset_offboard_session()
                            offboard_phase = 'IDLE'
                            offboard_origin_mode = None

        self.flight_mode_pub.publish(String(data=flight_mode))
        self.armed_pub.publish(Bool(data=armed))
        with self._lock:
            handoff_complete = self._manual_handoff_complete
        self.manual_handoff_pub.publish(Bool(data=handoff_complete))

        status = (
            f'connected={connected} | output_enabled={self.output_enabled} | '
            f'target_system={target_system} | armed={armed} | '
            f'armed_allowed={armed_allowed} | '
            f'flight_mode={flight_mode} | '
            f'activation_fresh={activation_fresh} | '
            f'avoidance_active={avoidance_active} | '
            f'active_requested={active_requested} | '
            f'cmd_fresh={cmd_fresh} | source_acquired={source_acquired} | '
            f'source_status={source_status} | '
            f'operator_source_ready={operator_source_ready} | '
            f'fallback_latched={manual_fallback_latched} | '
            f'emergency_kill_latched={emergency_kill_latched} | '
            f'physical_kill_engaged={physical_kill_switch_engaged} | '
            f'physical_kill_pwm={physical_kill_pwm} | '
            f'physical_kill_blocked={physical_kill_blocked} | '
            'physical_kill_mission_escape='
            f'{physical_kill_mission_escape_latched} | '
            f'termination_active={termination_active} | '
            f'fallback_phase={fallback_phase} | '
            f'rc_fresh_neutral={rc_ready} | '
            f'rc_link_valid={rc_link_valid} | '
            f'rc_stick_takeover={rc_stick_takeover_requested} | '
            f'rc_rssi={rc_rssi} | '
            f'handoff_complete={handoff_complete} | '
            f'ground_speed={float(ground_speed):.3f} | '
            f'ground_speed_fresh={ground_speed_fresh} | '
            f'stop_command={stop_command_requested} | '
            f'stop_speed_reached={stop_speed_reached} | '
            f'offboard_phase={offboard_phase} | '
            f'offboard_origin={offboard_origin_mode} | '
            f'operator_takeover={operator_takeover_latched} | '
            f'offboard_velocity={offboard_forward_speed:.3f},'
            f'{offboard_lateral_speed:.3f} | sent={offboard_sent}'
        )
        self.publish_status(status)

        safety_event_state = (
            connected,
            flight_mode,
            armed,
            activation_fresh,
            avoidance_active,
            source_status,
            manual_fallback_latched,
            emergency_kill_latched,
            physical_kill_switch_engaged,
            physical_kill_mission_escape_latched,
            fallback_phase,
            offboard_phase,
            offboard_origin_mode,
            operator_takeover_latched,
            rc_link_valid,
            rc_stick_takeover_requested,
            ground_speed_fresh,
            stop_command_requested,
            stop_speed_reached,
        )
        if safety_event_state != self._last_safety_event_state:
            self._last_safety_event_state = safety_event_state
            recorder = getattr(self, '_event_recorder', None)
            if recorder is not None:
                recorder.record(
                    'control_state',
                    connected=connected,
                    flight_mode=flight_mode,
                    armed=armed,
                    activation_fresh=activation_fresh,
                    avoidance_active=avoidance_active,
                    source_status=source_status,
                    manual_fallback_latched=manual_fallback_latched,
                    emergency_kill_latched=emergency_kill_latched,
                    physical_kill_engaged=physical_kill_switch_engaged,
                    physical_kill_pwm=physical_kill_pwm,
                    physical_kill_mission_escape=(
                        physical_kill_mission_escape_latched
                    ),
                    fallback_phase=fallback_phase,
                    offboard_phase=offboard_phase,
                    offboard_origin=offboard_origin_mode,
                    operator_takeover=operator_takeover_latched,
                    rc_rssi=rc_rssi,
                    rc_channels=list(rc_channels),
                    rc_link_valid=rc_link_valid,
                    rc_stick_takeover=rc_stick_takeover_requested,
                    ground_speed_mps=ground_speed,
                    ground_speed_fresh=ground_speed_fresh,
                    stop_command=stop_command_requested,
                    stop_speed_reached=stop_speed_reached,
                    command_fresh=cmd_fresh,
                    offboard_sent=offboard_sent,
                    offboard_forward_speed=offboard_forward_speed,
                    offboard_lateral_speed=offboard_lateral_speed,
                )

        with self._lock:
            self._last_timer_flight_mode = flight_mode

        if now - self._last_log_time >= self.log_period_sec:
            self._last_log_time = now
            self.get_logger().info(status)

    def publish_status(self, status):
        if status == self._last_status:
            return
        self._last_status = status
        self.status_pub.publish(String(data=status))


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CmdVelToMavlinkNode()
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
