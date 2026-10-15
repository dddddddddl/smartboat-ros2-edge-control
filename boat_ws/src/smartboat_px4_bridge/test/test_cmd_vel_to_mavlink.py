import math
import threading
import time

import pytest
from geometry_msgs.msg import Twist

from smartboat_px4_bridge.cmd_vel_to_mavlink_node import (
    CmdVelToMavlinkNode,
    SpeedController,
    acquire_exclusive_output_lock,
    activation_timeout_requests_control,
    flight_mode_requests_emergency_kill,
    heartbeat_is_armed,
    is_px4_autopilot_heartbeat,
    encode_px4_custom_mode,
    make_manual_control_values,
    make_offboard_velocity_setpoint,
    mode_string_from_heartbeat,
    offboard_command_requests_stop,
    physical_kill_state_from_rc_pwm,
    px4_kill_switch_state_from_statustext,
    rc_channels_ready_for_handoff,
)


class FakeHeartbeat:
    def __init__(
        self,
        source_system=1,
        source_component=1,
        autopilot=12,
        base_mode=0,
        custom_mode=0,
    ):
        self._source_system = source_system
        self._source_component = source_component
        self.autopilot = autopilot
        self.base_mode = base_mode
        self.custom_mode = custom_mode

    def get_type(self):
        return 'HEARTBEAT'

    def get_srcSystem(self):
        return self._source_system

    def get_srcComponent(self):
        return self._source_component


class FakeMav:
    def __init__(self):
        self.manual_control_calls = []
        self.command_long_calls = []
        self.set_mode_calls = []
        self.set_position_target_local_ned_calls = []

    def manual_control_send(self, *args):
        self.manual_control_calls.append(args)

    def command_long_send(self, *args):
        self.command_long_calls.append(args)

    def set_mode_send(self, *args):
        self.set_mode_calls.append(args)

    def set_position_target_local_ned_send(self, *args):
        self.set_position_target_local_ned_calls.append(args)


class FakeConnection:
    def __init__(self):
        self.mav = FakeMav()


class FakeRcChannels:
    def __init__(self, channels, rssi=255):
        self.chancount = len(channels)
        self.rssi = rssi
        for index, value in enumerate(channels, start=1):
            setattr(self, f'chan{index}_raw', value)


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeSpeedController:
    def reset(self):
        pass


def test_px4_heartbeat_rejects_router_and_non_autopilot_sources():
    assert not is_px4_autopilot_heartbeat(FakeHeartbeat(source_system=0))
    assert not is_px4_autopilot_heartbeat(FakeHeartbeat(source_component=0))
    assert not is_px4_autopilot_heartbeat(FakeHeartbeat(autopilot=8))


def test_hardware_output_process_lock_rejects_second_owner(tmp_path):
    path = tmp_path / 'px4-output.lock'
    first = acquire_exclusive_output_lock(path)
    try:
        with pytest.raises(RuntimeError, match='another PX4'):
            acquire_exclusive_output_lock(path)
    finally:
        first.close()

    replacement = acquire_exclusive_output_lock(path)
    replacement.close()


def test_px4_heartbeat_honors_configured_system_id():
    heartbeat = FakeHeartbeat(source_system=1)

    assert is_px4_autopilot_heartbeat(heartbeat)
    assert is_px4_autopilot_heartbeat(heartbeat, target_system_id=1)
    assert not is_px4_autopilot_heartbeat(heartbeat, target_system_id=2)


def test_heartbeat_armed_flag():
    assert not heartbeat_is_armed(FakeHeartbeat(base_mode=65))
    assert heartbeat_is_armed(FakeHeartbeat(base_mode=193))


def test_px4_custom_mode_fallback_decodes_auto_mission():
    heartbeat = FakeHeartbeat(
        base_mode=193,
        custom_mode=(4 << 16) | (4 << 24),
    )

    assert mode_string_from_heartbeat(heartbeat) == 'AUTO.MISSION'


def test_px4_termination_requests_emergency_rc_fallback():
    assert flight_mode_requests_emergency_kill('TERMINATION')
    assert not flight_mode_requests_emergency_kill('AUTO.MISSION')


def test_px4_physical_kill_switch_announcements_are_detected():
    assert px4_kill_switch_state_from_statustext(
        'Kill-switch engaged'
    ) is True
    assert px4_kill_switch_state_from_statustext(
        'Kill switch disengaged'
    ) is False
    assert px4_kill_switch_state_from_statustext(
        'Manual control lost'
    ) is None


def test_physical_kill_rc_pwm_uses_hysteresis():
    assert physical_kill_state_from_rc_pwm(
        current_engaged=False,
        pwm=1933,
        trigger_pwm=1850,
        release_pwm=1700,
    ) is True
    assert physical_kill_state_from_rc_pwm(
        current_engaged=True,
        pwm=1800,
        trigger_pwm=1850,
        release_pwm=1700,
    ) is None
    assert physical_kill_state_from_rc_pwm(
        current_engaged=True,
        pwm=1514,
        trigger_pwm=1850,
        release_pwm=1700,
    ) is False


def make_fallback_state(*, source_acquired=False):
    node = object.__new__(CmdVelToMavlinkNode)
    node._lock = threading.Lock()
    node._manual_fallback_requested = False
    node._manual_fallback_latched = False
    node._emergency_kill_requested = False
    node._emergency_kill_latched = False
    node._physical_kill_switch_engaged = False
    node._physical_kill_pwm = None
    node._physical_kill_mission_escape_latched = False
    node._last_physical_kill_disarm_request_time = None
    node._last_physical_kill_manual_request_time = None
    node._termination_active = False
    node._flight_mode = 'UNKNOWN'
    node._armed = False
    node._manual_handoff_complete = False
    node._fallback_phase = 'IDLE'
    node._fallback_phase_started_time = None
    node._last_fallback_mode_request_time = None
    node._source_acquired = source_acquired
    node._offboard_phase = 'IDLE'
    node._offboard_origin_mode = None
    node._offboard_phase_started_time = None
    node._last_offboard_mode_request_time = None
    node._last_hold_mode_request_time = None
    node._operator_takeover_latched = False
    node._last_timer_flight_mode = 'UNKNOWN'
    node.physical_kill_channel = 7
    node.physical_kill_trigger_pwm = 1850
    node.physical_kill_release_pwm = 1700
    return node


def make_command_timer_state(flight_mode):
    node = make_fallback_state(source_acquired=False)
    now = time.monotonic()
    node._connection = FakeConnection()
    node._connected = True
    node._target_system = 1
    node._target_component = 1
    node._flight_mode = flight_mode
    node._armed = True
    node._last_heartbeat_time = now
    node._last_cmd = None
    node._last_cmd_time = None
    node._avoidance_active = False
    node._last_activation_time = now
    node._source_status = 'operator_input_restored'
    node._source_desired = False
    node._source_wakeup = threading.Event()
    node._ground_speed = 0.0
    node._last_ground_speed_time = None
    node._ground_speed_valid = False
    node._last_ground_speed_valid_time = None
    node._rc_handoff_watchdog_active = False
    node._last_rc_channels_time = now
    node._rc_rssi = 255
    node._rc_channels = (1514, 1514, 1514, 1514)
    node._rc_centered_since = now - 1.0
    node._release_started_time = None
    node.cmd_timeout_sec = 0.3
    node.activation_timeout_sec = 1.0
    node.ground_speed_input_timeout_sec = 0.5
    node.mode_timeout_sec = 2.0
    node.rc_channel_timeout_sec = 0.8
    node.rc_minimum_rssi = 1
    node.rc_throttle_channel = 3
    node.rc_steering_channel = 1
    node.rc_neutral_pwm = 1500
    node.rc_throttle_neutral_pwm = 1514
    node.rc_steering_neutral_pwm = 1514
    node.rc_neutral_tolerance_pwm = 100
    node.rc_takeover_threshold_pwm = 80
    node.rc_entry_neutral_sec = 0.3
    node.allow_unknown_rc_rssi = True
    node.stop_speed_threshold_mps = 0.2
    node.stop_command_deadband_mps = 0.02
    node.require_activation = True
    node.fail_closed_on_activation_timeout = True
    node.manual_release_on_activation_timeout = True
    node.require_armed = True
    node.output_enabled = True
    node.allowed_control_modes = {'MANUAL', 'AUTO.MISSION'}
    node.source_switching_enabled = True
    node.offboard_warmup_sec = 1.0
    node.offboard_release_sec = 0.0
    node.offboard_mode_retry_sec = 1.0
    node.offboard_entry_timeout_sec = 4.0
    node.offboard_heading_lookahead_sec = 1.0
    node.offboard_max_heading_offset_rad = 0.7
    node.hold_before_offboard = True
    node.manual_avoidance_enabled = True
    node.neutral_on_timeout = True
    node.max_linear_speed = 1.0
    node.max_yaw_rate = 0.7
    node.yaw_to_mavlink_sign = -1.0
    node.throttle_center = 500
    node.fallback_mode_retry_sec = 1.0
    node.speed_controller = FakeSpeedController()
    node.flight_mode_pub = FakePublisher()
    node.armed_pub = FakePublisher()
    node.manual_handoff_pub = FakePublisher()
    node.status_pub = FakePublisher()
    node._last_status = ''
    node._last_safety_event_state = None
    node._last_log_time = now
    node.log_period_sec = 3600.0
    return node


def test_physical_kill_does_not_enter_mavlink_fallback():
    node = make_fallback_state()

    node._update_physical_kill_state_locked(True)

    assert node._physical_kill_switch_engaged
    assert node._emergency_kill_latched
    assert not node._manual_fallback_latched


def test_input_source_invariant_rejects_mavlink_only_request():
    node = make_command_timer_state('AUTO.MISSION')

    node._set_source_desired(True)

    assert node._source_desired is False


def test_rc_input_parameter_writer_hard_rejects_non_rc_mode():
    node = make_command_timer_state('MANUAL')

    with pytest.raises(ValueError, match='anything other than 2'):
        node._set_rc_input_mode(node._connection, 1)


def test_rc_channels_drive_physical_kill_without_statustext():
    node = make_fallback_state()
    node._flight_mode = 'AUTO.MISSION'
    node._armed = True
    centered = (1514,) * 7
    triggered = centered[:6] + (1933,)

    assert node._update_rc_state_locked(FakeRcChannels(triggered)) is True
    assert node._physical_kill_switch_engaged
    assert node._physical_kill_pwm == 1933
    assert node._physical_kill_mission_escape_latched

    assert node._update_rc_state_locked(FakeRcChannels(centered)) is False
    assert not node._physical_kill_switch_engaged
    assert node._emergency_kill_latched


def test_auto_mission_heartbeat_after_kill_still_latches_escape():
    node = make_fallback_state()
    node._update_physical_kill_state_locked(True)
    assert not node._physical_kill_mission_escape_latched

    heartbeat = FakeHeartbeat(
        base_mode=193,
        custom_mode=(4 << 16) | (4 << 24),
    )
    node._update_heartbeat_state_locked(heartbeat)

    assert node._flight_mode == 'AUTO.MISSION'
    assert node._armed
    assert node._physical_kill_mission_escape_latched


def test_physical_kill_release_waits_only_for_rc_source_restore():
    node = make_fallback_state(source_acquired=True)
    node._update_physical_kill_state_locked(True)

    node._update_physical_kill_state_locked(False)
    assert not node._physical_kill_switch_engaged
    assert node._emergency_kill_latched

    node._source_acquired = False
    assert node._finish_fallback_if_safe_locked()
    assert not node._emergency_kill_latched


def test_physical_kill_release_clears_after_earlier_rc_source_restore():
    node = make_fallback_state(source_acquired=True)
    node._update_physical_kill_state_locked(True)

    node._source_acquired = False
    assert not node._finish_fallback_if_safe_locked()
    assert node._emergency_kill_latched

    node._update_physical_kill_state_locked(False)
    assert not node._physical_kill_switch_engaged
    assert not node._emergency_kill_latched


def test_physical_kill_in_manual_restores_rc_without_forcing_disarm():
    node = make_command_timer_state('MANUAL')

    node._update_physical_kill_state_locked(True)
    node.command_timer()

    assert node._source_desired is False
    assert not node._physical_kill_mission_escape_latched
    assert not node._connection.mav.manual_control_calls
    assert not node._connection.mav.command_long_calls
    assert not node._connection.mav.set_mode_calls

    node._update_physical_kill_state_locked(False)
    node._source_acquired = False
    node._source_status = 'released'
    node.command_timer()

    assert not node._emergency_kill_latched
    assert node._armed
    assert node._flight_mode == 'MANUAL'
    assert node._source_desired is False
    assert not node._connection.mav.manual_control_calls


def test_physical_kill_in_mission_requires_disarmed_manual_and_rc():
    node = make_command_timer_state('AUTO.MISSION')

    node._update_physical_kill_state_locked(True)
    node.command_timer()

    assert node._source_desired is False
    assert node._physical_kill_mission_escape_latched
    assert len(node._connection.mav.command_long_calls) == 1
    assert node._connection.mav.command_long_calls[0][4] == 0
    assert node._connection.mav.command_long_calls[0][5] == 21196.0
    assert len(node._connection.mav.set_mode_calls) == 1
    assert not node._connection.mav.manual_control_calls

    # Returning the wheel and restoring RC are insufficient while PX4 still
    # reports armed AUTO.MISSION.
    node._update_physical_kill_state_locked(False)
    node._source_acquired = False
    node._source_status = 'released'
    node.command_timer()

    assert node._emergency_kill_latched
    assert node._physical_kill_mission_escape_latched
    assert node._source_desired is False
    assert not node._connection.mav.manual_control_calls

    # Simulate subsequent PX4 heartbeat confirmations.
    node._flight_mode = 'MANUAL'
    node._armed = False
    node._last_heartbeat_time = time.monotonic()
    node.command_timer()

    assert not node._emergency_kill_latched
    assert not node._physical_kill_mission_escape_latched
    assert not node._manual_fallback_latched
    assert node._source_desired is False
    assert not node._connection.mav.manual_control_calls
    assert node.flight_mode_pub.messages[-1].data == 'MANUAL'
    assert node.armed_pub.messages[-1].data is False
    assert 'physical_kill_blocked=False' in (
        node.status_pub.messages[-1].data
    )


def test_mission_escape_waits_for_kill_release_after_other_confirmations():
    node = make_fallback_state(source_acquired=True)
    node._flight_mode = 'AUTO.MISSION'
    node._armed = True
    node._update_physical_kill_state_locked(True)

    node._source_acquired = False
    node._flight_mode = 'MANUAL'
    node._armed = False
    assert not node._finish_fallback_if_safe_locked()
    assert node._physical_kill_mission_escape_latched

    node._update_physical_kill_state_locked(False)
    assert not node._emergency_kill_latched
    assert not node._physical_kill_mission_escape_latched


def test_early_emergency_release_clears_when_handoff_finishes_later():
    node = make_fallback_state()
    node._emergency_kill_requested = True
    node._emergency_kill_latched = True
    node._manual_fallback_latched = True

    node._emergency_kill_requested = False
    assert not node._finish_fallback_if_safe_locked()
    assert node._emergency_kill_latched

    node._manual_handoff_complete = True
    node._fallback_phase = 'COMPLETE'
    assert node._finish_fallback_if_safe_locked()
    assert not node._emergency_kill_latched
    assert not node._manual_fallback_latched
    assert node._fallback_phase == 'IDLE'


def test_termination_state_is_independent_from_ros_emergency_request():
    node = make_fallback_state()
    node._termination_active = True
    node._emergency_kill_latched = True
    node._manual_fallback_latched = True
    node._manual_handoff_complete = True

    assert not node._finish_fallback_if_safe_locked()
    assert not node._emergency_kill_requested

    node._termination_active = False
    assert node._finish_fallback_if_safe_locked()
    assert not node._emergency_kill_latched


def test_activation_timeout_releases_manual_but_stops_mission():
    assert not activation_timeout_requests_control('MANUAL', True, True)
    assert activation_timeout_requests_control('AUTO.MISSION', True, True)
    assert not activation_timeout_requests_control('AUTO.MISSION', False, True)


def test_offboard_velocity_is_bounded_and_uses_px4_yaw_sign():
    forward, lateral = make_offboard_velocity_setpoint(
        linear_x=2.0,
        angular_z=1.0,
        maximum_linear_speed=0.3,
        maximum_yaw_rate=0.4,
        yaw_to_mavlink_sign=-1.0,
        heading_lookahead_sec=1.0,
        maximum_heading_offset_rad=0.3,
    )
    assert forward == pytest.approx(0.3 * math.cos(0.3))
    assert lateral == pytest.approx(-0.3 * math.sin(0.3))
    assert encode_px4_custom_mode(4, 4) == (4 << 16) | (4 << 24)


def test_auto_avoidance_warmup_holds_without_disabling_rc():
    node = make_command_timer_state('AUTO.MISSION')
    node._avoidance_active = True
    node._last_activation_time = time.monotonic()
    node._last_cmd_time = time.monotonic()
    node._last_cmd = Twist()
    node._last_cmd.linear.x = 0.2
    node._last_cmd.angular.z = 0.35

    node.command_timer()

    assert node._offboard_phase == 'WARMUP', node.status_pub.messages[-1].data
    assert node._offboard_origin_mode == 'AUTO.MISSION'
    assert node._source_desired is False
    assert not node._connection.mav.manual_control_calls
    assert len(node._connection.mav.set_position_target_local_ned_calls) == 1
    setpoint = node._connection.mav.set_position_target_local_ned_calls[0]
    assert setpoint[8] > 0.0
    assert setpoint[9] < 0.0
    assert setpoint[15] == 0.0
    assert setpoint[4] & (1 << 11)
    assert len(node._connection.mav.set_mode_calls) == 1
    assert node._connection.mav.set_mode_calls[0][2] == (
        (4 << 16) | (3 << 24)
    )


def test_warmed_hold_requests_offboard_and_keeps_rc_source():
    node = make_command_timer_state('AUTO.LOITER')
    node._avoidance_active = True
    node._last_activation_time = time.monotonic()
    node._last_cmd_time = time.monotonic()
    node._last_cmd = Twist()
    node._offboard_phase = 'WARMUP'
    node._offboard_origin_mode = 'AUTO.MISSION'
    node._offboard_phase_started_time = time.monotonic() - 2.0

    node.command_timer()

    assert node._offboard_phase == 'REQUESTING_OFFBOARD', (
        node.status_pub.messages[-1].data
    )
    assert node._source_desired is False
    assert node._connection.mav.set_mode_calls[-1][2] == (6 << 16)
    assert not node._connection.mav.manual_control_calls


def test_manual_mode_transition_cancels_auto_session_permanently():
    node = make_command_timer_state('MANUAL')
    node._offboard_phase = 'ACTIVE'
    node._offboard_origin_mode = 'AUTO.MISSION'
    node._offboard_phase_started_time = time.monotonic() - 1.0
    node._last_timer_flight_mode = 'OFFBOARD'
    node._avoidance_active = True

    node.command_timer()

    assert node._offboard_phase == 'IDLE'
    assert node._operator_takeover_latched
    assert not node._connection.mav.set_position_target_local_ned_calls
    assert not node._connection.mav.manual_control_calls


def test_manual_assist_stick_movement_immediately_returns_manual():
    node = make_command_timer_state('OFFBOARD')
    node._offboard_phase = 'ACTIVE'
    node._offboard_origin_mode = 'MANUAL'
    node._offboard_phase_started_time = time.monotonic() - 1.0
    node._last_timer_flight_mode = 'OFFBOARD'
    node._avoidance_active = True
    node._last_activation_time = time.monotonic()
    node._last_rc_channels_time = time.monotonic()
    node._rc_rssi = 255
    node._rc_channels = (1514, 1514, 1800, 1514)

    node.command_timer()

    assert node._offboard_phase == 'IDLE'
    assert node._operator_takeover_latched
    assert node._connection.mav.set_mode_calls[-1][2] == (1 << 16)
    assert not node._connection.mav.set_position_target_local_ned_calls


def test_manual_assist_steering_movement_immediately_returns_manual():
    node = make_command_timer_state('OFFBOARD')
    node._offboard_phase = 'ACTIVE'
    node._offboard_origin_mode = 'MANUAL'
    node._offboard_phase_started_time = time.monotonic() - 1.0
    node._last_timer_flight_mode = 'OFFBOARD'
    node._avoidance_active = True
    node._last_activation_time = time.monotonic()
    node._last_rc_channels_time = time.monotonic()
    node._rc_channels = (1750, 1514, 1514, 1514)

    node.command_timer()

    assert node._offboard_phase == 'IDLE'
    assert node._operator_takeover_latched
    assert node._connection.mav.set_mode_calls[-1][2] == (1 << 16)
    assert not node._connection.mav.set_position_target_local_ned_calls


def test_rc_takeover_threshold_is_separate_from_entry_tolerance():
    node = make_command_timer_state('OFFBOARD')
    node._offboard_phase = 'ACTIVE'
    node._offboard_origin_mode = 'MANUAL'
    node._offboard_phase_started_time = time.monotonic() - 1.0
    node._last_timer_flight_mode = 'OFFBOARD'
    node._avoidance_active = True
    node._last_activation_time = time.monotonic()
    node._last_rc_channels_time = time.monotonic()
    node._rc_channels = (1594, 1514, 1514, 1514)

    node.command_timer()

    assert node._offboard_phase == 'ACTIVE'
    assert not node._operator_takeover_latched

    node._last_heartbeat_time = time.monotonic()
    node._last_activation_time = time.monotonic()
    node._last_rc_channels_time = time.monotonic()
    node._rc_channels = (1595, 1514, 1514, 1514)
    node.command_timer()

    assert node._offboard_phase == 'IDLE'
    assert node._operator_takeover_latched
    assert node._connection.mav.set_mode_calls[-1][2] == (1 << 16)


def test_auto_assist_rc_link_loss_exits_to_hold():
    node = make_command_timer_state('OFFBOARD')
    node._offboard_phase = 'ACTIVE'
    node._offboard_origin_mode = 'AUTO.MISSION'
    node._offboard_phase_started_time = time.monotonic() - 1.0
    node._last_timer_flight_mode = 'OFFBOARD'
    node._avoidance_active = True
    node._last_activation_time = time.monotonic()
    node._last_rc_channels_time = None
    node._rc_channels = ()

    node.command_timer()

    assert node._offboard_phase == 'IDLE'
    assert node._operator_takeover_latched
    assert node._connection.mav.set_mode_calls[-1][2] == (
        (4 << 16) | (3 << 24)
    )
    assert not node._connection.mav.set_position_target_local_ned_calls


def test_rc_must_remain_centered_before_manual_assist_entry():
    node = make_command_timer_state('MANUAL')
    node._avoidance_active = True
    node._last_activation_time = time.monotonic()
    node._last_cmd = Twist()
    node._last_cmd.linear.x = 0.2
    node._last_cmd_time = time.monotonic()
    node._rc_centered_since = None

    node.command_timer()
    assert node._offboard_phase == 'IDLE'

    node._last_heartbeat_time = time.monotonic()
    node._last_activation_time = time.monotonic()
    node._last_cmd_time = time.monotonic()
    node._last_rc_channels_time = time.monotonic()
    node._rc_centered_since = time.monotonic() - 0.4
    node.command_timer()

    assert node._offboard_phase == 'WARMUP'


def test_localization_fallback_requests_manual_without_neutral_or_rssi_gate():
    node = make_command_timer_state('AUTO.MISSION')
    node._manual_fallback_latched = True
    node._manual_fallback_requested = True
    node._rc_rssi = 0
    node._rc_channels = (1800, 1200, 1900, 1100)

    node.command_timer()

    assert node._connection.mav.set_mode_calls[-1][2] == (1 << 16)
    assert node._source_desired is False
    assert not node._connection.mav.manual_control_calls


def test_executor_timeout_exits_offboard_to_hold_not_mission():
    node = make_command_timer_state('OFFBOARD')
    node._offboard_phase = 'ACTIVE'
    node._offboard_origin_mode = 'AUTO.MISSION'
    node._offboard_phase_started_time = time.monotonic() - 2.0
    node._last_activation_time = time.monotonic() - 5.0

    node.command_timer()

    assert node._connection.mav.set_mode_calls[-1][2] == (
        (4 << 16) | (3 << 24)
    )
    assert len(node._connection.mav.set_position_target_local_ned_calls) == 1
    assert not node._connection.mav.manual_control_calls


def test_executor_timeout_aborts_pending_manual_offboard_entry():
    node = make_command_timer_state('MANUAL')
    node._offboard_phase = 'REQUESTING_OFFBOARD'
    node._offboard_origin_mode = 'MANUAL'
    node._offboard_phase_started_time = time.monotonic() - 2.0
    node._last_activation_time = time.monotonic() - 5.0

    node.command_timer()

    assert node._offboard_phase == 'IDLE'
    assert node._operator_takeover_latched
    assert not node._connection.mav.set_position_target_local_ned_calls


def test_rejected_offboard_entry_times_out_to_manual_and_locks_out():
    node = make_command_timer_state('MANUAL')
    node._avoidance_active = True
    node._last_activation_time = time.monotonic()
    node._last_cmd = Twist()
    node._last_cmd_time = time.monotonic()
    node._last_rc_channels_time = time.monotonic()
    node._rc_rssi = 255
    node._rc_channels = (1514, 1514, 1500, 1500)
    node._offboard_phase = 'REQUESTING_OFFBOARD'
    node._offboard_origin_mode = 'MANUAL'
    node._offboard_phase_started_time = time.monotonic() - 5.0

    node.command_timer()

    assert node._offboard_phase == 'IDLE'
    assert node._operator_takeover_latched
    assert node._connection.mav.set_mode_calls[-1][2] == (1 << 16)
    assert not node._connection.mav.set_position_target_local_ned_calls


def test_transport_loss_locks_out_automatic_resume():
    node = make_command_timer_state('OFFBOARD')
    node._offboard_phase = 'ACTIVE'
    node._offboard_origin_mode = 'AUTO.MISSION'

    node.close_connection()

    assert node._offboard_phase == 'IDLE'
    assert node._offboard_origin_mode is None
    assert node._operator_takeover_latched


def test_operator_lockout_survives_disarm_until_manual_to_mission():
    node = make_command_timer_state('MANUAL')
    node._armed = False
    node._operator_takeover_latched = True
    node._avoidance_active = True
    node._last_rc_channels_time = time.monotonic()
    node._rc_rssi = 255
    node._rc_channels = (1514, 1514, 1500, 1500)

    node.command_timer()

    assert node._operator_takeover_latched
    assert node._offboard_phase == 'IDLE'

    node._armed = True
    node._last_heartbeat_time = time.monotonic()
    node.command_timer()
    assert node._operator_takeover_latched
    assert node._offboard_phase == 'IDLE'

    node._flight_mode = 'AUTO.MISSION'
    node._last_heartbeat_time = time.monotonic()
    node.command_timer()
    assert not node._operator_takeover_latched


def test_auto_offboard_kill_forces_disarm_manual_without_manual_control():
    node = make_command_timer_state('OFFBOARD')
    node._offboard_phase = 'ACTIVE'
    node._offboard_origin_mode = 'AUTO.MISSION'
    node._last_timer_flight_mode = 'OFFBOARD'

    node._update_physical_kill_state_locked(True)
    node.command_timer()

    assert node._operator_takeover_latched
    assert node._physical_kill_mission_escape_latched
    assert node._offboard_phase == 'IDLE'
    assert len(node._connection.mav.command_long_calls) == 1
    assert node._connection.mav.set_mode_calls[-1][2] == (1 << 16)
    assert not node._connection.mav.manual_control_calls


def test_clear_auto_offboard_session_resumes_mission_only_without_takeover():
    node = make_command_timer_state('OFFBOARD')
    node._offboard_phase = 'ACTIVE'
    node._offboard_origin_mode = 'AUTO.MISSION'
    node._offboard_phase_started_time = time.monotonic() - 2.0
    node._last_timer_flight_mode = 'OFFBOARD'
    node._avoidance_active = False

    node.command_timer()

    assert node._offboard_phase == 'RELEASING'
    assert node._connection.mav.set_mode_calls[-1][2] == (
        (4 << 16) | (4 << 24)
    )
    assert not node._connection.mav.manual_control_calls


def test_ros_positive_yaw_maps_to_negative_mavlink_y_for_rover():
    values = make_manual_control_values(
        linear_x=0.0,
        angular_z=0.1,
        max_linear_speed=1.0,
        max_yaw_rate=0.7,
        throttle_center=500,
        throttle_scale=500,
        max_throttle_opening_percent=100.0,
        steering_scale=1000,
        yaw_to_mavlink_sign=-1.0,
        steering_mavlink_axis='y',
    )

    assert values == (0, -142, 500, 0)


def test_steering_axis_can_remain_mavlink_r_for_other_airframes():
    values = make_manual_control_values(
        linear_x=0.0,
        angular_z=0.1,
        max_linear_speed=1.0,
        max_yaw_rate=0.7,
        throttle_center=500,
        throttle_scale=500,
        max_throttle_opening_percent=100.0,
        steering_scale=1000,
        yaw_to_mavlink_sign=-1.0,
        steering_mavlink_axis='r',
    )

    assert values == (0, 0, 500, -142)


def test_shore_scales_limit_full_input():
    values = make_manual_control_values(
        linear_x=10.0,
        angular_z=-10.0,
        max_linear_speed=1.0,
        max_yaw_rate=0.7,
        throttle_center=500,
        throttle_scale=50,
        max_throttle_opening_percent=10.0,
        steering_scale=150,
        yaw_to_mavlink_sign=-1.0,
        steering_mavlink_axis='y',
    )

    assert values == (0, 150, 550, 0)


def test_production_throttle_opening_has_independent_fifty_percent_cap():
    common = {
        'angular_z': 0.0,
        'max_linear_speed': 1.0,
        'max_yaw_rate': 0.7,
        'throttle_center': 500,
        'throttle_scale': 500,
        'max_throttle_opening_percent': 50.0,
        'steering_scale': 1000,
        'yaw_to_mavlink_sign': -1.0,
        'steering_mavlink_axis': 'y',
    }

    forward = make_manual_control_values(linear_x=10.0, **common)
    reverse = make_manual_control_values(linear_x=-10.0, **common)

    assert forward[2] == 750
    assert reverse[2] == 250


def test_non_finite_ros_command_is_always_neutral():
    values = make_manual_control_values(
        linear_x=float('nan'),
        angular_z=0.0,
        max_linear_speed=1.0,
        max_yaw_rate=0.7,
        throttle_center=500,
        throttle_scale=500,
        max_throttle_opening_percent=100.0,
        steering_scale=1000,
        yaw_to_mavlink_sign=-1.0,
        steering_mavlink_axis='y',
    )
    assert values == (0, 0, 500, 0)


def test_speed_controller_never_uses_opposite_thrust_for_overspeed():
    controller = SpeedController(
        kp=0.8,
        ki=0.25,
        integral_limit=2.0,
        maximum_effort=0.5,
        overspeed_tolerance_mps=0.05,
        target_deadband_mps=0.02,
    )
    assert controller.update(0.2, 0.4, 1.0) == 0.0
    assert controller.update(-0.2, 0.4, 1.1) == 0.0
    assert controller.update(0.2, float('nan'), 1.2) is None


def test_speed_controller_effort_is_bounded_and_same_direction():
    controller = SpeedController(
        kp=4.0,
        ki=2.0,
        integral_limit=2.0,
        maximum_effort=0.3,
        overspeed_tolerance_mps=0.05,
        target_deadband_mps=0.02,
    )
    assert controller.update(1.0, 0.0, 1.0) == 0.3
    assert controller.update(-1.0, 0.0, 1.1) == -0.3


def test_stop_command_requires_fresh_active_near_zero_linear_velocity():
    cmd = Twist()
    cmd.angular.z = 0.7
    assert offboard_command_requests_stop(cmd, True, 0.02)
    cmd.linear.x = 0.02
    assert offboard_command_requests_stop(cmd, True, 0.02)
    cmd.linear.x = 0.021
    assert not offboard_command_requests_stop(cmd, True, 0.02)
    assert not offboard_command_requests_stop(cmd, False, 0.02)
    cmd.linear.x = float('nan')
    assert not offboard_command_requests_stop(cmd, True, 0.02)


def prepare_active_stop(node, *, speed_mps):
    now = time.monotonic()
    node._avoidance_active = True
    node._last_activation_time = now
    node._last_cmd = Twist()
    node._last_cmd_time = now
    node._ground_speed = speed_mps
    node._last_ground_speed_time = now
    node._ground_speed_valid = True
    node._last_ground_speed_valid_time = now


def test_manual_stop_below_point_two_exits_offboard_control_to_manual():
    node = make_command_timer_state('OFFBOARD')
    node._offboard_phase = 'ACTIVE'
    node._offboard_origin_mode = 'MANUAL'
    node._offboard_phase_started_time = time.monotonic() - 1.0
    node._last_timer_flight_mode = 'OFFBOARD'
    prepare_active_stop(node, speed_mps=0.199)

    node.command_timer()

    assert node._offboard_phase == 'STOPPED'
    assert node._connection.mav.set_mode_calls[-1][2] == (1 << 16)
    assert len(node._connection.mav.set_position_target_local_ned_calls) == 1
    setpoint = node._connection.mav.set_position_target_local_ned_calls[0]
    assert setpoint[8] == 0.0
    assert setpoint[9] == 0.0


def test_mission_stop_below_point_two_exits_offboard_control_to_hold():
    node = make_command_timer_state('OFFBOARD')
    node._offboard_phase = 'ACTIVE'
    node._offboard_origin_mode = 'AUTO.MISSION'
    node._offboard_phase_started_time = time.monotonic() - 1.0
    node._last_timer_flight_mode = 'OFFBOARD'
    prepare_active_stop(node, speed_mps=0.199)

    node.command_timer()

    assert node._offboard_phase == 'STOPPED'
    assert node._connection.mav.set_mode_calls[-1][2] == (
        (4 << 16) | (3 << 24)
    )
    assert len(node._connection.mav.set_position_target_local_ned_calls) == 1


@pytest.mark.parametrize('speed_mps', [0.2, 0.201])
def test_stop_at_or_above_point_two_keeps_zero_offboard_braking(speed_mps):
    node = make_command_timer_state('OFFBOARD')
    node._offboard_phase = 'ACTIVE'
    node._offboard_origin_mode = 'MANUAL'
    node._offboard_phase_started_time = time.monotonic() - 1.0
    node._last_timer_flight_mode = 'OFFBOARD'
    prepare_active_stop(node, speed_mps=speed_mps)

    node.command_timer()

    assert node._offboard_phase == 'ACTIVE'
    assert not node._connection.mav.set_mode_calls
    assert len(node._connection.mav.set_position_target_local_ned_calls) == 1


def test_stale_ground_speed_never_claims_stopped():
    node = make_command_timer_state('OFFBOARD')
    node._offboard_phase = 'ACTIVE'
    node._offboard_origin_mode = 'MANUAL'
    node._offboard_phase_started_time = time.monotonic() - 1.0
    node._last_timer_flight_mode = 'OFFBOARD'
    prepare_active_stop(node, speed_mps=0.0)
    node._last_ground_speed_time = time.monotonic() - 2.0

    node.command_timer()

    assert node._offboard_phase == 'ACTIVE'
    assert not node._connection.mav.set_mode_calls


def test_stopped_session_restarts_warmup_for_new_nonzero_command():
    node = make_command_timer_state('MANUAL')
    node._offboard_phase = 'STOPPED'
    node._offboard_origin_mode = 'MANUAL'
    node._offboard_phase_started_time = time.monotonic() - 1.0
    node._last_timer_flight_mode = 'MANUAL'
    now = time.monotonic()
    node._avoidance_active = True
    node._last_activation_time = now
    node._last_cmd = Twist()
    node._last_cmd.linear.x = 0.2
    node._last_cmd_time = now

    node.command_timer()

    assert node._offboard_phase == 'WARMUP'
    assert node._offboard_origin_mode == 'MANUAL'
    assert len(node._connection.mav.set_position_target_local_ned_calls) == 1
    assert node._connection.mav.set_position_target_local_ned_calls[0][8] > 0


def rc_ready(**overrides):
    values = {
        'now_sec': 10.0,
        'last_message_time_sec': 9.8,
        'rssi': 80,
        'channels': (1500, 1500, 1500, 1500),
        'timeout_sec': 0.8,
        'minimum_rssi': 1,
        'throttle_channel': 3,
        'steering_channel': 1,
        'neutral_pwm': 1500,
        'neutral_tolerance_pwm': 120,
    }
    values.update(overrides)
    return rc_channels_ready_for_handoff(**values)


def test_rc_handoff_requires_fresh_link_and_neutral_critical_sticks():
    assert rc_ready()
    assert not rc_ready(last_message_time_sec=8.0)
    assert not rc_ready(rssi=0)
    assert rc_ready(rssi=255)
    assert not rc_ready(rssi=255, allow_unknown_rssi=False)
    assert not rc_ready(channels=(1700, 1500, 1500, 1500))
    assert not rc_ready(channels=(1500, 1500, 1700, 1500))
    assert not rc_ready(channels=(1500, 1500))
    assert not rc_ready(channels=(1500, 1500, 65535, 1500))
