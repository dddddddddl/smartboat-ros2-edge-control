import pytest

from smartboat_navigation.avoidance_execution_model import (
    AvoidanceExecutionModel,
    ExecutionParameters,
    ExecutionPhase,
)


@pytest.fixture
def model():
    return AvoidanceExecutionModel(ExecutionParameters())


def step(model, now, **overrides):
    values = {
        'flight_mode': 'AUTO.MISSION',
        'armed': True,
        'assessment_state': 'CLEAR',
        'ground_speed_mps': 0.7,
        'recommended_yaw_rate_rad_s': 0.35,
        'ground_speed_valid': True,
        'inputs_fresh': True,
    }
    values.update(overrides)
    return model.step(now_sec=now, **values)


def test_clear_mission_does_not_take_control(model):
    output = step(model, 0.0)
    assert not output.active
    assert output.phase is ExecutionPhase.INACTIVE


def test_mission_radar_slowdown_and_camera_detour(model):
    slowdown = step(model, 0.0, assessment_state='RADAR_SLOWDOWN')
    assert slowdown.active
    assert slowdown.phase is ExecutionPhase.MISSION_SLOW
    assert slowdown.linear_x == pytest.approx(0.20)

    detour = step(
        model,
        0.1,
        assessment_state='CAMERA_AVOID',
        recommended_yaw_rate_rad_s=-0.8,
    )
    assert detour.phase is ExecutionPhase.MISSION_DETOUR
    assert detour.linear_x == pytest.approx(0.25)
    assert detour.angular_z == pytest.approx(-0.35)


def test_mission_avoidance_survives_hold_and_offboard_transition(model):
    initial = step(model, 0.0, assessment_state='RADAR_SLOWDOWN')
    hold = step(
        model,
        0.1,
        flight_mode='AUTO.LOITER',
        assessment_state='RADAR_SLOWDOWN',
    )
    offboard = step(
        model,
        0.2,
        flight_mode='OFFBOARD',
        assessment_state='CAMERA_AVOID',
    )

    assert initial.phase is ExecutionPhase.MISSION_SLOW
    assert hold.phase is ExecutionPhase.MISSION_SLOW
    assert offboard.phase is ExecutionPhase.MISSION_DETOUR


def test_offboard_without_known_origin_never_takes_control(model):
    output = step(
        model,
        0.0,
        flight_mode='OFFBOARD',
        assessment_state='RADAR_SLOWDOWN',
    )
    assert not output.active
    assert output.reason == 'unsupported_flight_mode:OFFBOARD'


@pytest.mark.parametrize('state', ['SENSOR_FAULT', 'EMERGENCY_STOP'])
def test_mission_fault_or_emergency_stops(model, state):
    output = step(model, 0.0, assessment_state=state)
    assert output.active
    assert output.phase is ExecutionPhase.STOP
    assert output.linear_x == 0.0


def test_camera_without_safe_direction_stops(model):
    output = step(
        model,
        0.0,
        assessment_state='CAMERA_AVOID',
        recommended_yaw_rate_rad_s=0.0,
    )
    assert output.phase is ExecutionPhase.STOP


def test_manual_slow_brake_reverse_hold_sequence(model):
    model = AvoidanceExecutionModel(ExecutionParameters(
        manual_reverse_enabled=True,
    ))
    slow = step(
        model,
        0.0,
        flight_mode='MANUAL',
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.8,
    )
    assert slow.phase is ExecutionPhase.MANUAL_SLOW
    assert slow.linear_x == 0.0

    brake = step(
        model,
        0.2,
        flight_mode='MANUAL',
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.05,
    )
    assert brake.phase is ExecutionPhase.MANUAL_BRAKE
    assert brake.linear_x == 0.0

    reverse = step(
        model,
        0.8,
        flight_mode='MANUAL',
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.0,
    )
    assert reverse.phase is ExecutionPhase.MANUAL_REVERSE
    assert reverse.linear_x == pytest.approx(-0.25)

    hold = step(
        model,
        1.9,
        flight_mode='MANUAL',
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.0,
    )
    assert hold.phase is ExecutionPhase.MANUAL_HOLD
    assert hold.linear_x == 0.0


def test_manual_sensor_fault_never_reverses(model):
    for now in (0.0, 1.0, 4.9):
        output = step(
            model,
            now,
            flight_mode='MANUAL',
            assessment_state='SENSOR_FAULT',
            ground_speed_mps=0.0,
        )
        assert output.phase is ExecutionPhase.STOP
        assert output.linear_x == 0.0


def test_invalid_localization_requests_manual_fallback(model):
    output = step(
        model,
        0.0,
        flight_mode='MANUAL',
        assessment_state='SENSOR_FAULT',
        ground_speed_valid=False,
        inputs_fresh=False,
    )

    assert output.active
    assert output.phase is ExecutionPhase.LOCALIZATION_FALLBACK
    assert output.linear_x == 0.0
    assert output.manual_fallback_requested

    complete = step(
        model,
        0.1,
        flight_mode='MANUAL',
        ground_speed_valid=False,
        inputs_fresh=False,
        manual_handoff_complete=True,
    )
    assert not complete.active
    assert complete.reason == 'manual_fallback_complete'

    held = step(
        model,
        0.2,
        flight_mode='MANUAL',
        ground_speed_valid=False,
        inputs_fresh=False,
    )
    assert not held.active
    assert held.reason == 'manual_fallback_complete_hold'


def test_default_manual_collision_sequence_never_reverses(model):
    step(
        model,
        0.0,
        flight_mode='MANUAL',
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.5,
    )
    step(
        model,
        0.2,
        flight_mode='MANUAL',
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.0,
    )
    output = step(
        model,
        0.8,
        flight_mode='MANUAL',
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.0,
    )
    assert output.phase is ExecutionPhase.MANUAL_HOLD
    assert output.linear_x == 0.0


@pytest.mark.parametrize('bad_speed', [float('nan'), float('inf'), -0.1])
def test_invalid_numeric_speed_fails_into_localization_fallback(
    model,
    bad_speed,
):
    output = step(model, 0.0, ground_speed_mps=bad_speed)
    assert output.active
    assert output.phase is ExecutionPhase.LOCALIZATION_FALLBACK
    assert output.linear_x == 0.0


def test_manual_never_reverses_until_stopped_is_confirmed(model):
    step(
        model,
        0.0,
        flight_mode='MANUAL',
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.8,
    )
    braking = step(
        model,
        4.0,
        flight_mode='MANUAL',
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.8,
    )

    assert braking.active
    assert braking.phase is ExecutionPhase.MANUAL_BRAKE
    assert braking.linear_x == 0.0


def test_manual_stop_threshold_is_strictly_below_point_two():
    parameters = ExecutionParameters(manual_reverse_enabled=False)
    assert parameters.manual_stop_speed_mps == pytest.approx(0.2)
    model = AvoidanceExecutionModel(parameters)

    at_threshold = step(
        model,
        0.0,
        flight_mode='MANUAL',
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.2,
    )
    still_braking = step(
        model,
        1.0,
        flight_mode='MANUAL',
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.2,
    )
    below_threshold = step(
        model,
        1.1,
        flight_mode='MANUAL',
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.199,
    )
    stopped = step(
        model,
        1.7,
        flight_mode='MANUAL',
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.199,
    )

    assert at_threshold.phase is ExecutionPhase.MANUAL_BRAKE
    assert still_braking.phase is ExecutionPhase.MANUAL_BRAKE
    assert below_threshold.phase is ExecutionPhase.MANUAL_BRAKE
    assert stopped.phase is ExecutionPhase.MANUAL_HOLD


def test_manual_timeout_releases_rc_until_disarm_rearm_cycle(model):
    first = step(
        model,
        0.0,
        flight_mode='MANUAL',
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.8,
    )
    timed_out = step(
        model,
        5.0,
        flight_mode='MANUAL',
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.0,
    )
    still_locked = step(
        model,
        8.0,
        flight_mode='MANUAL',
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.0,
    )

    assert first.active
    assert not timed_out.active
    assert timed_out.phase is ExecutionPhase.MANUAL_LOCKOUT
    assert not still_locked.active

    disarmed = step(
        model,
        9.0,
        flight_mode='MANUAL',
        armed=False,
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.0,
    )
    rearmed = step(
        model,
        10.0,
        flight_mode='MANUAL',
        armed=True,
        assessment_state='RADAR_SLOWDOWN',
        ground_speed_mps=0.0,
    )

    assert not disarmed.active
    assert rearmed.active
    assert rearmed.phase is ExecutionPhase.MANUAL_BRAKE


def test_clear_hysteresis_holds_stop_before_release(model):
    step(model, 0.0, assessment_state='CAMERA_AVOID')
    holding = step(model, 0.5, assessment_state='CLEAR')
    assert holding.active
    assert holding.phase is ExecutionPhase.CLEAR_HOLD

    released = step(model, 1.6, assessment_state='CLEAR')
    assert not released.active


def test_stale_inputs_fail_closed_in_supported_mode(model):
    output = step(model, 0.0, inputs_fresh=False)
    assert output.active
    assert output.phase is ExecutionPhase.STOP


def test_disarmed_or_unsupported_mode_never_takes_control(model):
    assert not step(model, 0.0, armed=False).active
    assert not step(model, 0.1, flight_mode='ALTCTL').active
