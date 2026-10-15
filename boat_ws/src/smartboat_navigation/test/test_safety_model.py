import math

import pytest

from smartboat_navigation.safety_model import (
    AvoidanceState,
    SafetyModel,
    SafetyParameters,
)


@pytest.fixture
def model():
    return SafetyModel(SafetyParameters())


def assess(model, **overrides):
    values = {
        'speed_mps': 0.7,
        'ground_speed_fresh': True,
        'radar_distance_m': 8.0,
        'radar_fresh': True,
        'radar_required': True,
        'camera_confirmed': False,
        'camera_distance_m': None,
        'camera_fresh': True,
    }
    values.update(overrides)
    return model.assess(**values)


def test_reference_speed_uses_measured_braking_plus_margins(model):
    distance = model.dynamic_stopping_distance(1.506)
    expected = 2.963 + 1.506 * 0.3 + 0.5
    assert distance == pytest.approx(expected)


def test_avoidance_speed_stopping_distance_is_about_1_35_m(model):
    distance = model.dynamic_stopping_distance(0.7)
    assert distance == pytest.approx(1.35, abs=0.01)


def test_cruise_speed_cannot_stop_inside_d435_reliable_range(model):
    assert model.dynamic_stopping_distance(1.5) > 3.0


def test_clear_path_allows_configured_cruise(model):
    result = assess(model)
    assert result.state is AvoidanceState.CLEAR
    assert result.speed_limit_mps == pytest.approx(1.5)
    assert not result.emergency_stop


def test_radar_causes_early_slowdown(model):
    result = assess(model, radar_distance_m=5.5)
    assert result.state is AvoidanceState.RADAR_SLOWDOWN
    assert result.speed_limit_mps == pytest.approx(0.7)


def test_radar_ttc_can_slow_or_stop_before_distance_threshold(model):
    slowdown = assess(model, radar_distance_m=8.0, radar_ttc_sec=4.0)
    emergency = assess(model, radar_distance_m=8.0, radar_ttc_sec=2.0)
    assert slowdown.state is AvoidanceState.RADAR_SLOWDOWN
    assert emergency.state is AvoidanceState.EMERGENCY_STOP
    assert emergency.emergency_stop


def test_camera_confirmed_obstacle_requests_avoidance(model):
    result = assess(
        model,
        camera_confirmed=True,
        camera_distance_m=2.5,
    )
    assert result.state is AvoidanceState.CAMERA_AVOID
    assert result.speed_limit_mps == pytest.approx(0.7)


def test_obstacle_inside_dynamic_stopping_distance_requests_stop(model):
    result = assess(
        model,
        camera_confirmed=True,
        camera_distance_m=1.0,
    )
    assert result.state is AvoidanceState.EMERGENCY_STOP
    assert result.speed_limit_mps == 0.0
    assert result.emergency_stop


@pytest.mark.parametrize(
    ('overrides', 'reason'),
    [
        ({'ground_speed_fresh': False}, 'ground_speed_stale'),
        ({'radar_fresh': False}, 'radar_required_but_stale'),
        ({'camera_fresh': False}, 'camera_fusion_stale'),
    ],
)
def test_required_stale_inputs_fail_closed(model, overrides, reason):
    result = assess(model, **overrides)
    assert result.state is AvoidanceState.SENSOR_FAULT
    assert result.emergency_stop
    assert result.reason == reason


def test_invalid_distances_are_not_used(model):
    result = assess(
        model,
        radar_distance_m=math.nan,
        radar_required=False,
        camera_distance_m=-1.0,
    )
    assert result.state is AvoidanceState.CLEAR
    assert result.closest_obstacle_distance_m is None


@pytest.mark.parametrize('bad_speed', [math.nan, math.inf, -0.1])
def test_invalid_ground_speed_fails_closed(model, bad_speed):
    result = assess(model, speed_mps=bad_speed)
    assert result.state is AvoidanceState.SENSOR_FAULT
    assert result.emergency_stop
    assert result.reason == 'ground_speed_stale'


def test_fresh_radar_minus_one_means_clear_scan(model):
    result = assess(model, radar_distance_m=-1.0, radar_fresh=True)
    assert result.state is AvoidanceState.CLEAR
    assert not result.emergency_stop


@pytest.mark.parametrize(
    ('decision', 'expected_sign'),
    [('TURN_LEFT', 1.0), ('TURN_RIGHT', -1.0), ('GO_STRAIGHT', 0.0)],
)
def test_camera_decision_uses_ros_yaw_sign(model, decision, expected_sign):
    yaw_rate = model.yaw_rate_for_decision(
        AvoidanceState.CAMERA_AVOID,
        decision,
        0.5,
    )
    assert yaw_rate == pytest.approx(expected_sign * 0.5)
