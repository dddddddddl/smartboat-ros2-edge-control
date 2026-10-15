import numpy as np
import pytest

from smartboat_perception.fusion_model import (
    FusionParameters,
    GO_STRAIGHT,
    TURN_LEFT,
    TURN_RIGHT,
    analyze_fusion,
)


PARAMETERS = FusionParameters(
    minimum_valid_depth_m=0.15,
    maximum_valid_depth_m=3.0,
    avoidance_distance_m=2.8,
    minimum_obstacle_pixels=200,
    maximum_depth_samples=3000,
)


def scene(x_start, x_stop, depth=1.5):
    mask = np.ones((48, 64), dtype=np.uint8)
    mask[10:35, x_start:x_stop] = 0
    depth_m = np.zeros((48, 64), dtype=np.float32)
    depth_m[10:35, x_start:x_stop] = depth
    return depth_m, mask


def test_no_obstacle_pixels_goes_straight():
    depth_m = np.zeros((48, 64), dtype=np.float32)
    mask = np.ones((48, 64), dtype=np.uint8)
    result = analyze_fusion(depth_m, mask, 0, PARAMETERS)
    assert not result.obstacle_detected
    assert result.decision == GO_STRAIGHT
    assert result.reason == "vision_no_obstacle"


def test_invalid_depth_is_ignored():
    depth_m, mask = scene(2, 18, depth=0.0)
    result = analyze_fusion(depth_m, mask, 0, PARAMETERS)
    assert not result.obstacle_detected
    assert result.distance_m == -1.0
    assert result.reason == "vision_only_depth_invalid_ignore"


def test_far_obstacle_does_not_request_turn():
    depth_m, mask = scene(2, 18, depth=2.9)
    result = analyze_fusion(depth_m, mask, 0, PARAMETERS)
    assert not result.obstacle_detected
    assert result.decision == GO_STRAIGHT
    assert result.reason == "obstacle_far_enough"


@pytest.mark.parametrize(
    ("x_start", "x_stop", "expected"),
    ((2, 18, TURN_RIGHT), (46, 62, TURN_LEFT)),
)
def test_side_obstacle_turns_away(x_start, x_stop, expected):
    depth_m, mask = scene(x_start, x_stop)
    result = analyze_fusion(depth_m, mask, 0, PARAMETERS)
    assert result.obstacle_detected
    assert result.decision == expected


def test_parameters_reject_invalid_depth_range():
    with pytest.raises(ValueError):
        FusionParameters(
            minimum_valid_depth_m=3.0,
            maximum_valid_depth_m=3.0,
        ).validate()
