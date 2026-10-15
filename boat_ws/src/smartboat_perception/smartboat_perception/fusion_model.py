"""Pure EWaSR mask and aligned-depth fusion calculations."""

from dataclasses import dataclass

import cv2
import numpy as np


GO_STRAIGHT = "GO_STRAIGHT"
TURN_LEFT = "TURN_LEFT"
TURN_RIGHT = "TURN_RIGHT"


@dataclass(frozen=True)
class FusionParameters:
    minimum_valid_depth_m: float = 0.15
    maximum_valid_depth_m: float = 3.0
    avoidance_distance_m: float = 2.8
    minimum_obstacle_pixels: int = 200
    maximum_depth_samples: int = 3000
    near_distance_percentile: float = 5.0
    maximum_turn_obstacle_fraction: float = 0.25

    def validate(self):
        finite_values = (
            self.minimum_valid_depth_m,
            self.maximum_valid_depth_m,
            self.avoidance_distance_m,
            self.near_distance_percentile,
            self.maximum_turn_obstacle_fraction,
        )
        if not all(np.isfinite(float(value)) for value in finite_values):
            raise ValueError("fusion floating-point parameters must be finite")
        if self.minimum_valid_depth_m < 0.0:
            raise ValueError("minimum_valid_depth_m cannot be negative")
        if self.maximum_valid_depth_m <= self.minimum_valid_depth_m:
            raise ValueError("maximum_valid_depth_m must exceed the minimum")
        if not (
            self.minimum_valid_depth_m
            < self.avoidance_distance_m
            <= self.maximum_valid_depth_m
        ):
            raise ValueError("avoidance_distance_m must be inside the depth range")
        if self.minimum_obstacle_pixels <= 0:
            raise ValueError("minimum_obstacle_pixels must be positive")
        if self.maximum_depth_samples <= 0:
            raise ValueError("maximum_depth_samples must be positive")
        if not 0.0 <= self.near_distance_percentile <= 50.0:
            raise ValueError("near_distance_percentile must be in [0, 50]")
        if not 0.0 <= self.maximum_turn_obstacle_fraction < 0.5:
            raise ValueError(
                "maximum_turn_obstacle_fraction must be in [0, 0.5)"
            )


@dataclass(frozen=True)
class FusionResult:
    obstacle_detected: bool
    distance_m: float
    decision: str
    obstacle_pixels: int
    reason: str


def analyze_fusion(depth_m, mask, obstacle_class, parameters, rng=None):
    """Fuse an EWaSR class mask with an aligned depth image in metres."""
    parameters.validate()
    depth = np.asarray(depth_m, dtype=np.float32)
    if depth.ndim != 2:
        raise ValueError("depth_m must be a two-dimensional image")

    mask_big = cv2.resize(
        np.asarray(mask, dtype=np.uint8),
        (depth.shape[1], depth.shape[0]),
        interpolation=cv2.INTER_NEAREST,
    )
    obstacle_area = mask_big == int(obstacle_class)
    ys, xs = np.where(obstacle_area)
    obstacle_pixels = int(xs.size)
    if obstacle_pixels < parameters.minimum_obstacle_pixels:
        return FusionResult(
            False,
            -1.0,
            GO_STRAIGHT,
            obstacle_pixels,
            "vision_no_obstacle",
        )

    if xs.size > parameters.maximum_depth_samples:
        if rng is None:
            rng = np.random.default_rng()
        sample = rng.choice(
            xs.size,
            size=parameters.maximum_depth_samples,
            replace=False,
        )
        xs_sample = xs[sample]
        ys_sample = ys[sample]
    else:
        xs_sample = xs
        ys_sample = ys

    samples = depth[ys_sample, xs_sample]
    valid = (
        np.isfinite(samples)
        & (samples > parameters.minimum_valid_depth_m)
        & (samples < parameters.maximum_valid_depth_m)
    )
    if not np.any(valid):
        return FusionResult(
            False,
            -1.0,
            GO_STRAIGHT,
            obstacle_pixels,
            "vision_only_depth_invalid_ignore",
        )

    valid_depths = samples[valid]
    valid_xs = xs_sample[valid]
    # A median can hide a small, very close obstacle behind a large farther
    # region of the same segmentation class. A low percentile remains robust
    # to isolated depth noise while preserving close safety hazards.
    distance_m = float(np.percentile(
        valid_depths,
        parameters.near_distance_percentile,
    ))
    if distance_m >= parameters.avoidance_distance_m:
        return FusionResult(
            False,
            distance_m,
            GO_STRAIGHT,
            obstacle_pixels,
            "obstacle_far_enough",
        )

    obstacle_center_x = float(np.median(valid_xs))
    width = depth.shape[1]
    if obstacle_center_x < width / 3.0:
        decision = TURN_RIGHT
        reason = "obstacle_on_left_turn_right"
    elif obstacle_center_x > width * 2.0 / 3.0:
        decision = TURN_LEFT
        reason = "obstacle_on_right_turn_left"
    else:
        left_count = int(np.sum(obstacle_area[:, : width // 2]))
        right_count = int(np.sum(obstacle_area[:, width // 2:]))
        if left_count <= right_count:
            decision = TURN_LEFT
            reason = "obstacle_center_left_side_freer"
        else:
            decision = TURN_RIGHT
            reason = "obstacle_center_right_side_freer"

    half = max(1, width // 2)
    if decision == TURN_LEFT:
        turn_area = obstacle_area[:, :half]
    else:
        turn_area = obstacle_area[:, width - half:]
    obstacle_fraction = float(np.mean(turn_area))
    if obstacle_fraction > parameters.maximum_turn_obstacle_fraction:
        return FusionResult(
            True,
            distance_m,
            GO_STRAIGHT,
            obstacle_pixels,
            "obstacle_detected_no_safe_turn_corridor",
        )

    return FusionResult(True, distance_m, decision, obstacle_pixels, reason)
