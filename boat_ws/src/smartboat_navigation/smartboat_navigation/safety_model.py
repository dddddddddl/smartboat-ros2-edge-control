"""Pure safety calculations used by the local avoidance supervisor."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional


class AvoidanceState(str, Enum):
    SENSOR_FAULT = 'SENSOR_FAULT'
    CLEAR = 'CLEAR'
    RADAR_SLOWDOWN = 'RADAR_SLOWDOWN'
    CAMERA_AVOID = 'CAMERA_AVOID'
    EMERGENCY_STOP = 'EMERGENCY_STOP'


@dataclass(frozen=True)
class SafetyParameters:
    cruise_speed_mps: float = 1.5
    avoidance_speed_mps: float = 0.7
    braking_reference_speed_mps: float = 1.506
    braking_reference_distance_m: float = 2.963
    perception_latency_sec: float = 0.2
    actuation_latency_sec: float = 0.1
    stopping_reserve_m: float = 0.5
    radar_slowdown_distance_m: float = 6.0
    d435_reliable_distance_m: float = 3.0
    radar_emergency_ttc_sec: float = 2.5
    radar_slowdown_ttc_sec: float = 5.0

    def validate(self):
        positive_values = (
            self.cruise_speed_mps,
            self.avoidance_speed_mps,
            self.braking_reference_speed_mps,
            self.braking_reference_distance_m,
            self.radar_slowdown_distance_m,
            self.d435_reliable_distance_m,
            self.radar_emergency_ttc_sec,
            self.radar_slowdown_ttc_sec,
        )
        if any(
            not math.isfinite(float(value)) or value <= 0.0
            for value in positive_values
        ):
            raise ValueError(
                'speed and distance parameters must be finite and positive'
            )
        if self.avoidance_speed_mps > self.cruise_speed_mps:
            raise ValueError('avoidance speed cannot exceed cruise speed')
        if self.radar_emergency_ttc_sec >= self.radar_slowdown_ttc_sec:
            raise ValueError('radar emergency TTC must be below slowdown TTC')
        if not all(math.isfinite(float(value)) for value in (
            self.perception_latency_sec,
            self.actuation_latency_sec,
            self.stopping_reserve_m,
        )):
            raise ValueError('latency and reserve parameters must be finite')
        if (
            self.perception_latency_sec < 0.0
            or self.actuation_latency_sec < 0.0
        ):
            raise ValueError('latency parameters cannot be negative')
        if self.stopping_reserve_m < 0.0:
            raise ValueError('stopping reserve cannot be negative')


@dataclass(frozen=True)
class SafetyAssessment:
    state: AvoidanceState
    speed_limit_mps: float
    emergency_stop: bool
    dynamic_stopping_distance_m: float
    closest_obstacle_distance_m: Optional[float]
    reason: str


class SafetyModel:
    def __init__(self, parameters: SafetyParameters):
        parameters.validate()
        self.parameters = parameters

    def dynamic_stopping_distance(self, speed_mps):
        """Return empirical braking distance plus latency and fixed reserve."""
        speed = float(speed_mps)
        if not math.isfinite(speed) or speed < 0.0:
            speed = self.parameters.cruise_speed_mps
        params = self.parameters
        ratio = speed / params.braking_reference_speed_mps
        empirical_distance = (
            params.braking_reference_distance_m * ratio * ratio
        )
        latency_distance = speed * (
            params.perception_latency_sec
            + params.actuation_latency_sec
        )
        return (
            empirical_distance
            + latency_distance
            + params.stopping_reserve_m
        )

    @staticmethod
    def yaw_rate_for_decision(state, decision, yaw_rate_limit_rad_s):
        """Convert a camera avoidance decision to ROS yaw-rate convention."""
        if state is not AvoidanceState.CAMERA_AVOID:
            return 0.0
        limit = abs(float(yaw_rate_limit_rad_s))
        if not math.isfinite(limit):
            return 0.0
        if decision == 'TURN_LEFT':
            return limit
        if decision == 'TURN_RIGHT':
            return -limit
        return 0.0

    @staticmethod
    def _valid_distance(distance):
        if distance is None:
            return None
        value = float(distance)
        if not math.isfinite(value) or value <= 0.0:
            return None
        return value

    def assess(
        self,
        *,
        speed_mps,
        ground_speed_fresh,
        radar_distance_m,
        radar_fresh,
        radar_required,
        camera_confirmed,
        camera_distance_m,
        camera_fresh,
        radar_ttc_sec=None,
    ):
        speed = float(speed_mps)
        if not math.isfinite(speed) or speed < 0.0:
            ground_speed_fresh = False
            speed = self.parameters.cruise_speed_mps
        stopping_distance = self.dynamic_stopping_distance(speed)
        radar_distance = self._valid_distance(radar_distance_m)
        camera_distance = self._valid_distance(camera_distance_m)
        radar_ttc = self._valid_distance(radar_ttc_sec)

        if not ground_speed_fresh:
            return SafetyAssessment(
                AvoidanceState.SENSOR_FAULT,
                0.0,
                True,
                stopping_distance,
                None,
                'ground_speed_stale',
            )

        if radar_required and not radar_fresh:
            return SafetyAssessment(
                AvoidanceState.SENSOR_FAULT,
                0.0,
                True,
                stopping_distance,
                camera_distance,
                'radar_required_but_stale',
            )

        if not camera_fresh:
            return SafetyAssessment(
                AvoidanceState.SENSOR_FAULT,
                0.0,
                True,
                stopping_distance,
                radar_distance,
                'camera_fusion_stale',
            )

        if (
            radar_ttc is not None
            and radar_ttc <= self.parameters.radar_emergency_ttc_sec
        ):
            return SafetyAssessment(
                AvoidanceState.EMERGENCY_STOP,
                0.0,
                True,
                stopping_distance,
                radar_distance,
                'radar_ttc_emergency',
            )

        distances = [
            distance for distance in (radar_distance, camera_distance)
            if distance is not None
        ]
        closest = min(distances) if distances else None

        if closest is not None and closest <= stopping_distance:
            return SafetyAssessment(
                AvoidanceState.EMERGENCY_STOP,
                0.0,
                True,
                stopping_distance,
                closest,
                'obstacle_inside_dynamic_stopping_distance',
            )

        if (
            camera_confirmed
            and camera_distance is not None
            and camera_distance <= self.parameters.d435_reliable_distance_m
        ):
            return SafetyAssessment(
                AvoidanceState.CAMERA_AVOID,
                self.parameters.avoidance_speed_mps,
                False,
                stopping_distance,
                closest,
                'camera_confirmed_obstacle',
            )

        if (
            (
                radar_distance is not None
                and radar_distance <= self.parameters.radar_slowdown_distance_m
            )
            or (
                radar_ttc is not None
                and radar_ttc <= self.parameters.radar_slowdown_ttc_sec
            )
        ):
            return SafetyAssessment(
                AvoidanceState.RADAR_SLOWDOWN,
                self.parameters.avoidance_speed_mps,
                False,
                stopping_distance,
                closest,
                'radar_obstacle_slowdown',
            )

        return SafetyAssessment(
            AvoidanceState.CLEAR,
            self.parameters.cruise_speed_mps,
            False,
            stopping_distance,
            closest,
            'path_clear',
        )
