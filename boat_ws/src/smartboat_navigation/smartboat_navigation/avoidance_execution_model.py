"""Pure mode-aware state machine for production obstacle execution."""

import math
from dataclasses import dataclass
from enum import Enum


class ExecutionPhase(str, Enum):
    INACTIVE = 'INACTIVE'
    CLEAR_HOLD = 'CLEAR_HOLD'
    MISSION_SLOW = 'MISSION_SLOW'
    MISSION_DETOUR = 'MISSION_DETOUR'
    STOP = 'STOP'
    MANUAL_SLOW = 'MANUAL_SLOW'
    MANUAL_BRAKE = 'MANUAL_BRAKE'
    MANUAL_REVERSE = 'MANUAL_REVERSE'
    MANUAL_HOLD = 'MANUAL_HOLD'
    MANUAL_LOCKOUT = 'MANUAL_LOCKOUT'
    LOCALIZATION_FALLBACK = 'LOCALIZATION_FALLBACK'


@dataclass(frozen=True)
class ExecutionParameters:
    mission_slow_speed_mps: float = 0.20
    mission_detour_speed_mps: float = 0.25
    mission_detour_yaw_rate_rad_s: float = 0.35
    manual_slow_speed_mps: float = 0.15
    manual_stop_speed_mps: float = 0.20
    manual_slow_max_sec: float = 1.5
    manual_brake_hold_sec: float = 0.5
    manual_brake_max_sec: float = 2.5
    manual_reverse_speed_mps: float = 0.25
    manual_reverse_duration_sec: float = 1.0
    manual_reverse_enabled: bool = False
    manual_takeover_max_sec: float = 5.0
    clear_hold_sec: float = 1.0

    def validate(self):
        positive = (
            self.mission_slow_speed_mps,
            self.mission_detour_speed_mps,
            self.mission_detour_yaw_rate_rad_s,
            self.manual_slow_speed_mps,
            self.manual_stop_speed_mps,
            self.manual_slow_max_sec,
            self.manual_brake_hold_sec,
            self.manual_brake_max_sec,
            self.manual_reverse_speed_mps,
            self.manual_reverse_duration_sec,
            self.manual_takeover_max_sec,
            self.clear_hold_sec,
        )
        if any(
            not math.isfinite(float(value)) or value <= 0.0
            for value in positive
        ):
            raise ValueError(
                'avoidance execution parameters must be finite and positive'
            )
        if self.manual_brake_hold_sec > self.manual_brake_max_sec:
            raise ValueError('manual brake hold cannot exceed brake maximum')


@dataclass(frozen=True)
class ExecutionOutput:
    active: bool
    linear_x: float
    angular_z: float
    phase: ExecutionPhase
    reason: str
    manual_fallback_requested: bool = False


class AvoidanceExecutionModel:
    """Convert supervisor state into conservative mode-specific commands."""

    HAZARD_STATES = {
        'RADAR_SLOWDOWN',
        'CAMERA_AVOID',
        'EMERGENCY_STOP',
        'SENSOR_FAULT',
    }
    MANUAL_PHASES = {
        ExecutionPhase.MANUAL_SLOW,
        ExecutionPhase.MANUAL_BRAKE,
        ExecutionPhase.MANUAL_REVERSE,
        ExecutionPhase.MANUAL_HOLD,
    }

    def __init__(self, parameters: ExecutionParameters):
        parameters.validate()
        self.parameters = parameters
        self._manual_timeout_latched = False
        self._localization_fallback_latched = False
        self._localization_handoff_complete_latched = False
        self._control_context_mode = None
        self.reset()

    def reset(self, *, preserve_control_context=False):
        self._phase = ExecutionPhase.INACTIVE
        self._phase_started_sec = None
        self._clear_started_sec = None
        self._last_mode = None
        self._manual_session_started_sec = None
        self._stopped_since_sec = None
        if not preserve_control_context:
            self._control_context_mode = None

    @staticmethod
    def _output(
        active,
        linear_x,
        angular_z,
        phase,
        reason,
        manual_fallback_requested=False,
    ):
        return ExecutionOutput(
            bool(active),
            float(linear_x),
            float(angular_z),
            phase,
            reason,
            bool(manual_fallback_requested),
        )

    def _set_phase(self, phase, now_sec):
        if phase is not self._phase:
            self._phase = phase
            self._phase_started_sec = float(now_sec)
            self._stopped_since_sec = None

    def _inactive(self, reason, *, preserve_control_context=False):
        self.reset(preserve_control_context=preserve_control_context)
        return self._output(
            False,
            0.0,
            0.0,
            ExecutionPhase.INACTIVE,
            reason,
        )

    def step(
        self,
        *,
        now_sec,
        flight_mode,
        armed,
        assessment_state,
        ground_speed_mps,
        recommended_yaw_rate_rad_s,
        ground_speed_valid,
        inputs_fresh,
        manual_handoff_complete=False,
    ):
        now = float(now_sec)
        if not math.isfinite(now):
            raise ValueError('now_sec must be finite')
        reported_mode = str(flight_mode).upper()
        if reported_mode in ('AUTO.MISSION', 'MANUAL'):
            self._control_context_mode = reported_mode
            mode = reported_mode
        elif (
            reported_mode in ('AUTO.LOITER', 'OFFBOARD')
            and self._control_context_mode in ('AUTO.MISSION', 'MANUAL')
        ):
            # The bridge first requests HOLD while it warms up PX4 Offboard
            # setpoints. Keep producing the same avoidance command until the
            # bridge returns to the origin mode or the operator selects MANUAL.
            mode = self._control_context_mode
        else:
            mode = reported_mode
        state = str(assessment_state).upper()
        speed = float(ground_speed_mps)
        yaw = float(recommended_yaw_rate_rad_s)
        if not math.isfinite(speed) or speed < 0.0:
            ground_speed_valid = False
            speed = 0.0
        if not math.isfinite(yaw):
            yaw = 0.0

        if not armed:
            self._manual_timeout_latched = False
            self._localization_fallback_latched = False
            self._localization_handoff_complete_latched = False
            return self._inactive('px4_disarmed')

        if ground_speed_valid:
            self._localization_handoff_complete_latched = False
        elif (
            mode == 'MANUAL'
            and self._localization_handoff_complete_latched
        ):
            return self._inactive('manual_fallback_complete_hold')
        else:
            self._localization_fallback_latched = True

        if self._localization_fallback_latched:
            if bool(manual_handoff_complete) and mode == 'MANUAL':
                self._localization_fallback_latched = False
                self._localization_handoff_complete_latched = True
                return self._inactive('manual_fallback_complete')
            self._set_phase(ExecutionPhase.LOCALIZATION_FALLBACK, now)
            return self._output(
                True,
                0.0,
                0.0,
                self._phase,
                'localization_invalid_neutral_then_manual',
                manual_fallback_requested=True,
            )

        if mode not in ('AUTO.MISSION', 'MANUAL'):
            return self._inactive(
                f'unsupported_flight_mode:{reported_mode}'
            )

        if mode == 'MANUAL' and self._manual_timeout_latched:
            self.reset()
            self._phase = ExecutionPhase.MANUAL_LOCKOUT
            return self._output(
                False,
                0.0,
                0.0,
                self._phase,
                'manual_timeout_wait_disarm',
            )

        if self._last_mode != mode:
            self._phase = ExecutionPhase.INACTIVE
            self._phase_started_sec = None
            self._clear_started_sec = None
            self._last_mode = mode

        if not inputs_fresh:
            self._clear_started_sec = None
            self._set_phase(ExecutionPhase.STOP, now)
            return self._output(True, 0.0, 0.0, self._phase, 'inputs_stale')

        if state == 'CLEAR':
            if self._phase is ExecutionPhase.INACTIVE:
                return self._output(
                    False,
                    0.0,
                    0.0,
                    self._phase,
                    'path_clear',
                )
            if self._clear_started_sec is None:
                self._clear_started_sec = now
            if (
                now - self._clear_started_sec
            ) < self.parameters.clear_hold_sec:
                self._set_phase(ExecutionPhase.CLEAR_HOLD, now)
                return self._output(
                    True,
                    0.0,
                    0.0,
                    self._phase,
                    'clear_hysteresis',
                )
            return self._inactive('path_clear_stable')

        self._clear_started_sec = None
        if state not in self.HAZARD_STATES:
            self._set_phase(ExecutionPhase.STOP, now)
            return self._output(
                True,
                0.0,
                0.0,
                self._phase,
                f'unknown_assessment_state:{state}',
            )

        if mode == 'AUTO.MISSION':
            return self._mission_step(state, yaw, now)
        return self._manual_step(state, speed, now)

    def _mission_step(self, state, recommended_yaw, now):
        if state in ('SENSOR_FAULT', 'EMERGENCY_STOP'):
            self._set_phase(ExecutionPhase.STOP, now)
            return self._output(
                True,
                0.0,
                0.0,
                self._phase,
                state.lower(),
            )
        if state == 'RADAR_SLOWDOWN':
            self._set_phase(ExecutionPhase.MISSION_SLOW, now)
            return self._output(
                True,
                self.parameters.mission_slow_speed_mps,
                0.0,
                self._phase,
                'mission_radar_slowdown',
            )

        yaw_limit = self.parameters.mission_detour_yaw_rate_rad_s
        yaw = max(-yaw_limit, min(yaw_limit, float(recommended_yaw)))
        if abs(yaw) < 1e-6:
            self._set_phase(ExecutionPhase.STOP, now)
            return self._output(
                True,
                0.0,
                0.0,
                self._phase,
                'mission_detour_direction_unavailable',
            )
        self._set_phase(ExecutionPhase.MISSION_DETOUR, now)
        return self._output(
            True,
            self.parameters.mission_detour_speed_mps,
            yaw,
            self._phase,
            'mission_camera_detour',
        )

    def _manual_step(self, state, speed, now):
        if self._manual_session_started_sec is None:
            self._manual_session_started_sec = now
        if (
            now - self._manual_session_started_sec
        ) >= self.parameters.manual_takeover_max_sec:
            self._manual_timeout_latched = True
            self.reset()
            self._phase = ExecutionPhase.MANUAL_LOCKOUT
            return self._output(
                False,
                0.0,
                0.0,
                self._phase,
                'manual_takeover_timeout_wait_disarm',
            )

        if state == 'SENSOR_FAULT':
            self._set_phase(ExecutionPhase.STOP, now)
            return self._output(
                True,
                0.0,
                0.0,
                self._phase,
                'manual_sensor_fault_stop_only',
            )

        if self._phase not in self.MANUAL_PHASES:
            initial = (
                ExecutionPhase.MANUAL_SLOW
                if speed > self.parameters.manual_stop_speed_mps
                else ExecutionPhase.MANUAL_BRAKE
            )
            self._set_phase(initial, now)

        elapsed = now - float(self._phase_started_sec)
        if self._phase is ExecutionPhase.MANUAL_SLOW:
            if (
                speed <= self.parameters.manual_slow_speed_mps
                or elapsed >= self.parameters.manual_slow_max_sec
            ):
                self._set_phase(ExecutionPhase.MANUAL_BRAKE, now)
            else:
                return self._output(
                    True,
                    0.0,
                    0.0,
                    self._phase,
                    'manual_collision_neutral_deceleration',
                )

        elapsed = now - float(self._phase_started_sec)
        if self._phase is ExecutionPhase.MANUAL_BRAKE:
            if speed < self.parameters.manual_stop_speed_mps:
                if self._stopped_since_sec is None:
                    self._stopped_since_sec = now
            else:
                self._stopped_since_sec = None
            stopped_long_enough = (
                self._stopped_since_sec is not None
                and (now - self._stopped_since_sec)
                >= self.parameters.manual_brake_hold_sec
            )
            if stopped_long_enough:
                next_phase = (
                    ExecutionPhase.MANUAL_REVERSE
                    if self.parameters.manual_reverse_enabled
                    else ExecutionPhase.MANUAL_HOLD
                )
                self._set_phase(next_phase, now)
            elif elapsed >= self.parameters.manual_brake_max_sec:
                self._set_phase(ExecutionPhase.MANUAL_HOLD, now)
            else:
                return self._output(
                    True,
                    0.0,
                    0.0,
                    self._phase,
                    'manual_collision_braking',
                )

        elapsed = now - float(self._phase_started_sec)
        if self._phase is ExecutionPhase.MANUAL_REVERSE:
            if elapsed < self.parameters.manual_reverse_duration_sec:
                return self._output(
                    True,
                    -self.parameters.manual_reverse_speed_mps,
                    0.0,
                    self._phase,
                    'manual_collision_reverse',
                )
            self._set_phase(ExecutionPhase.MANUAL_HOLD, now)

        return self._output(
            True,
            0.0,
            0.0,
            self._phase,
            'manual_collision_hold',
        )
