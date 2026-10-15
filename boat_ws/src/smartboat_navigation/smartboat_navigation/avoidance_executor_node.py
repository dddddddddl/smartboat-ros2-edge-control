"""Production obstacle execution state machine for mission and manual modes."""

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from smartboat_navigation.avoidance_execution_model import (
    AvoidanceExecutionModel,
    ExecutionParameters,
)


class AvoidanceExecutorNode(Node):
    def __init__(self):
        super().__init__('avoidance_executor')

        defaults = ExecutionParameters()
        for name in (
            'mission_slow_speed_mps',
            'mission_detour_speed_mps',
            'mission_detour_yaw_rate_rad_s',
            'manual_slow_speed_mps',
            'manual_stop_speed_mps',
            'manual_slow_max_sec',
            'manual_brake_hold_sec',
            'manual_brake_max_sec',
            'manual_reverse_speed_mps',
            'manual_reverse_duration_sec',
            'manual_takeover_max_sec',
            'clear_hold_sec',
        ):
            self.declare_parameter(name, getattr(defaults, name))
        self.declare_parameter(
            'manual_reverse_enabled',
            defaults.manual_reverse_enabled,
        )

        self.declare_parameter(
            'supervisor_status_topic',
            '/smartboat/navigation/avoidance_supervisor_status',
        )
        self.declare_parameter(
            'recommendation_topic',
            '/smartboat/navigation/avoidance_cmd_vel',
        )
        self.declare_parameter(
            'ground_speed_topic',
            '/smartboat/localization/ground_speed',
        )
        self.declare_parameter(
            'ground_speed_valid_topic',
            '/smartboat/localization/ground_speed_valid',
        )
        self.declare_parameter(
            'flight_mode_topic',
            '/smartboat/px4_bridge/flight_mode',
        )
        self.declare_parameter(
            'armed_topic',
            '/smartboat/px4_bridge/armed',
        )
        self.declare_parameter(
            'output_cmd_vel_topic',
            '/smartboat/control/cmd_vel',
        )
        self.declare_parameter(
            'output_active_topic',
            '/smartboat/control/avoidance_active',
        )
        self.declare_parameter(
            'output_status_topic',
            '/smartboat/control/avoidance_executor_status',
        )
        self.declare_parameter(
            'output_manual_fallback_topic',
            '/smartboat/control/manual_fallback_requested',
        )
        self.declare_parameter(
            'manual_handoff_complete_topic',
            '/smartboat/px4_bridge/manual_handoff_complete',
        )
        self.declare_parameter('input_timeout_sec', 0.8)
        self.declare_parameter('px4_state_timeout_sec', 2.0)
        self.declare_parameter('control_rate_hz', 20.0)

        self.model = AvoidanceExecutionModel(ExecutionParameters(
            mission_slow_speed_mps=self._float('mission_slow_speed_mps'),
            mission_detour_speed_mps=self._float(
                'mission_detour_speed_mps'
            ),
            mission_detour_yaw_rate_rad_s=self._float(
                'mission_detour_yaw_rate_rad_s'
            ),
            manual_slow_speed_mps=self._float('manual_slow_speed_mps'),
            manual_stop_speed_mps=self._float('manual_stop_speed_mps'),
            manual_slow_max_sec=self._float('manual_slow_max_sec'),
            manual_brake_hold_sec=self._float('manual_brake_hold_sec'),
            manual_brake_max_sec=self._float('manual_brake_max_sec'),
            manual_reverse_speed_mps=self._float(
                'manual_reverse_speed_mps'
            ),
            manual_reverse_duration_sec=self._float(
                'manual_reverse_duration_sec'
            ),
            manual_reverse_enabled=bool(
                self.get_parameter('manual_reverse_enabled').value
            ),
            manual_takeover_max_sec=self._float(
                'manual_takeover_max_sec'
            ),
            clear_hold_sec=self._float('clear_hold_sec'),
        ))
        self.input_timeout_sec = self._float('input_timeout_sec')
        self.px4_state_timeout_sec = self._float('px4_state_timeout_sec')
        if (
            not math.isfinite(self.input_timeout_sec)
            or self.input_timeout_sec <= 0.0
        ):
            raise ValueError('input_timeout_sec must be greater than zero')
        if (
            not math.isfinite(self.px4_state_timeout_sec)
            or self.px4_state_timeout_sec <= 0.0
        ):
            raise ValueError('px4_state_timeout_sec must be greater than zero')

        self.assessment_state = 'UNKNOWN'
        self.recommended_yaw_rate = 0.0
        self.ground_speed = 0.0
        self.ground_speed_valid = False
        self.flight_mode = 'UNKNOWN'
        self.armed = False
        self.manual_handoff_complete = False
        self.supervisor_time = None
        self.ground_speed_time = None
        self.ground_speed_valid_time = None
        self.flight_mode_time = None
        self.armed_time = None
        self.manual_handoff_complete_time = None
        self.last_phase = None

        self.create_subscription(
            String,
            self._string('supervisor_status_topic'),
            self._supervisor_callback,
            10,
        )
        self.create_subscription(
            Float32,
            self._string('ground_speed_topic'),
            self._ground_speed_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self._string('ground_speed_valid_topic'),
            self._ground_speed_valid_callback,
            10,
        )
        self.create_subscription(
            String,
            self._string('flight_mode_topic'),
            self._flight_mode_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self._string('armed_topic'),
            self._armed_callback,
            10,
        )
        self.create_subscription(
            Bool,
            self._string('manual_handoff_complete_topic'),
            self._manual_handoff_complete_callback,
            10,
        )
        self.command_pub = self.create_publisher(
            Twist,
            self._string('output_cmd_vel_topic'),
            10,
        )
        self.active_pub = self.create_publisher(
            Bool,
            self._string('output_active_topic'),
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            self._string('output_status_topic'),
            10,
        )
        self.manual_fallback_pub = self.create_publisher(
            Bool,
            self._string('output_manual_fallback_topic'),
            10,
        )

        rate_hz = self._float('control_rate_hz')
        if rate_hz <= 0.0:
            raise ValueError('control_rate_hz must be greater than zero')
        self.create_timer(1.0 / rate_hz, self._control_loop)
        self.get_logger().info(
            'Avoidance executor enabled | mission=slow/detour/stop | '
            'manual=neutral/stop-confirm/reverse/timeout-release'
        )

    def _float(self, name):
        return float(self.get_parameter(name).value)

    def _string(self, name):
        return str(self.get_parameter(name).value)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    @staticmethod
    def _recent(now, timestamp, timeout):
        return timestamp is not None and (now - timestamp) <= timeout

    def _supervisor_callback(self, msg):
        try:
            payload = json.loads(msg.data)
            self.assessment_state = str(payload['state']).upper()
            yaw = float(payload.get('recommended_yaw_rate_rad_s', 0.0))
            if not math.isfinite(yaw):
                raise ValueError('recommended yaw rate is not finite')
            self.recommended_yaw_rate = yaw
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(
                f'ignored invalid avoidance supervisor status: {exc}'
            )
            return
        self.supervisor_time = self._now_sec()

    def _ground_speed_callback(self, msg):
        value = float(msg.data)
        if not math.isfinite(value) or value < 0.0:
            self.ground_speed_valid = False
            self.ground_speed_valid_time = self._now_sec()
            return
        self.ground_speed = value
        self.ground_speed_time = self._now_sec()

    def _ground_speed_valid_callback(self, msg):
        self.ground_speed_valid = bool(msg.data)
        self.ground_speed_valid_time = self._now_sec()

    def _flight_mode_callback(self, msg):
        self.flight_mode = str(msg.data).upper()
        self.flight_mode_time = self._now_sec()

    def _armed_callback(self, msg):
        self.armed = bool(msg.data)
        self.armed_time = self._now_sec()

    def _manual_handoff_complete_callback(self, msg):
        self.manual_handoff_complete = bool(msg.data)
        self.manual_handoff_complete_time = self._now_sec()

    def _control_loop(self):
        now = self._now_sec()
        common_inputs_fresh = all((
            self._recent(now, self.supervisor_time, self.input_timeout_sec),
            self._recent(now, self.flight_mode_time, self.px4_state_timeout_sec),
            self._recent(now, self.armed_time, self.px4_state_timeout_sec),
        ))
        ground_speed_valid = (
            self.ground_speed_valid
            and self._recent(
                now,
                self.ground_speed_valid_time,
                self.input_timeout_sec,
            )
            and self._recent(
                now,
                self.ground_speed_time,
                self.input_timeout_sec,
            )
        )
        inputs_fresh = common_inputs_fresh and ground_speed_valid
        output = self.model.step(
            now_sec=now,
            flight_mode=self.flight_mode,
            armed=self.armed,
            assessment_state=self.assessment_state,
            ground_speed_mps=self.ground_speed,
            recommended_yaw_rate_rad_s=self.recommended_yaw_rate,
            ground_speed_valid=ground_speed_valid,
            inputs_fresh=inputs_fresh,
            manual_handoff_complete=(
                self.manual_handoff_complete
                and self._recent(
                    now,
                    self.manual_handoff_complete_time,
                    self.px4_state_timeout_sec,
                )
            ),
        )

        command = Twist()
        command.linear.x = output.linear_x
        command.angular.z = output.angular_z
        self.command_pub.publish(command)
        self.active_pub.publish(Bool(data=output.active))
        self.manual_fallback_pub.publish(Bool(
            data=output.manual_fallback_requested
        ))
        status = {
            'active': output.active,
            'phase': output.phase.value,
            'reason': output.reason,
            'flight_mode': self.flight_mode,
            'armed': self.armed,
            'assessment_state': self.assessment_state,
            'ground_speed_mps': round(self.ground_speed, 3),
            'ground_speed_valid': ground_speed_valid,
            'inputs_fresh': inputs_fresh,
            'linear_x': round(output.linear_x, 3),
            'angular_z': round(output.angular_z, 3),
            'manual_fallback_requested': output.manual_fallback_requested,
        }
        self.status_pub.publish(String(data=json.dumps(status, sort_keys=True)))

        if output.phase is not self.last_phase:
            self.last_phase = output.phase
            self.get_logger().warning(
                f'avoidance phase={output.phase.value} | active={output.active} | '
                f'mode={self.flight_mode} | state={self.assessment_state} | '
                f'cmd=({output.linear_x:.2f}, {output.angular_z:.2f}) | '
                f'reason={output.reason}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceExecutorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
