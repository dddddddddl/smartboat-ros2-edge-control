"""ROS 2 safety assessment for radar/camera obstacle inputs."""

import json
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String

from smartboat_msgs.msg import FusionDetection, RadarTargetArray
from smartboat_navigation.safety_model import SafetyModel, SafetyParameters


class AvoidanceSupervisorNode(Node):
    def __init__(self):
        super().__init__('avoidance_supervisor')

        defaults = SafetyParameters()
        geometry_defaults = {
            'vessel_length_m': 1.20,
            'vessel_width_m': 0.90,
            'vessel_total_height_m': 0.80,
            'vessel_draft_m': 0.20,
            'clearance_each_side_m': 0.20,
            'safety_length_m': 1.60,
            'safety_width_m': 1.30,
            'safety_height_m': 1.20,
            'minimum_bridge_height_above_water_m': 0.80,
            'maximum_bridge_length_m': 5.0,
            'measured_turn_speed_mps': 1.257,
            'measured_yaw_rate_deg_s': 35.72,
            'measured_turn_radius_m': 2.016,
            'conservative_yaw_rate_limit_deg_s': 30.0,
        }
        for name, value in geometry_defaults.items():
            self.declare_parameter(name, value)
        self.declare_parameter('cruise_speed_mps', defaults.cruise_speed_mps)
        self.declare_parameter(
            'avoidance_speed_mps',
            defaults.avoidance_speed_mps,
        )
        self.declare_parameter(
            'braking_reference_speed_mps',
            defaults.braking_reference_speed_mps,
        )
        self.declare_parameter(
            'braking_reference_distance_m',
            defaults.braking_reference_distance_m,
        )
        self.declare_parameter(
            'perception_latency_sec',
            defaults.perception_latency_sec,
        )
        self.declare_parameter(
            'actuation_latency_sec',
            defaults.actuation_latency_sec,
        )
        self.declare_parameter(
            'stopping_reserve_m',
            defaults.stopping_reserve_m,
        )
        self.declare_parameter(
            'radar_slowdown_distance_m',
            defaults.radar_slowdown_distance_m,
        )
        self.declare_parameter(
            'd435_reliable_distance_m',
            defaults.d435_reliable_distance_m,
        )
        self.declare_parameter(
            'radar_emergency_ttc_sec',
            defaults.radar_emergency_ttc_sec,
        )
        self.declare_parameter(
            'radar_slowdown_ttc_sec',
            defaults.radar_slowdown_ttc_sec,
        )
        self.declare_parameter(
            'radar_topic',
            '/smartboat/perception/radar/closest_obstacle_distance',
        )
        self.declare_parameter('radar_timeout_sec', 0.5)
        self.declare_parameter('camera_timeout_sec', 0.8)
        self.declare_parameter('camera_to_bow_m', 0.45)
        self.declare_parameter('ground_speed_timeout_sec', 0.8)
        self.declare_parameter('maximum_assumed_deceleration_mps2', 0.8)
        self.declare_parameter('require_radar', True)
        self.declare_parameter('control_rate_hz', 10.0)

        self.model = SafetyModel(SafetyParameters(
            cruise_speed_mps=self._float_param('cruise_speed_mps'),
            avoidance_speed_mps=self._float_param('avoidance_speed_mps'),
            braking_reference_speed_mps=self._float_param(
                'braking_reference_speed_mps'
            ),
            braking_reference_distance_m=self._float_param(
                'braking_reference_distance_m'
            ),
            perception_latency_sec=self._float_param('perception_latency_sec'),
            actuation_latency_sec=self._float_param('actuation_latency_sec'),
            stopping_reserve_m=self._float_param('stopping_reserve_m'),
            radar_slowdown_distance_m=self._float_param(
                'radar_slowdown_distance_m'
            ),
            d435_reliable_distance_m=self._float_param(
                'd435_reliable_distance_m'
            ),
            radar_emergency_ttc_sec=self._float_param(
                'radar_emergency_ttc_sec'
            ),
            radar_slowdown_ttc_sec=self._float_param(
                'radar_slowdown_ttc_sec'
            ),
        ))

        self.radar_timeout_sec = self._float_param('radar_timeout_sec')
        self.camera_timeout_sec = self._float_param('camera_timeout_sec')
        self.camera_to_bow_m = self._float_param('camera_to_bow_m')
        self.ground_speed_timeout_sec = self._float_param(
            'ground_speed_timeout_sec'
        )
        self.maximum_assumed_deceleration_mps2 = self._float_param(
            'maximum_assumed_deceleration_mps2'
        )
        if (
            not math.isfinite(self.maximum_assumed_deceleration_mps2)
            or self.maximum_assumed_deceleration_mps2 <= 0.0
        ):
            raise ValueError(
                'maximum_assumed_deceleration_mps2 must be positive'
            )
        self.require_radar = bool(self.get_parameter('require_radar').value)
        yaw_rate_deg_s = self._float_param('conservative_yaw_rate_limit_deg_s')
        self.yaw_rate_limit_rad_s = math.radians(yaw_rate_deg_s)

        self.ground_speed = 0.0
        self.radar_distance = None
        self.radar_ttc = None
        self.camera_confirmed = False
        self.camera_sensor_ready = False
        self.camera_distance = None
        self.camera_decision = 'GO_STRAIGHT'
        self.ground_speed_time = None
        self.ground_speed_valid = False
        self.ground_speed_valid_time = None
        self.radar_time = None
        self.camera_detected_time = None
        self.camera_distance_time = None
        self.camera_decision_time = None

        self.create_subscription(
            Float32,
            '/smartboat/localization/ground_speed',
            self._ground_speed_callback,
            10,
        )
        self.create_subscription(
            RadarTargetArray,
            '/smartboat/perception/radar/targets',
            self._radar_targets_callback,
            10,
        )
        self.create_subscription(
            Bool,
            '/smartboat/localization/ground_speed_valid',
            self._ground_speed_valid_callback,
            10,
        )
        radar_topic = str(self.get_parameter('radar_topic').value)
        self.create_subscription(
            Float32,
            radar_topic,
            self._radar_callback,
            10,
        )
        self.create_subscription(
            FusionDetection,
            '/smartboat/perception/fusion/detection',
            self._camera_detection_callback,
            10,
        )

        self.speed_limit_pub = self.create_publisher(
            Float32,
            '/smartboat/navigation/avoidance_speed_limit',
            10,
        )
        self.stop_pub = self.create_publisher(
            Bool,
            '/smartboat/navigation/emergency_stop_requested',
            10,
        )
        self.state_pub = self.create_publisher(
            String,
            '/smartboat/navigation/avoidance_supervisor_status',
            10,
        )
        self.command_pub = self.create_publisher(
            Twist,
            '/smartboat/navigation/avoidance_cmd_vel',
            10,
        )

        rate_hz = self._float_param('control_rate_hz')
        if not math.isfinite(rate_hz) or rate_hz <= 0.0:
            raise ValueError('control_rate_hz must be finite and positive')
        self.create_timer(1.0 / rate_hz, self._control_loop)
        self.get_logger().info(
            'Avoidance supervisor started; recommendations are consumed by '
            'the separately gated production avoidance executor.'
        )

    def _float_param(self, name):
        return float(self.get_parameter(name).value)

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _recent(self, timestamp, timeout):
        if timestamp is None:
            return False
        return (self._now_sec() - timestamp) <= timeout

    def _ground_speed_callback(self, msg):
        value = float(msg.data)
        now = self._now_sec()
        if not math.isfinite(value) or value < 0.0:
            self.ground_speed_valid = False
            self.ground_speed_valid_time = now
            return
        if self.ground_speed_time is not None:
            elapsed = max(0.0, now - self.ground_speed_time)
            conservative_floor = max(
                0.0,
                self.ground_speed
                - self.maximum_assumed_deceleration_mps2 * elapsed,
            )
            value = max(value, conservative_floor)
        self.ground_speed = value
        self.ground_speed_time = now

    def _ground_speed_valid_callback(self, msg):
        self.ground_speed_valid = bool(msg.data)
        self.ground_speed_valid_time = self._now_sec()

    def _radar_callback(self, msg):
        value = float(msg.data)
        if not math.isfinite(value) or (value <= 0.0 and value != -1.0):
            self.radar_distance = None
            self.radar_time = None
            return
        self.radar_distance = value
        self.radar_time = self._now_sec()

    def _radar_targets_callback(self, msg):
        values = [
            float(target.ttc_sec)
            for target in msg.targets
            if target.ttc_valid
            and math.isfinite(float(target.ttc_sec))
            and float(target.ttc_sec) > 0.0
        ]
        self.radar_ttc = min(values) if values else None

    def _camera_detection_callback(self, msg):
        now = self._now_sec()
        self.camera_sensor_ready = bool(msg.sensor_ready)
        self.camera_confirmed = bool(msg.obstacle_detected)
        raw_distance = float(msg.distance_m)
        if (
            self.camera_sensor_ready
            and self.camera_confirmed
            and (not math.isfinite(raw_distance) or raw_distance <= 0.0)
        ):
            self.camera_sensor_ready = False
        if raw_distance > 0.0:
            self.camera_distance = max(
                0.01,
                raw_distance - self.camera_to_bow_m,
            )
        else:
            self.camera_distance = raw_distance
        if msg.decision == FusionDetection.TURN_LEFT:
            self.camera_decision = 'TURN_LEFT'
        elif msg.decision == FusionDetection.TURN_RIGHT:
            self.camera_decision = 'TURN_RIGHT'
        else:
            self.camera_decision = 'GO_STRAIGHT'
        self.camera_detected_time = now
        self.camera_distance_time = now
        self.camera_decision_time = now

    def _camera_fresh(self):
        return self.camera_sensor_ready and all((
            self._recent(self.camera_detected_time, self.camera_timeout_sec),
            self._recent(self.camera_distance_time, self.camera_timeout_sec),
            self._recent(self.camera_decision_time, self.camera_timeout_sec),
        ))

    def _control_loop(self):
        assessment = self.model.assess(
            speed_mps=self.ground_speed,
            ground_speed_fresh=self._recent(
                self.ground_speed_time,
                self.ground_speed_timeout_sec,
            ) and self.ground_speed_valid and self._recent(
                self.ground_speed_valid_time,
                self.ground_speed_timeout_sec,
            ),
            radar_distance_m=self.radar_distance,
            radar_fresh=self._recent(self.radar_time, self.radar_timeout_sec),
            radar_required=self.require_radar,
            camera_confirmed=self.camera_confirmed,
            camera_distance_m=self.camera_distance,
            camera_fresh=self._camera_fresh(),
            radar_ttc_sec=self.radar_ttc,
        )

        self.speed_limit_pub.publish(Float32(data=assessment.speed_limit_mps))
        self.stop_pub.publish(Bool(data=assessment.emergency_stop))
        command = Twist()
        command.linear.x = float(assessment.speed_limit_mps)
        command.angular.z = self.model.yaw_rate_for_decision(
            assessment.state,
            self.camera_decision,
            self.yaw_rate_limit_rad_s,
        )
        self.command_pub.publish(command)
        status = {
            'state': assessment.state.value,
            'reason': assessment.reason,
            'speed_mps': round(self.ground_speed, 3),
            'ground_speed_valid': self.ground_speed_valid,
            'speed_limit_mps': round(assessment.speed_limit_mps, 3),
            'dynamic_stopping_distance_m': round(
                assessment.dynamic_stopping_distance_m,
                3,
            ),
            'closest_obstacle_distance_m': (
                assessment.closest_obstacle_distance_m
            ),
            'radar_ttc_sec': self.radar_ttc,
            'camera_decision': self.camera_decision,
            'recommended_yaw_rate_rad_s': round(command.angular.z, 3),
            'recommendation_only': True,
        }
        self.state_pub.publish(String(data=json.dumps(status, sort_keys=True)))


def main(args=None):
    rclpy.init(args=args)
    node = AvoidanceSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
