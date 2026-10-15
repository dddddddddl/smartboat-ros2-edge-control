import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Twist


def clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def shortest_angle_error(target, current):
    target_deg = math.degrees(target)
    current_deg = math.degrees(current)
    error_deg = (target_deg - current_deg + 180.0) % 360.0 - 180.0
    return math.radians(error_deg)


class AvoidancePIDNode(Node):
    def __init__(self):
        super().__init__('avoidance_pid')

        self.obstacle_detected = False
        self.obstacle_distance = -1.0
        self.avoidance_decision = 'GO_STRAIGHT'

        self.current_yaw = None
        self.mission_yaw = None
        self.target_yaw = None

        self.last_detected_time = None
        self.last_distance_time = None
        self.last_decision_time = None
        self.last_yaw_time = None
        self.ground_speed = None
        self.last_ground_speed_time = None

        # -----------------------------
        # Parameters from YAML
        # -----------------------------
        self.declare_parameter('control_rate_hz', 10.0)

        self.declare_parameter('safe_distance', 2.0)
        self.declare_parameter('stop_distance', 0.8)

        self.declare_parameter('normal_speed', 0.5)
        self.declare_parameter('slow_speed', 0.2)
        self.declare_parameter(
            'ground_speed_topic',
            '/smartboat/localization/ground_speed'
        )
        self.declare_parameter('enable_ground_speed_limit', True)
        self.declare_parameter('max_avoidance_ground_speed', 1.0)
        self.declare_parameter('ground_speed_timeout_sec', 1.0)
        self.declare_parameter('ground_speed_kp', 0.8)
        self.declare_parameter('ground_speed_ki', 0.0)
        self.declare_parameter('ground_speed_kd', 0.1)
        self.declare_parameter('ground_speed_integral_limit', 0.5)

        self.declare_parameter('avoidance_yaw_deg', 30.0)

        self.declare_parameter('kp', 1.2)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.25)

        self.declare_parameter('integral_limit', 0.5)
        self.declare_parameter('max_yaw_rate', 0.7)

        self.declare_parameter('speed_smoothing_alpha', 0.12)
        self.declare_parameter('yaw_smoothing_alpha', 0.18)

        self.declare_parameter('obstacle_timeout_sec', 0.8)
        self.declare_parameter('yaw_timeout_sec', 1.0)
        self.declare_parameter('yaw_input_unit', 'degrees')
        self.declare_parameter('clear_delay_sec', 2.0)

        self.control_rate_hz = float(
            self.get_parameter('control_rate_hz').value
        )

        self.safe_distance = float(
            self.get_parameter('safe_distance').value
        )
        self.stop_distance = float(
            self.get_parameter('stop_distance').value
        )

        self.normal_speed = float(
            self.get_parameter('normal_speed').value
        )
        self.slow_speed = float(
            self.get_parameter('slow_speed').value
        )
        self.ground_speed_topic = str(
            self.get_parameter('ground_speed_topic').value
        )
        self.enable_ground_speed_limit = bool(
            self.get_parameter('enable_ground_speed_limit').value
        )
        self.max_avoidance_ground_speed = float(
            self.get_parameter('max_avoidance_ground_speed').value
        )
        self.ground_speed_timeout_sec = float(
            self.get_parameter('ground_speed_timeout_sec').value
        )
        self.ground_speed_kp = float(
            self.get_parameter('ground_speed_kp').value
        )
        self.ground_speed_ki = float(
            self.get_parameter('ground_speed_ki').value
        )
        self.ground_speed_kd = float(
            self.get_parameter('ground_speed_kd').value
        )
        self.ground_speed_integral_limit = float(
            self.get_parameter('ground_speed_integral_limit').value
        )

        self.avoidance_yaw_deg = float(
            self.get_parameter('avoidance_yaw_deg').value
        )
        self.avoidance_yaw_rad = math.radians(self.avoidance_yaw_deg)

        self.kp = float(self.get_parameter('kp').value)
        self.ki = float(self.get_parameter('ki').value)
        self.kd = float(self.get_parameter('kd').value)

        self.integral = 0.0
        self.prev_error = 0.0
        self.pid_initialized = False
        self.ground_speed_integral = 0.0
        self.ground_speed_prev_error = 0.0
        self.ground_speed_pid_initialized = False

        self.integral_limit = float(
            self.get_parameter('integral_limit').value
        )
        self.max_yaw_rate = float(
            self.get_parameter('max_yaw_rate').value
        )

        self.speed_smoothing_alpha = float(
            self.get_parameter('speed_smoothing_alpha').value
        )
        self.yaw_smoothing_alpha = float(
            self.get_parameter('yaw_smoothing_alpha').value
        )

        self.obstacle_timeout_sec = float(
            self.get_parameter('obstacle_timeout_sec').value
        )
        self.yaw_timeout_sec = float(
            self.get_parameter('yaw_timeout_sec').value
        )
        self.yaw_input_unit = str(
            self.get_parameter('yaw_input_unit').value
        ).lower()
        self.clear_delay_sec = float(
            self.get_parameter('clear_delay_sec').value
        )

        # 避障状态锁存，防止 target_yaw 从 30° 累加到 60°、90°
        self.avoidance_active = False
        self.active_decision = 'GO_STRAIGHT'
        self.last_valid_obstacle_time = None

        self.current_speed_cmd = 0.0
        self.current_yaw_rate_cmd = 0.0

        self.create_subscription(
            Bool,
            '/fusion_obstacle_detected',
            self.obstacle_detected_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/fusion_obstacle_distance',
            self.obstacle_distance_callback,
            10
        )

        self.create_subscription(
            String,
            '/fusion_avoidance_decision',
            self.avoidance_decision_callback,
            10
        )

        self.create_subscription(
            Float32,
            '/smartboat/localization/yaw',
            self.yaw_callback,
            10
        )

        self.create_subscription(
            Float32,
            self.ground_speed_topic,
            self.ground_speed_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/smartboat/control/cmd_vel',
            10
        )

        self.status_pub = self.create_publisher(
            String,
            '/smartboat/control/avoidance_status',
            10
        )

        self.create_timer(1.0 / self.control_rate_hz, self.control_loop)

        self.get_logger().info(
            f'PID avoidance node started. '
            f'kp={self.kp}, ki={self.ki}, kd={self.kd}, '
            f'safe_distance={self.safe_distance}, '
            f'avoidance_yaw_deg={self.avoidance_yaw_deg}, '
            f'yaw_input_unit={self.yaw_input_unit}, '
            f'ground_speed_limit={self.enable_ground_speed_limit}, '
            f'max_avoidance_ground_speed={self.max_avoidance_ground_speed}'
        )

    def now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def obstacle_detected_callback(self, msg):
        self.obstacle_detected = bool(msg.data)
        self.last_detected_time = self.now_sec()

    def obstacle_distance_callback(self, msg):
        self.obstacle_distance = float(msg.data)
        self.last_distance_time = self.now_sec()

    def avoidance_decision_callback(self, msg):
        self.avoidance_decision = str(msg.data)
        self.last_decision_time = self.now_sec()

    def yaw_callback(self, msg):
        yaw_value = float(msg.data)
        if self.yaw_input_unit in ('degree', 'degrees', 'deg'):
            self.current_yaw = wrap_angle(math.radians(yaw_value))
        elif self.yaw_input_unit in ('radian', 'radians', 'rad'):
            self.current_yaw = wrap_angle(yaw_value)
        else:
            self.get_logger().warning(
                f'unknown yaw_input_unit={self.yaw_input_unit}; '
                'treating yaw as degrees'
            )
            self.current_yaw = wrap_angle(math.radians(yaw_value))
        self.last_yaw_time = self.now_sec()

        if self.mission_yaw is None:
            self.mission_yaw = self.current_yaw

    def ground_speed_callback(self, msg):
        self.ground_speed = abs(float(msg.data))
        self.last_ground_speed_time = self.now_sec()

    def is_recent(self, timestamp, timeout):
        if timestamp is None:
            return False
        return (self.now_sec() - timestamp) < timeout

    def has_valid_yaw(self):
        return (
            self.current_yaw is not None
            and self.is_recent(self.last_yaw_time, self.yaw_timeout_sec)
        )

    def has_valid_ground_speed(self):
        return (
            self.ground_speed is not None
            and self.is_recent(
                self.last_ground_speed_time,
                self.ground_speed_timeout_sec
            )
        )

    def has_valid_obstacle_raw(self):
        data_recent = (
            self.is_recent(self.last_detected_time, self.obstacle_timeout_sec)
            and self.is_recent(self.last_distance_time, self.obstacle_timeout_sec)
            and self.is_recent(self.last_decision_time, self.obstacle_timeout_sec)
        )

        if not data_recent:
            return False

        if not self.obstacle_detected:
            return False

        if self.obstacle_distance <= 0.0:
            return False

        if self.obstacle_distance >= self.safe_distance:
            return False

        return True

    def has_valid_obstacle(self):
        now = self.now_sec()
        raw_valid = self.has_valid_obstacle_raw()

        if raw_valid:
            self.last_valid_obstacle_time = now
            return True

        if (
            self.avoidance_active
            and self.last_valid_obstacle_time is not None
            and (now - self.last_valid_obstacle_time) < self.clear_delay_sec
        ):
            return True

        return False

    def low_pass(self, current, target, alpha):
        return current + alpha * (target - current)

    def reset_pid(self):
        self.integral = 0.0
        self.prev_error = 0.0
        self.pid_initialized = False

    def reset_ground_speed_pid(self):
        self.ground_speed_integral = 0.0
        self.ground_speed_prev_error = 0.0
        self.ground_speed_pid_initialized = False

    def update_pid(self, target_yaw, current_yaw, dt):
        error = shortest_angle_error(target_yaw, current_yaw)

        if not self.pid_initialized:
            self.prev_error = error
            self.integral = 0.0
            self.pid_initialized = True

        self.integral += error * dt
        self.integral = clamp(
            self.integral,
            -self.integral_limit,
            self.integral_limit
        )

        derivative = (error - self.prev_error) / dt

        yaw_rate = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        yaw_rate = clamp(
            yaw_rate,
            -self.max_yaw_rate,
            self.max_yaw_rate
        )

        self.prev_error = error

        return yaw_rate, error

    def update_target_yaw(self, valid_obstacle):
        if self.current_yaw is None:
            return None

        # 没有障碍物：退出避障模式，任务航向跟随当前航向
        if not valid_obstacle:
            self.avoidance_active = False
            self.mission_yaw = self.current_yaw
            self.target_yaw = self.mission_yaw
            self.active_decision = 'GO_STRAIGHT'
            self.reset_pid()
            return self.target_yaw

        # 第一次进入避障：锁定进入避障瞬间的航向
        if not self.avoidance_active:
            self.avoidance_active = True
            self.mission_yaw = self.current_yaw
            self.target_yaw = None
            self.active_decision = 'GO_STRAIGHT'
            self.reset_pid()

        # 同一个 TURN_LEFT / TURN_RIGHT 期间，不重新累加 30°
        if self.target_yaw is not None and self.avoidance_decision == self.active_decision:
            return self.target_yaw

        if self.avoidance_decision == 'TURN_LEFT':
            self.target_yaw = wrap_angle(
                self.mission_yaw + self.avoidance_yaw_rad
            )
            self.active_decision = 'TURN_LEFT'

        elif self.avoidance_decision == 'TURN_RIGHT':
            self.target_yaw = wrap_angle(
                self.mission_yaw - self.avoidance_yaw_rad
            )
            self.active_decision = 'TURN_RIGHT'

        else:
            self.target_yaw = self.mission_yaw
            self.active_decision = 'GO_STRAIGHT'

        self.reset_pid()
        return self.target_yaw

    def apply_ground_speed_limit(self, base_speed, dt):
        if not self.enable_ground_speed_limit:
            self.reset_ground_speed_pid()
            return base_speed

        if not self.has_valid_ground_speed():
            self.reset_ground_speed_pid()
            return base_speed

        error = self.max_avoidance_ground_speed - self.ground_speed

        if not self.ground_speed_pid_initialized:
            self.ground_speed_prev_error = error
            self.ground_speed_integral = 0.0
            self.ground_speed_pid_initialized = True

        self.ground_speed_integral += error * dt
        self.ground_speed_integral = clamp(
            self.ground_speed_integral,
            -self.ground_speed_integral_limit,
            self.ground_speed_integral_limit
        )
        derivative = (error - self.ground_speed_prev_error) / dt
        self.ground_speed_prev_error = error

        speed_limit_cmd = (
            base_speed
            + self.ground_speed_kp * error
            + self.ground_speed_ki * self.ground_speed_integral
            + self.ground_speed_kd * derivative
        )
        return clamp(speed_limit_cmd, 0.0, base_speed)

    def calculate_target_speed(self, valid_obstacle, dt):
        if not valid_obstacle:
            self.reset_ground_speed_pid()
            return self.normal_speed

        if self.obstacle_distance <= self.stop_distance:
            self.reset_ground_speed_pid()
            return 0.0

        return self.apply_ground_speed_limit(self.slow_speed, dt)

    def control_loop(self):
        dt = 1.0 / self.control_rate_hz

        valid_yaw = self.has_valid_yaw()
        valid_obstacle = self.has_valid_obstacle()

        if not valid_yaw:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.get_logger().warning(
                'valid_yaw=False | waiting for /smartboat/localization/yaw'
            )
            return

        target_speed = self.calculate_target_speed(valid_obstacle, dt)
        target_yaw = self.update_target_yaw(valid_obstacle)

        if target_yaw is None:
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            return

        target_yaw_rate, yaw_error = self.update_pid(
            target_yaw,
            self.current_yaw,
            dt
        )

        self.current_speed_cmd = self.low_pass(
            self.current_speed_cmd,
            target_speed,
            self.speed_smoothing_alpha
        )

        self.current_yaw_rate_cmd = self.low_pass(
            self.current_yaw_rate_cmd,
            target_yaw_rate,
            self.yaw_smoothing_alpha
        )

        cmd = Twist()
        cmd.linear.x = float(self.current_speed_cmd)
        cmd.angular.z = float(self.current_yaw_rate_cmd)

        self.cmd_pub.publish(cmd)

        status = (
            f'valid_yaw={valid_yaw} | '
            f'valid_obstacle={valid_obstacle} | '
            f'distance={self.obstacle_distance:.2f} m | '
            f'decision={self.avoidance_decision} | '
            f'current_yaw={math.degrees(self.current_yaw):.1f} deg | '
            f'target_yaw={math.degrees(target_yaw):.1f} deg | '
            f'yaw_error={math.degrees(yaw_error):.1f} deg | '
            f'ground_speed='
            f'{self.ground_speed if self.ground_speed is not None else -1.0:.2f} | '
            f'target_speed={target_speed:.2f} | '
            f'cmd_speed={cmd.linear.x:.2f} | '
            f'cmd_yaw_rate={cmd.angular.z:.2f}'
        )

        self.status_pub.publish(String(data=status))
        self.get_logger().info(status)


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = AvoidancePIDNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        # launch Ctrl+C 时 rclpy 可能已经在关闭，避免退出时报红
        if rclpy.ok() and node is not None:
            node.get_logger().error(f'avoidance_pid_node error: {exc}')

    finally:
        if node is not None:
            # 只有 ROS context 还有效时才发布停止指令
            try:
                if rclpy.ok():
                    stop_cmd = Twist()
                    node.cmd_pub.publish(stop_cmd)
                    rclpy.spin_once(node, timeout_sec=0.1)
            except Exception:
                pass

            try:
                node.destroy_node()
            except Exception:
                pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
