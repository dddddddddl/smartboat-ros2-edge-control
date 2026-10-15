"""Formal RealSense D435i and EWaSR fusion ROS 2 node."""

import time

from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, String

from smartboat_msgs.msg import FusionDetection
from smartboat_perception.ewasr_core import (
    create_ort_session,
    postprocess_mask,
    preprocess_bgr,
)
from smartboat_perception.fusion_model import (
    FusionParameters,
    GO_STRAIGHT,
    TURN_LEFT,
    TURN_RIGHT,
    analyze_fusion,
)
from smartboat_perception.realsense_source import RealSenseSource


class EwasrFusionNode(Node):
    """Publish safety perception independently from dashboard/SRT streaming."""

    def __init__(self):
        super().__init__("ewasr_fusion")
        self.declare_parameter("model_path", "")
        self.declare_parameter("use_fp16", True)
        self.declare_parameter("model_width", 512)
        self.declare_parameter("model_height", 384)
        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 30)
        self.declare_parameter("frame_timeout_ms", 1000)
        self.declare_parameter("reconnect_delay_sec", 2.0)
        self.declare_parameter("obstacle_class", 0)
        self.declare_parameter("minimum_valid_depth_m", 0.15)
        self.declare_parameter("maximum_valid_depth_m", 3.0)
        self.declare_parameter("avoidance_distance_m", 2.8)
        self.declare_parameter("minimum_obstacle_pixels", 200)
        self.declare_parameter("maximum_depth_samples", 3000)
        self.declare_parameter("near_distance_percentile", 5.0)
        self.declare_parameter("maximum_turn_obstacle_fraction", 0.25)
        self.declare_parameter("visualization_stride", 2)

        self.model_width = self._int_param("model_width")
        self.model_height = self._int_param("model_height")
        self.frame_timeout_ms = self._int_param("frame_timeout_ms")
        self.reconnect_delay_sec = self._float_param("reconnect_delay_sec")
        self.obstacle_class = self._int_param("obstacle_class")
        self.visualization_stride = max(
            1,
            self._int_param("visualization_stride"),
        )
        self.parameters = FusionParameters(
            minimum_valid_depth_m=self._float_param(
                "minimum_valid_depth_m"
            ),
            maximum_valid_depth_m=self._float_param(
                "maximum_valid_depth_m"
            ),
            avoidance_distance_m=self._float_param("avoidance_distance_m"),
            minimum_obstacle_pixels=self._int_param(
                "minimum_obstacle_pixels"
            ),
            maximum_depth_samples=self._int_param("maximum_depth_samples"),
            near_distance_percentile=self._float_param(
                "near_distance_percentile"
            ),
            maximum_turn_obstacle_fraction=self._float_param(
                "maximum_turn_obstacle_fraction"
            ),
        )
        self.parameters.validate()

        self.get_logger().warning("Creating EWaSR ONNX Runtime session")
        self.session = create_ort_session(
            str(self.get_parameter("model_path").value),
            bool(self.get_parameter("use_fp16").value),
        )
        self.input_name = self.session.get_inputs()[0].name
        self.source = RealSenseSource(
            width=self._int_param("camera_width"),
            height=self._int_param("camera_height"),
            fps=self._int_param("camera_fps"),
        )
        self.bridge = CvBridge()
        self.last_start_attempt = 0.0
        self.last_error = "not started"
        self.frame_count = 0

        self.detection_pub = self.create_publisher(
            FusionDetection,
            "/smartboat/perception/fusion/detection",
            10,
        )
        # Transitional compatibility topics. Formal consumers use Detection.
        self.legacy_obstacle_pub = self.create_publisher(
            Bool,
            "/fusion_obstacle_detected",
            10,
        )
        self.legacy_distance_pub = self.create_publisher(
            Float32,
            "/fusion_obstacle_distance",
            10,
        )
        self.legacy_decision_pub = self.create_publisher(
            String,
            "/fusion_avoidance_decision",
            10,
        )
        self.color_pub = self.create_publisher(
            Image,
            "/smartboat/perception/ewasr/color",
            1,
        )
        self.depth_pub = self.create_publisher(
            Image,
            "/smartboat/perception/ewasr/depth_m",
            1,
        )
        self.mask_pub = self.create_publisher(
            Image,
            "/smartboat/perception/ewasr/mask",
            1,
        )

    def _int_param(self, name):
        return int(self.get_parameter(name).value)

    def _float_param(self, name):
        return float(self.get_parameter(name).value)

    def _ensure_source(self):
        if self.source.started:
            return True
        now = time.monotonic()
        if now - self.last_start_attempt < self.reconnect_delay_sec:
            return False
        self.last_start_attempt = now
        try:
            self.source.start()
        except RuntimeError as exc:
            self.last_error = str(exc)
            self.get_logger().error(f"RealSense start failed: {exc}")
            self._publish_unavailable("realsense_start_failed")
            return False
        self.last_error = ""
        self.get_logger().warning("RealSense D435i pipeline started")
        return True

    @staticmethod
    def _decision_value(decision):
        if decision == TURN_LEFT:
            return FusionDetection.TURN_LEFT
        if decision == TURN_RIGHT:
            return FusionDetection.TURN_RIGHT
        return FusionDetection.GO_STRAIGHT

    def _build_detection(self, result, inference_ms, sensor_ready=True):
        msg = FusionDetection()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "d435i_color_optical_frame"
        msg.sensor_ready = bool(sensor_ready)
        msg.obstacle_detected = bool(result.obstacle_detected)
        msg.distance_m = float(result.distance_m)
        msg.decision = self._decision_value(result.decision)
        msg.obstacle_pixels = int(result.obstacle_pixels)
        msg.reason = str(result.reason)
        msg.inference_ms = float(inference_ms)
        return msg

    def _publish_detection(self, msg):
        self.detection_pub.publish(msg)
        decision = GO_STRAIGHT
        if msg.decision == FusionDetection.TURN_LEFT:
            decision = TURN_LEFT
        elif msg.decision == FusionDetection.TURN_RIGHT:
            decision = TURN_RIGHT
        self.legacy_obstacle_pub.publish(
            Bool(data=bool(msg.obstacle_detected))
        )
        self.legacy_distance_pub.publish(Float32(data=float(msg.distance_m)))
        self.legacy_decision_pub.publish(String(data=decision))

    def _publish_unavailable(self, reason):
        from smartboat_perception.fusion_model import FusionResult

        result = FusionResult(False, -1.0, GO_STRAIGHT, 0, reason)
        self._publish_detection(
            self._build_detection(result, 0.0, sensor_ready=False)
        )

    def process_once(self):
        if not self._ensure_source():
            time.sleep(0.05)
            return
        try:
            color, depth_m = self.source.read(self.frame_timeout_ms)
        except RuntimeError as exc:
            self.last_error = str(exc)
            self.get_logger().error(f"RealSense read failed: {exc}")
            self.source.stop()
            self._publish_unavailable("realsense_read_failed")
            return

        started = time.monotonic()
        model_input = preprocess_bgr(
            color,
            self.model_width,
            self.model_height,
        )
        outputs = self.session.run(None, {self.input_name: model_input})
        mask = postprocess_mask(outputs)
        result = analyze_fusion(
            depth_m,
            mask,
            self.obstacle_class,
            self.parameters,
        )
        inference_ms = (time.monotonic() - started) * 1000.0
        detection = self._build_detection(result, inference_ms)
        self._publish_detection(detection)

        if self.frame_count % self.visualization_stride == 0:
            stamp = detection.header.stamp
            color_msg = self.bridge.cv2_to_imgmsg(color, encoding="bgr8")
            depth_msg = self.bridge.cv2_to_imgmsg(depth_m, encoding="32FC1")
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
            for msg in (color_msg, depth_msg, mask_msg):
                msg.header.stamp = stamp
                msg.header.frame_id = detection.header.frame_id
            self.color_pub.publish(color_msg)
            self.depth_pub.publish(depth_msg)
            self.mask_pub.publish(mask_msg)
        self.frame_count += 1

    def close(self):
        self.source.stop()


def main(args=None):
    rclpy.init(args=args)
    node = EwasrFusionNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            node.process_once()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
