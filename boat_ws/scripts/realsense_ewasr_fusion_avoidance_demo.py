import argparse
import os
import time

import numpy as np
import cv2
import pyrealsense2 as rs
import onnxruntime as ort

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


MODEL_PATH_CANDIDATES = (
    os.environ.get("SMARTBOAT_EWASR_MODEL"),
    "/workspace/boat_ws/models/ewasr_resnet18.onnx",
    "/home/ubuntu2204/smartboat/boat_ws/models/ewasr_resnet18.onnx",
)

MODEL_W = 512
MODEL_H = 384

CAM_W = 640
CAM_H = 480

# RealSense 深度可靠距离上限：你的场景按 3m 处理
MIN_VALID_DEPTH_M = 0.15
MAX_VALID_DEPTH_M = 3.0

# 小于这个距离才建议避让
AVOID_DISTANCE_THRESHOLD_M = 2.8

# 视觉障碍区域至少多少像素才算“视觉看到了障碍”
MIN_VISION_OBSTACLE_PIXELS = 200

# 最多采样多少个深度点，避免太慢
MAX_DEPTH_SAMPLES = 3000

# 当前 ewasr_resnet18.onnx 已核对：obstacle / water / sky 中 obstacle 为 0
DEFAULT_OBSTACLE_CLASS = 0


def resolve_model_path():
    for path in MODEL_PATH_CANDIDATES:
        if path and os.path.exists(path):
            return path
    return "/workspace/boat_ws/models/ewasr_resnet18.onnx"


def preprocess_bgr(color_image):
    img = cv2.resize(color_image, (MODEL_W, MODEL_H), interpolation=cv2.INTER_LINEAR)
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)
    img = np.ascontiguousarray(img, dtype=np.float32)
    return img


def postprocess_mask(outputs):
    prediction = outputs[0]  # [1, 3, 96, 128]
    mask = np.argmax(prediction[0], axis=0).astype(np.uint8)
    return mask


def create_ort_session(use_fp16=False):
    so = ort.SessionOptions()
    so.intra_op_num_threads = 1
    so.inter_op_num_threads = 1

    providers = [
        (
            "TensorrtExecutionProvider",
            {
                "device_id": 0,
                "trt_fp16_enable": use_fp16,
                "trt_engine_cache_enable": True,
                "trt_engine_cache_path": "/workspace/boat_ws/trt_engine_cache",
            },
        ),
        ("CUDAExecutionProvider", {"device_id": 0}),
        "CPUExecutionProvider",
    ]

    print("ONNX Runtime:", ort.__version__)
    print("Available providers:", ort.get_available_providers())

    sess = ort.InferenceSession(
        resolve_model_path(),
        sess_options=so,
        providers=providers,
    )

    print("Actual session providers:", sess.get_providers())
    return sess


def analyze_fusion(depth_frame, mask, obstacle_class):
    """
    融合逻辑：
    1. EWaSR mask 先看到障碍物
    2. 对应区域 RealSense 深度在 0.15m~3m 内有效
    3. 距离小于 2.8m 才建议避让
    """
    mask_big = cv2.resize(mask, (CAM_W, CAM_H), interpolation=cv2.INTER_NEAREST)

    obstacle_area = mask_big == obstacle_class
    ys, xs = np.where(obstacle_area)
    vision_pixels = int(len(xs))

    if vision_pixels < MIN_VISION_OBSTACLE_PIXELS:
        return False, -1.0, "GO_STRAIGHT", vision_pixels, "vision_no_obstacle"

    if len(xs) > MAX_DEPTH_SAMPLES:
        idx = np.random.choice(len(xs), size=MAX_DEPTH_SAMPLES, replace=False)
        xs_sample = xs[idx]
        ys_sample = ys[idx]
    else:
        xs_sample = xs
        ys_sample = ys

    valid_depths = []
    valid_xs = []

    for x, y in zip(xs_sample, ys_sample):
        d = depth_frame.get_distance(int(x), int(y))
        if MIN_VALID_DEPTH_M < d < MAX_VALID_DEPTH_M:
            valid_depths.append(d)
            valid_xs.append(x)

    if len(valid_depths) == 0:
        # 视觉看到了，但深度没有 3m 内有效障碍物：按你的要求忽略
        return False, -1.0, "GO_STRAIGHT", vision_pixels, "vision_only_depth_invalid_ignore"

    distance_m = float(np.median(valid_depths))
    obstacle_center_x = float(np.median(valid_xs))

    if distance_m >= AVOID_DISTANCE_THRESHOLD_M:
        return False, distance_m, "GO_STRAIGHT", vision_pixels, "obstacle_far_enough"

    left_boundary = CAM_W / 3
    right_boundary = CAM_W * 2 / 3

    if obstacle_center_x < left_boundary:
        direction = "TURN_RIGHT"
        reason = "obstacle_on_left_turn_right"
    elif obstacle_center_x > right_boundary:
        direction = "TURN_LEFT"
        reason = "obstacle_on_right_turn_left"
    else:
        left_count = int(np.sum(obstacle_area[:, : CAM_W // 2]))
        right_count = int(np.sum(obstacle_area[:, CAM_W // 2 :]))

        if left_count <= right_count:
            direction = "TURN_LEFT"
            reason = "obstacle_center_left_side_freer"
        else:
            direction = "TURN_RIGHT"
            reason = "obstacle_center_right_side_freer"

    return True, distance_m, direction, vision_pixels, reason


class FusionAvoidanceNode(Node):
    def __init__(self, obstacle_class, use_fp16):
        super().__init__("realsense_ewasr_fusion_avoidance_demo")

        self.obstacle_class = obstacle_class

        self.pub_obstacle = self.create_publisher(Bool, "/fusion_obstacle_detected", 10)
        self.pub_distance = self.create_publisher(Float32, "/fusion_obstacle_distance", 10)
        self.pub_decision = self.create_publisher(String, "/fusion_avoidance_decision", 10)

        self.get_logger().info("Creating ONNX Runtime TensorRT session...")
        self.sess = create_ort_session(use_fp16=use_fp16)
        self.input_name = self.sess.get_inputs()[0].name

        self.get_logger().info("Starting RealSense D435i pipeline...")
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_stream(rs.stream.depth, CAM_W, CAM_H, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, CAM_W, CAM_H, rs.format.bgr8, 30)

        self.pipeline.start(self.config)

        # 重点：把 depth 对齐到 color，因为 EWaSR 是基于彩色图做分割
        self.align = rs.align(rs.stream.color)

        self.frame_id = 0
        self.timer = self.create_timer(0.2, self.loop)

        self.get_logger().info("Fusion avoidance demo started.")
        self.get_logger().info("No MAVLink. Only ROS2 console + topics.")
        self.get_logger().info("Publishing:")
        self.get_logger().info("  /fusion_obstacle_detected")
        self.get_logger().info("  /fusion_obstacle_distance")
        self.get_logger().info("  /fusion_avoidance_decision")

    def loop(self):
        t0 = time.time()

        frames = self.pipeline.wait_for_frames()
        frames = self.align.process(frames)

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            self.get_logger().warn("No valid RealSense frame")
            return

        color_image = np.asanyarray(color_frame.get_data())

        t1 = time.time()
        x = preprocess_bgr(color_image)

        t2 = time.time()
        outputs = self.sess.run(None, {self.input_name: x})

        t3 = time.time()
        mask = postprocess_mask(outputs)

        confirmed, distance_m, direction, vision_pixels, reason = analyze_fusion(
            depth_frame=depth_frame,
            mask=mask,
            obstacle_class=self.obstacle_class,
        )

        t4 = time.time()

        self.frame_id += 1

        obstacle_msg = Bool()
        obstacle_msg.data = bool(confirmed)

        distance_msg = Float32()
        distance_msg.data = float(distance_m)

        decision_msg = String()
        decision_msg.data = direction

        self.pub_obstacle.publish(obstacle_msg)
        self.pub_distance.publish(distance_msg)
        self.pub_decision.publish(decision_msg)

        if confirmed:
            self.get_logger().warn(
                f"[FRAME {self.frame_id}] CONFIRMED OBSTACLE | "
                f"distance={distance_m:.3f} m | "
                f"decision={direction} | "
                f"vision_pixels={vision_pixels} | "
                f"reason={reason} | "
                f"pre={(t2 - t1) * 1000:.2f} ms | "
                f"infer={(t3 - t2) * 1000:.2f} ms | "
                f"fusion={(t4 - t3) * 1000:.2f} ms"
            )
        else:
            self.get_logger().info(
                f"[FRAME {self.frame_id}] no confirmed obstacle | "
                f"distance={distance_m:.3f} m | "
                f"decision={direction} | "
                f"vision_pixels={vision_pixels} | "
                f"reason={reason}"
            )

    def destroy_node(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--obstacle-class", type=int, default=DEFAULT_OBSTACLE_CLASS)
    parser.add_argument("--fp16", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = FusionAvoidanceNode(
        obstacle_class=args.obstacle_class,
        use_fp16=args.fp16,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
