import argparse
import json
import math
import os
import queue
import shlex
import subprocess
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort
import pyrealsense2 as rs
from PIL import Image, ImageDraw, ImageFont

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String


WINDOW_NAME = "RealSense + EWaSR Fusion Avoidance"
MODEL_PATH_CANDIDATES = (
    os.environ.get("SMARTBOAT_EWASR_MODEL"),
    "/workspace/boat_ws/models/ewasr_resnet18.onnx",
    "/home/ubuntu2204/smartboat/boat_ws/models/ewasr_resnet18.onnx",
)

MODEL_W = 512
MODEL_H = 384

CAM_W = 640
CAM_H = 480
DASHBOARD_W = 1280
DASHBOARD_H = 960
PANEL_W = DASHBOARD_W // 2
PANEL_H = DASHBOARD_H // 2

MIN_VALID_DEPTH_M = 0.15
MAX_VALID_DEPTH_M = 3.0
AVOID_DISTANCE_THRESHOLD_M = 2.8
MIN_VISION_OBSTACLE_PIXELS = 200
MAX_DEPTH_SAMPLES = 3000
# 当前 ewasr_resnet18.onnx 已核对：obstacle / water / sky 中 obstacle 为 0
DEFAULT_OBSTACLE_CLASS = 0
DEFAULT_STREAM_SERVER = "111.229.242.175"
DEFAULT_STREAM_PORT = 10000
DEFAULT_IDENTITY_DIR = "/host_boot"
REALSENSE_RETRY_SEC = 2.0
RADAR_DISPLAY_TIMEOUT_SEC = 1.0
RADAR_MAX_DISPLAY_DISTANCE_M = 10.0
SEMANTIC_CLASS_COLORS = {
    0: (0, 0, 255),      # obstacle: red (BGR)
    1: (255, 0, 0),      # water: blue (BGR)
    2: (255, 0, 255),    # sky: purple (BGR)
}
SEMANTIC_COLOR_LUT = np.asarray(
    [SEMANTIC_CLASS_COLORS[index] for index in range(3)],
    dtype=np.uint8,
)
CJK_FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
)
CJK_BOLD_FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Bold.otf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf",
) + CJK_FONT_CANDIDATES
_FONT_CACHE = {}
_TEXT_SPRITE_CACHE = {}


def resolve_model_path():
    for path in MODEL_PATH_CANDIDATES:
        if path and os.path.exists(path):
            return path
    return "/workspace/boat_ws/models/ewasr_resnet18.onnx"


def read_text_or_default(path, default):
    try:
        value = Path(path).read_text(encoding="utf-8").strip()
        if value:
            return value
    except OSError:
        pass
    return default


def read_identity(identity_dir, station_default, boat_default):
    base = Path(identity_dir)
    return (
        read_text_or_default(base / "name.txt", station_default),
        read_text_or_default(base / "boat_name.txt", boat_default),
    )


def build_srt_url(args):
    if args.stream_url:
        return args.stream_url

    if args.stream_path:
        stream_path = args.stream_path
    else:
        station, boat = read_identity(
            args.identity_dir,
            args.station_default,
            args.boat_default,
        )
        suffix = args.stream_suffix.strip("/")
        stream_path = f"{station}/{boat}_{suffix}" if suffix else f"{station}/{boat}"

    stream_id = f"publish:{stream_path}"
    return (
        f"srt://{args.stream_server}:{args.stream_port}"
        f"?streamid={stream_id}&pkt_size=1316&mode=caller"
    )


def preprocess_bgr(color_image):
    img = cv2.resize(color_image, (MODEL_W, MODEL_H), interpolation=cv2.INTER_LINEAR)
    img = img.astype(np.float32) / 255.0
    img = np.transpose(img, (2, 0, 1))
    img = np.expand_dims(img, axis=0)
    return np.ascontiguousarray(img, dtype=np.float32)


def postprocess_mask(outputs):
    prediction = outputs[0]
    return np.argmax(prediction[0], axis=0).astype(np.uint8)


def create_ort_session(use_fp16=False):
    session_options = ort.SessionOptions()
    session_options.intra_op_num_threads = 1
    session_options.inter_op_num_threads = 1
    cache_precision = "fp16" if use_fp16 else "fp32"
    cache_path = f"/workspace/boat_ws/trt_engine_cache/ewasr_{cache_precision}"
    try:
        Path(cache_path).mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        cache_path = "/workspace/boat_ws/trt_engine_cache"
        print(
            f"TensorRT precision cache directory is unavailable ({exc}); "
            f"falling back to {cache_path}"
        )

    providers = [
        (
            "TensorrtExecutionProvider",
            {
                "device_id": 0,
                "trt_fp16_enable": use_fp16,
                "trt_engine_cache_enable": True,
                "trt_engine_cache_path": cache_path,
            },
        ),
        ("CUDAExecutionProvider", {"device_id": 0}),
        "CPUExecutionProvider",
    ]

    print("ONNX Runtime:", ort.__version__)
    print("Available providers:", ort.get_available_providers())

    session = ort.InferenceSession(
        resolve_model_path(),
        sess_options=session_options,
        providers=providers,
    )

    print("Actual session providers:", session.get_providers())
    return session


def analyze_fusion(depth_frame, mask, obstacle_class):
    mask_big = cv2.resize(mask, (CAM_W, CAM_H), interpolation=cv2.INTER_NEAREST)
    obstacle_area = mask_big == obstacle_class
    ys, xs = np.where(obstacle_area)
    vision_pixels = int(len(xs))

    if vision_pixels < MIN_VISION_OBSTACLE_PIXELS:
        return False, -1.0, "GO_STRAIGHT", vision_pixels, "vision_no_obstacle"

    if len(xs) > MAX_DEPTH_SAMPLES:
        sample_idx = np.random.choice(len(xs), size=MAX_DEPTH_SAMPLES, replace=False)
        xs_sample = xs[sample_idx]
        ys_sample = ys[sample_idx]
    else:
        xs_sample = xs
        ys_sample = ys

    valid_depths = []
    valid_xs = []

    for x, y in zip(xs_sample, ys_sample):
        distance = depth_frame.get_distance(int(x), int(y))
        if MIN_VALID_DEPTH_M < distance < MAX_VALID_DEPTH_M:
            valid_depths.append(distance)
            valid_xs.append(x)

    if not valid_depths:
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


def make_obstacle_mask(mask, obstacle_class):
    mask_big = cv2.resize(mask, (CAM_W, CAM_H), interpolation=cv2.INTER_NEAREST)
    return mask_big == obstacle_class


def obstacle_bbox(obstacle_mask):
    ys, xs = np.where(obstacle_mask)
    if len(xs) == 0:
        return None
    return int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())


def colorize_depth(depth_frame):
    depth_image = np.asanyarray(depth_frame.get_data())
    depth_m = depth_image.astype(np.float32) * depth_frame.get_units()
    depth_norm = np.clip(depth_m / MAX_VALID_DEPTH_M, 0.0, 1.0)
    depth_u8 = (255 - depth_norm * 255).astype(np.uint8)
    depth_color = cv2.applyColorMap(depth_u8, cv2.COLORMAP_TURBO)
    depth_color[depth_image == 0] = (25, 25, 25)
    return depth_color


def cover_frame_to_size(frame, width, height):
    """Resize and center-crop a frame to completely fill one panel."""
    src_h, src_w = frame.shape[:2]
    scale = max(width / src_w, height / src_h)
    resized_w = max(width, int(round(src_w * scale)))
    resized_h = max(height, int(round(src_h * scale)))
    resized = cv2.resize(
        frame,
        (resized_w, resized_h),
        interpolation=cv2.INTER_AREA,
    )
    x0 = (resized_w - width) // 2
    y0 = (resized_h - height) // 2
    return resized[y0:y0 + height, x0:x0 + width]


def semantic_overlay(color_image, mask, obstacle_class=DEFAULT_OBSTACLE_CLASS):
    mask_big = cv2.resize(
        mask,
        (color_image.shape[1], color_image.shape[0]),
        interpolation=cv2.INTER_NEAREST,
    )
    known = mask_big < len(SEMANTIC_COLOR_LUT)
    safe_indices = np.minimum(mask_big, len(SEMANTIC_COLOR_LUT) - 1)
    color_mask = SEMANTIC_COLOR_LUT[safe_indices]
    blended = cv2.addWeighted(color_image, 0.58, color_mask, 0.42, 0.0)
    if not np.all(known):
        blended[~known] = color_image[~known]
    return blended, mask_big == obstacle_class


def _resolve_font(size, bold=False):
    key = (int(size), bool(bold))
    if key in _FONT_CACHE:
        return _FONT_CACHE[key]
    candidates = CJK_BOLD_FONT_CANDIDATES if bold else CJK_FONT_CANDIDATES
    for path in candidates:
        if os.path.exists(path):
            font = ImageFont.truetype(path, int(size))
            _FONT_CACHE[key] = font
            return font
    font = ImageFont.load_default()
    _FONT_CACHE[key] = font
    return font


def _text_sprite(text, size, bgr, bold=False):
    """Render one Unicode label once, then keep its transparent BGRA sprite."""
    color = tuple(int(channel) for channel in bgr)
    key = (str(text), int(size), color, bool(bold))
    if key in _TEXT_SPRITE_CACHE:
        return _TEXT_SPRITE_CACHE[key]

    font = _resolve_font(size, bold)
    stroke_width = 1
    bbox = font.getbbox(str(text), stroke_width=stroke_width)
    width = max(1, bbox[2] - bbox[0] + 2)
    height = max(1, bbox[3] - bbox[1] + 2)
    rgba = Image.new("RGBA", (width, height), (0, 0, 0, 0))
    drawer = ImageDraw.Draw(rgba)
    drawer.text(
        (1 - bbox[0], 1 - bbox[1]),
        str(text),
        font=font,
        fill=(color[2], color[1], color[0], 255),
        stroke_width=stroke_width,
        stroke_fill=(0, 0, 0, 255),
    )
    sprite = cv2.cvtColor(np.asarray(rgba), cv2.COLOR_RGBA2BGRA)
    _TEXT_SPRITE_CACHE[key] = sprite
    return sprite


def _overlay_text_sprite(image, sprite, xy):
    x, y = int(xy[0]), int(xy[1])
    sprite_h, sprite_w = sprite.shape[:2]
    x0 = max(0, x)
    y0 = max(0, y)
    x1 = min(image.shape[1], x + sprite_w)
    y1 = min(image.shape[0], y + sprite_h)
    if x0 >= x1 or y0 >= y1:
        return

    sprite_roi = sprite[y0 - y:y1 - y, x0 - x:x1 - x]
    alpha = sprite_roi[:, :, 3:4].astype(np.float32) / 255.0
    background = image[y0:y1, x0:x1].astype(np.float32)
    foreground = sprite_roi[:, :, :3].astype(np.float32)
    image[y0:y1, x0:x1] = (
        foreground * alpha + background * (1.0 - alpha)
    ).astype(np.uint8)


def draw_unicode_texts(image, items):
    """Composite cached Unicode sprites without converting the full frame."""
    for item in items:
        text, xy, size, bgr = item[:4]
        bold = bool(item[4]) if len(item) > 4 else False
        _overlay_text_sprite(
            image,
            _text_sprite(text, size, bgr, bold),
            xy,
        )
    return image


def put_ascii_text(
    image,
    text,
    xy,
    scale=0.58,
    color=(215, 225, 235),
    thickness=1,
):
    cv2.putText(
        image,
        str(text),
        (int(xy[0]), int(xy[1])),
        cv2.FONT_HERSHEY_SIMPLEX,
        float(scale),
        color,
        int(thickness),
        cv2.LINE_AA,
    )


def prewarm_dashboard_text():
    """Create every normal-runtime Chinese sprite before frame processing."""
    items = [
        ("智能船避障状态", 28, (245, 245, 245), True),
        ("避障类型：无障碍", 32, (60, 220, 80), True),
        ("避障类型：雷达探测到障碍", 32, (0, 200, 255), True),
        ("避障类型：视觉探测到障碍", 32, (0, 140, 255), True),
        ("避障类型：雷达与视觉均探测到障碍", 32, (0, 80, 255), True),
        ("建议动作：保持直行", 24, (230, 235, 245), False),
        ("建议动作：左转绕行", 24, (230, 235, 245), False),
        ("建议动作：右转绕行", 24, (230, 235, 245), False),
        ("雷达：", 21, (215, 225, 235), False),
        ("等待雷达数据", 21, (215, 225, 235), False),
        ("个目标", 19, (215, 225, 235), False),
        ("最近", 19, (215, 225, 235), False),
        ("深度", 19, (215, 225, 235), False),
        ("米", 19, (215, 225, 235), False),
        ("视觉：已确认障碍", 21, (215, 225, 235), False),
        ("视觉：未确认近距离障碍", 21, (215, 225, 235), False),
        ("判定：视觉未发现障碍", 20, (190, 205, 220), False),
        ("判定：视觉目标深度无效", 20, (190, 205, 220), False),
        ("判定：视觉目标距离较远", 20, (190, 205, 220), False),
        ("判定：障碍在左侧", 20, (190, 205, 220), False),
        ("判定：障碍在右侧", 20, (190, 205, 220), False),
        ("判定：中央障碍，左侧较空", 20, (190, 205, 220), False),
        ("判定：中央障碍，右侧较空", 20, (190, 205, 220), False),
        ("视觉像素", 19, (180, 195, 210), False),
        ("推理", 19, (180, 195, 210), False),
        ("画面", 19, (180, 195, 210), False),
        ("视觉阈值", 19, (180, 195, 210), False),
        ("输出", 19, (180, 195, 210), False),
        ("语义分割彩色遮罩", 24, (245, 245, 245), True),
        ("障碍", 17, (245, 245, 245), False),
        ("水面", 17, (245, 245, 245), False),
        ("天空", 17, (245, 245, 245), False),
        ("毫米波雷达目标（3–10米）", 24, (245, 245, 245), True),
        ("深度相机（0–3米）", 24, (245, 245, 245), True),
        ("彩色/深度相机暂不可用", 34, (0, 120, 255), True),
        ("相机恢复前不发布新的视觉避障判断", 21, (185, 200, 215), False),
        ("等待彩色相机", 26, (150, 165, 180), False),
        ("等待深度相机", 26, (150, 165, 180), False),
    ]
    for text, size, color, bold in items:
        _text_sprite(text, size, color, bold)


def decision_chinese(direction):
    return {
        "TURN_LEFT": "左转绕行",
        "TURN_RIGHT": "右转绕行",
        "GO_STRAIGHT": "保持直行",
    }.get(direction, direction)


def reason_chinese(reason):
    return {
        "vision_no_obstacle": "视觉未发现障碍",
        "vision_only_depth_invalid_ignore": "视觉目标深度无效",
        "obstacle_far_enough": "视觉目标距离较远",
        "obstacle_on_left_turn_right": "障碍在左侧",
        "obstacle_on_right_turn_left": "障碍在右侧",
        "obstacle_center_left_side_freer": "中央障碍，左侧较空",
        "obstacle_center_right_side_freer": "中央障碍，右侧较空",
    }.get(reason, reason)


def avoidance_type_chinese(radar_detected, vision_confirmed):
    if radar_detected and vision_confirmed:
        return "雷达与视觉均探测到障碍", (0, 80, 255)
    if vision_confirmed:
        return "视觉探测到障碍", (0, 140, 255)
    if radar_detected:
        return "雷达探测到障碍", (0, 200, 255)
    return "无障碍", (60, 220, 80)


def _radar_distance(obj):
    try:
        return float(obj.get("distance", -1.0))
    except (TypeError, ValueError):
        return -1.0


def render_radar_panel(radar_objects, radar_fresh):
    panel = np.zeros((PANEL_H, PANEL_W, 3), dtype=np.uint8)
    panel[:] = (22, 26, 34)
    header_h = 44
    cv2.rectangle(panel, (0, 0), (PANEL_W, header_h), (12, 15, 21), -1)

    origin = (PANEL_W // 2, PANEL_H - 14)
    radius_px = 270
    scale = radius_px / RADAR_MAX_DISPLAY_DISTANCE_M
    grid_color = (74, 84, 98)
    for distance in (3.0, 5.0, 10.0):
        arc = []
        for bearing in np.linspace(-56.0, 56.0, 90):
            angle = math.radians(bearing)
            arc.append((
                int(origin[0] + distance * scale * math.sin(angle)),
                int(origin[1] - distance * scale * math.cos(angle)),
            ))
        cv2.polylines(
            panel,
            [np.asarray(arc, dtype=np.int32)],
            False,
            grid_color,
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            panel,
            f"{distance:.0f}m",
            (origin[0] + 8, int(origin[1] - distance * scale)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (165, 175, 190),
            1,
            cv2.LINE_AA,
        )

    for bearing in (-56.0, 0.0, 56.0):
        angle = math.radians(bearing)
        end = (
            int(origin[0] + radius_px * math.sin(angle)),
            int(origin[1] - radius_px * math.cos(angle)),
        )
        cv2.line(panel, origin, end, grid_color, 1, cv2.LINE_AA)

    visible_objects = sorted(radar_objects, key=_radar_distance)
    if radar_fresh:
        for obj in visible_objects[:12]:
            distance = _radar_distance(obj)
            bearing = float(obj.get("bearing_deg", 0.0))
            if not (0.0 < distance <= RADAR_MAX_DISPLAY_DISTANCE_M):
                continue
            angle = math.radians(bearing)
            point = (
                int(origin[0] + distance * scale * math.sin(angle)),
                int(origin[1] - distance * scale * math.cos(angle)),
            )
            color = (0, 70, 255) if distance <= 6.0 else (0, 210, 255)
            cv2.circle(panel, point, 7, color, -1, cv2.LINE_AA)
            cv2.circle(panel, point, 10, (235, 235, 235), 1, cv2.LINE_AA)
            cv2.putText(
                panel,
                f"{obj.get('id', '?')}:{distance:.1f}m",
                (point[0] + 10, point[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.42,
                (240, 240, 240),
                1,
                cv2.LINE_AA,
            )
    return panel


def _panel_header(panel, bgr=(12, 15, 21)):
    cv2.rectangle(panel, (0, 0), (PANEL_W, 44), bgr, -1)


def _status_panel(
    radar_detected,
    radar_fresh,
    radar_count,
    radar_closest_m,
    vision_confirmed,
    distance_m,
    direction,
    vision_pixels,
    reason,
    fps,
    infer_ms,
    dashboard_fps,
):
    panel = np.zeros((PANEL_H, PANEL_W, 3), dtype=np.uint8)
    panel[:] = (28, 32, 40)
    obstacle_type, accent = avoidance_type_chinese(
        radar_detected,
        vision_confirmed,
    )
    cv2.rectangle(panel, (0, 0), (10, PANEL_H), accent, -1)
    cv2.rectangle(panel, (10, 0), (PANEL_W, 58), (16, 19, 26), -1)
    for y in (132, 198, 264):
        cv2.line(panel, (28, y), (PANEL_W - 28, y), (62, 68, 78), 1)

    texts = [
        ("智能船避障状态", (28, 12), 28, (245, 245, 245), True),
        (f"避障类型：{obstacle_type}", (30, 72), 32, accent, True),
        (f"建议动作：{decision_chinese(direction)}", (30, 142), 24, (230, 235, 245)),
        ("雷达：", (30, 174), 21, (215, 225, 235)),
        (
            "视觉：已确认障碍" if vision_confirmed else "视觉：未确认近距离障碍",
            (30, 208),
            21,
            (215, 225, 235),
        ),
        (f"判定：{reason_chinese(reason)}", (30, 240), 20, (190, 205, 220)),
        ("视觉像素", (30, 278), 19, (180, 195, 210)),
        ("推理", (230, 278), 19, (180, 195, 210)),
        ("画面", (30, 310), 19, (180, 195, 210)),
        ("视觉阈值", (220, 310), 19, (180, 195, 210)),
        ("输出", (425, 310), 19, (180, 195, 210)),
    ]
    if not radar_fresh:
        texts.append(("等待雷达数据", (96, 174), 21, (215, 225, 235)))
    draw_unicode_texts(panel, texts)

    if radar_fresh:
        put_ascii_text(panel, str(radar_count), (98, 195), scale=0.55)
        radar_items = [("个目标", (128, 174), 19, (215, 225, 235))]
        if radar_closest_m > 0.0:
            radar_items.extend([
                ("最近", (220, 174), 19, (215, 225, 235)),
                ("米", (360, 174), 19, (215, 225, 235)),
            ])
            put_ascii_text(panel, f"{radar_closest_m:.2f}", (278, 195), scale=0.55)
        draw_unicode_texts(panel, radar_items)
    if distance_m >= 0.0:
        draw_unicode_texts(
            panel,
            [
                ("深度", (335, 208), 19, (215, 225, 235)),
                ("米", (468, 208), 19, (215, 225, 235)),
            ],
        )
        put_ascii_text(panel, f"{distance_m:.2f}", (390, 228), scale=0.55)
    put_ascii_text(panel, str(vision_pixels), (120, 299), scale=0.56)
    put_ascii_text(
        panel,
        f"{fps:.1f} FPS / {infer_ms:.1f} ms",
        (282, 299),
        scale=0.53,
        color=(180, 195, 210),
    )
    put_ascii_text(panel, f"{dashboard_fps:.1f} FPS", (82, 331), scale=0.53)
    put_ascii_text(
        panel,
        f"{AVOID_DISTANCE_THRESHOLD_M:.1f} m",
        (315, 331),
        scale=0.53,
    )
    put_ascii_text(
        panel,
        f"{DASHBOARD_W}x{DASHBOARD_H}",
        (480, 331),
        scale=0.53,
    )
    return panel


def render_dashboard(
    color_image,
    depth_frame,
    mask,
    obstacle_class,
    confirmed,
    distance_m,
    direction,
    vision_pixels,
    reason,
    fps,
    infer_ms,
    dashboard_fps=0.0,
    radar_objects=None,
    radar_fresh=False,
    radar_closest_m=-1.0,
):
    radar_objects = list(radar_objects or [])
    radar_detected = radar_fresh and bool(radar_objects)
    status_panel = _status_panel(
        radar_detected=radar_detected,
        radar_fresh=radar_fresh,
        radar_count=len(radar_objects),
        radar_closest_m=radar_closest_m,
        vision_confirmed=confirmed,
        distance_m=distance_m,
        direction=direction,
        vision_pixels=vision_pixels,
        reason=reason,
        fps=fps,
        infer_ms=infer_ms,
        dashboard_fps=dashboard_fps,
    )

    blended, obstacle_mask = semantic_overlay(
        color_image,
        mask,
        obstacle_class,
    )
    bbox = obstacle_bbox(obstacle_mask)
    if bbox is not None:
        x1, y1, x2, y2 = bbox
        box_color = (0, 255, 255) if confirmed else (160, 160, 160)
        cv2.rectangle(blended, (x1, y1), (x2, y2), box_color, 2)
        cv2.circle(blended, ((x1 + x2) // 2, (y1 + y2) // 2), 5, box_color, -1)

    semantic_panel = cover_frame_to_size(blended, PANEL_W, PANEL_H)
    _panel_header(semantic_panel)
    depth_panel = cover_frame_to_size(
        colorize_depth(depth_frame),
        PANEL_W,
        PANEL_H,
    )
    _panel_header(depth_panel)
    radar_panel = render_radar_panel(radar_objects, radar_fresh)

    dashboard = np.zeros((DASHBOARD_H, DASHBOARD_W, 3), dtype=np.uint8)
    dashboard[0:PANEL_H, 0:PANEL_W] = status_panel
    dashboard[0:PANEL_H, PANEL_W:DASHBOARD_W] = semantic_panel
    dashboard[PANEL_H:DASHBOARD_H, 0:PANEL_W] = radar_panel
    dashboard[PANEL_H:DASHBOARD_H, PANEL_W:DASHBOARD_W] = depth_panel
    cv2.line(dashboard, (PANEL_W, 0), (PANEL_W, DASHBOARD_H), (115, 120, 130), 2)
    cv2.line(dashboard, (0, PANEL_H), (DASHBOARD_W, PANEL_H), (115, 120, 130), 2)

    text_items = [
        ("语义分割彩色遮罩", (PANEL_W + 16, 8), 24, (245, 245, 245), True),
        ("障碍", (PANEL_W + 355, 12), 17, (245, 245, 245)),
        ("水面", (PANEL_W + 445, 12), 17, (245, 245, 245)),
        ("天空", (PANEL_W + 535, 12), 17, (245, 245, 245)),
        ("毫米波雷达目标（3–10米）", (16, PANEL_H + 8), 24, (245, 245, 245), True),
        ("深度相机（0–3米）", (PANEL_W + 16, PANEL_H + 8), 24, (245, 245, 245), True),
    ]
    for x, color in (
        (PANEL_W + 330, SEMANTIC_CLASS_COLORS[0]),
        (PANEL_W + 420, SEMANTIC_CLASS_COLORS[1]),
        (PANEL_W + 510, SEMANTIC_CLASS_COLORS[2]),
    ):
        cv2.rectangle(dashboard, (x, 13), (x + 18, 31), color, -1)
    return draw_unicode_texts(dashboard, text_items)


class DashboardStreamer:
    def __init__(
        self,
        target_url,
        ffmpeg_path,
        fps,
        width,
        height,
        codec,
        bitrate,
        loglevel,
        logger,
        restart_delay_sec=1.0,
    ):
        self.target_url = target_url
        self.ffmpeg_path = ffmpeg_path
        self.fps = fps
        self.width = width
        self.height = height
        self.codec = codec
        self.bitrate = bitrate
        self.loglevel = loglevel
        self.logger = logger
        self.restart_delay_sec = restart_delay_sec
        self.process = None
        self.last_start_attempt = 0.0
        self.last_frame_sent = 0.0
        self.min_frame_interval = 1.0 / max(1, float(fps))

    def build_command(self):
        cmd = [
            self.ffmpeg_path,
            "-hide_banner",
            "-loglevel",
            self.loglevel,
            "-f",
            "rawvideo",
            "-pix_fmt",
            "bgr24",
            "-s",
            f"{self.width}x{self.height}",
            "-r",
            str(self.fps),
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            self.codec,
        ]

        if self.codec == "libx264":
            cmd.extend([
                "-preset",
                "ultrafast",
                "-tune",
                "zerolatency",
                "-profile:v",
                "baseline",
                "-level",
                "3.2",
                "-bf",
                "0",
                "-x264-params",
                "repeat-headers=1:scenecut=0",
            ])

        cmd.extend([
            "-g",
            str(max(1, int(self.fps))),
            "-b:v",
            self.bitrate,
            "-maxrate",
            self.bitrate,
            "-bufsize",
            self.bitrate,
            "-pix_fmt",
            "yuv420p",
            "-muxdelay",
            "0",
            "-muxpreload",
            "0",
            "-flush_packets",
            "1",
            "-mpegts_flags",
            "resend_headers+pat_pmt_at_frames",
            "-f",
            "mpegts",
            self.target_url,
        ])
        return cmd

    def start(self):
        now = time.time()
        if now - self.last_start_attempt < self.restart_delay_sec:
            return False
        self.last_start_attempt = now

        cmd = self.build_command()
        self.logger.warning(
            "Starting dashboard ffmpeg stream: "
            + " ".join(shlex.quote(part) for part in cmd)
        )
        try:
            self.process = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        except OSError as exc:
            self.logger.error(f"Failed to start ffmpeg stream: {exc}")
            self.process = None
            return False
        return True

    def is_running(self):
        return self.process is not None and self.process.poll() is None

    def write_frame(self, frame):
        now = time.time()
        if now - self.last_frame_sent < self.min_frame_interval:
            return
        self.last_frame_sent = now

        if frame.shape[1] != self.width or frame.shape[0] != self.height:
            frame = fit_frame_to_size(frame, self.width, self.height)
        frame = np.ascontiguousarray(frame)

        if not self.is_running() and not self.start():
            return

        try:
            self.process.stdin.write(frame.tobytes())
        except (BrokenPipeError, OSError) as exc:
            self.logger.warning(f"ffmpeg stream pipe failed: {exc}; restarting later")
            self.stop()

    def stop(self):
        if self.process is None:
            return

        if self.process.stdin:
            try:
                self.process.stdin.close()
            except OSError:
                pass

        try:
            self.process.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            try:
                self.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
        finally:
            self.process = None


def fit_frame_to_size(frame, width, height):
    src_h, src_w = frame.shape[:2]
    scale = min(width / src_w, height / src_h)
    resized_w = max(2, int(round(src_w * scale)))
    resized_h = max(2, int(round(src_h * scale)))

    if resized_w % 2:
        resized_w -= 1
    if resized_h % 2:
        resized_h -= 1

    resized = cv2.resize(frame, (resized_w, resized_h), interpolation=cv2.INTER_AREA)
    output = np.zeros((height, width, 3), dtype=np.uint8)
    x0 = (width - resized_w) // 2
    y0 = (height - resized_h) // 2
    output[y0:y0 + resized_h, x0:x0 + resized_w] = resized
    return output


def render_status_dashboard(
    status,
    detail,
    fps=0.0,
    radar_objects=None,
    radar_fresh=False,
    radar_closest_m=-1.0,
):
    radar_objects = list(radar_objects or [])
    dashboard = np.zeros((DASHBOARD_H, DASHBOARD_W, 3), dtype=np.uint8)
    dashboard[:] = (24, 28, 34)
    radar_panel = render_radar_panel(radar_objects, radar_fresh)
    dashboard[PANEL_H:DASHBOARD_H, 0:PANEL_W] = radar_panel
    for x0, y0 in ((PANEL_W, 0), (PANEL_W, PANEL_H)):
        cv2.rectangle(
            dashboard,
            (x0, y0),
            (x0 + PANEL_W, y0 + 44),
            (12, 15, 21),
            -1,
        )
    cv2.rectangle(dashboard, (0, 0), (PANEL_W, PANEL_H), (30, 34, 42), -1)
    cv2.rectangle(dashboard, (0, 0), (10, PANEL_H), (0, 80, 255), -1)
    cv2.line(dashboard, (PANEL_W, 0), (PANEL_W, DASHBOARD_H), (115, 120, 130), 2)
    cv2.line(dashboard, (0, PANEL_H), (DASHBOARD_W, PANEL_H), (115, 120, 130), 2)

    texts = [
        ("智能船避障状态", (28, 14), 28, (245, 245, 245), True),
        (status, (30, 86), 34, (0, 120, 255), True),
        ("雷达：", (30, 194), 22, (210, 220, 235)),
        ("画面", (30, 238), 21, (185, 200, 215)),
        ("相机恢复前不发布新的视觉避障判断", (30, 282), 21, (185, 200, 215)),
        ("语义分割彩色遮罩", (PANEL_W + 16, 8), 24, (245, 245, 245), True),
        ("等待彩色相机", (PANEL_W + 210, 165), 26, (150, 165, 180)),
        ("毫米波雷达目标（3–10米）", (16, PANEL_H + 8), 24, (245, 245, 245), True),
        ("深度相机（0–3米）", (PANEL_W + 16, PANEL_H + 8), 24, (245, 245, 245), True),
        ("等待深度相机", (PANEL_W + 220, PANEL_H + 165), 26, (150, 165, 180)),
    ]
    if not radar_fresh:
        texts.append(("等待雷达数据", (100, 194), 21, (210, 220, 235)))
    if detail and not detail.isascii():
        texts.append((detail, (30, 144), 21, (220, 225, 235)))
    draw_unicode_texts(dashboard, texts)
    if detail and detail.isascii():
        put_ascii_text(dashboard, detail[:70], (30, 168), scale=0.52)
    if radar_fresh:
        radar_value = f"{len(radar_objects)} targets"
        if radar_closest_m > 0.0:
            radar_value += f" / nearest {radar_closest_m:.2f} m"
        put_ascii_text(dashboard, radar_value, (100, 217), scale=0.55)
    put_ascii_text(dashboard, f"{fps:.1f} FPS", (88, 261), scale=0.55)
    return dashboard


class DepthFrameSnapshot:
    """Small depth-frame adapter safe to pass to the dashboard thread."""

    def __init__(self, depth_frame):
        self._data = np.asanyarray(depth_frame.get_data()).copy()
        self._units = float(depth_frame.get_units())

    def get_data(self):
        return self._data

    def get_units(self):
        return self._units


class DashboardRenderWorker:
    """Render only the latest requested dashboard without blocking inference."""

    def __init__(self, streamer, save_frame, logger):
        self.streamer = streamer
        self.save_frame = save_frame
        self.logger = logger
        self.tasks = queue.Queue(maxsize=1)
        self.latest_lock = threading.Lock()
        self.latest_dashboard = None
        self.dashboard_fps = 0.0
        self.last_render_time = None
        self.thread = threading.Thread(
            target=self._run,
            name="smartboat-dashboard-render",
            daemon=True,
        )
        self.thread.start()

    def _submit_latest(self, task):
        try:
            self.tasks.put_nowait(task)
            return
        except queue.Full:
            pass
        try:
            self.tasks.get_nowait()
            self.tasks.task_done()
        except queue.Empty:
            pass
        try:
            self.tasks.put_nowait(task)
        except queue.Full:
            pass

    def submit_dashboard(self, **kwargs):
        self._submit_latest(("dashboard", kwargs))

    def submit_status(self, **kwargs):
        self._submit_latest(("status", kwargs))

    def get_latest(self):
        with self.latest_lock:
            return self.latest_dashboard

    def _run(self):
        while True:
            task = self.tasks.get()
            try:
                if task is None:
                    return
                kind, kwargs = task
                if kind == "dashboard":
                    kwargs["dashboard_fps"] = self.dashboard_fps
                    dashboard = render_dashboard(**kwargs)
                else:
                    dashboard = render_status_dashboard(**kwargs)

                now = time.monotonic()
                if self.last_render_time is not None:
                    dt = now - self.last_render_time
                    instant_fps = 1.0 / dt if dt > 0.0 else 0.0
                    if self.dashboard_fps <= 0.0:
                        self.dashboard_fps = instant_fps
                    else:
                        self.dashboard_fps = (
                            0.85 * self.dashboard_fps + 0.15 * instant_fps
                        )
                self.last_render_time = now

                if self.save_frame:
                    cv2.imwrite(self.save_frame, dashboard)
                if self.streamer is not None:
                    self.streamer.write_frame(dashboard)
                with self.latest_lock:
                    self.latest_dashboard = dashboard
            except Exception as exc:
                self.logger.error(f"Dashboard render worker failed: {exc}")
            finally:
                self.tasks.task_done()

    def stop(self):
        self.tasks.join()
        self.tasks.put(None)
        self.thread.join(timeout=3.0)


class FusionAvoidanceVizNode(Node):
    def __init__(self, obstacle_class, use_fp16):
        super().__init__("realsense_ewasr_fusion_avoidance_viz_demo")
        self.obstacle_class = obstacle_class
        self.pub_obstacle = self.create_publisher(Bool, "/fusion_obstacle_detected", 10)
        self.pub_distance = self.create_publisher(Float32, "/fusion_obstacle_distance", 10)
        self.pub_decision = self.create_publisher(String, "/fusion_avoidance_decision", 10)
        self.radar_objects = []
        self.radar_status = {}
        self.radar_closest_m = -1.0
        self.last_radar_objects_time = None
        self.last_radar_status_time = None
        self.last_radar_closest_time = None
        self.create_subscription(
            String,
            "/smartboat/perception/radar/objects",
            self.radar_objects_callback,
            10,
        )
        self.create_subscription(
            String,
            "/smartboat/perception/radar/status",
            self.radar_status_callback,
            10,
        )
        self.create_subscription(
            Float32,
            "/smartboat/perception/radar/closest_obstacle_distance",
            self.radar_closest_callback,
            10,
        )

        self.get_logger().info("Creating ONNX Runtime TensorRT session...")
        self.sess = create_ort_session(use_fp16=use_fp16)
        self.input_name = self.sess.get_inputs()[0].name

        self.get_logger().info("Preparing RealSense D435i depth/color pipeline...")
        self.pipeline = None
        self.align = None
        self.pipeline_started = False
        self.last_realsense_attempt = 0.0
        self.last_realsense_error = "not started"
        self.config = rs.config()
        self.config.enable_stream(rs.stream.depth, CAM_W, CAM_H, rs.format.z16, 30)
        self.config.enable_stream(rs.stream.color, CAM_W, CAM_H, rs.format.bgr8, 30)

    def ensure_pipeline(self):
        if self.pipeline_started:
            return True

        now = time.time()
        if now - self.last_realsense_attempt < REALSENSE_RETRY_SEC:
            return False

        self.last_realsense_attempt = now
        self.get_logger().warning("Starting RealSense D435i depth/color pipeline...")
        try:
            self.pipeline = rs.pipeline()
            self.pipeline.start(self.config)
            self.align = rs.align(rs.stream.color)
            self.pipeline_started = True
            self.last_realsense_error = ""
            self.get_logger().warning("RealSense D435i pipeline started.")
            return True
        except RuntimeError as exc:
            self.last_realsense_error = str(exc)
            self.pipeline = None
            self.align = None
            self.pipeline_started = False
            self.get_logger().error(f"RealSense pipeline start failed: {exc}")
            return False

    def reset_pipeline(self, reason):
        self.last_realsense_error = reason
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except Exception:
                pass
        self.pipeline = None
        self.align = None
        self.pipeline_started = False

    def read_frame(self):
        if not self.ensure_pipeline():
            return None, None

        try:
            frames = self.align.process(self.pipeline.wait_for_frames(timeout_ms=1000))
        except RuntimeError as exc:
            self.get_logger().error(f"RealSense frame read failed: {exc}")
            self.reset_pipeline(str(exc))
            return None, None

        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            return None, None
        return depth_frame, np.asanyarray(color_frame.get_data())

    def publish_decision(self, confirmed, distance_m, direction):
        obstacle_msg = Bool()
        obstacle_msg.data = bool(confirmed)
        distance_msg = Float32()
        distance_msg.data = float(distance_m)
        decision_msg = String()
        decision_msg.data = direction
        self.pub_obstacle.publish(obstacle_msg)
        self.pub_distance.publish(distance_msg)
        self.pub_decision.publish(decision_msg)

    def radar_objects_callback(self, msg):
        try:
            parsed = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if isinstance(parsed, list):
            self.radar_objects = [obj for obj in parsed if isinstance(obj, dict)]
            self.last_radar_objects_time = time.monotonic()

    def radar_status_callback(self, msg):
        try:
            parsed = json.loads(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError):
            return
        if isinstance(parsed, dict):
            self.radar_status = parsed
            self.last_radar_status_time = time.monotonic()

    def radar_closest_callback(self, msg):
        self.radar_closest_m = float(msg.data)
        self.last_radar_closest_time = time.monotonic()

    @staticmethod
    def _recent(timestamp, timeout=RADAR_DISPLAY_TIMEOUT_SEC):
        return (
            timestamp is not None
            and (time.monotonic() - timestamp) <= timeout
        )

    def radar_snapshot(self):
        fresh = all((
            bool(self.radar_status.get("data_fresh", False)),
            self._recent(self.last_radar_status_time),
            self._recent(self.last_radar_objects_time),
            self._recent(self.last_radar_closest_time),
        ))
        objects = list(self.radar_objects) if fresh else []
        closest = self.radar_closest_m if fresh else -1.0
        return objects, fresh, closest

    def close(self):
        self.reset_pipeline("closed")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--obstacle-class", type=int, default=DEFAULT_OBSTACLE_CLASS)
    parser.add_argument("--fp16", action="store_true")
    parser.add_argument("--window-name", default=WINDOW_NAME)
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--save-frame", default="")
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--stream", action="store_true")
    parser.add_argument("--stream-url", default="")
    parser.add_argument("--stream-server", default=DEFAULT_STREAM_SERVER)
    parser.add_argument("--stream-port", type=int, default=DEFAULT_STREAM_PORT)
    parser.add_argument("--stream-path", default="")
    parser.add_argument("--stream-suffix", default="")
    parser.add_argument("--stream-fps", type=int, default=15)
    parser.add_argument("--stream-width", type=int, default=1280)
    parser.add_argument("--stream-height", type=int, default=DASHBOARD_H)
    parser.add_argument("--dashboard-stride", type=int, default=2)
    parser.add_argument("--stream-bitrate", default="1800k")
    parser.add_argument("--stream-codec", default="libx264")
    parser.add_argument("--stream-loglevel", default="warning")
    parser.add_argument("--ffmpeg-path", default="ffmpeg")
    parser.add_argument("--identity-dir", default=DEFAULT_IDENTITY_DIR)
    parser.add_argument("--station-default", default="test")
    parser.add_argument("--boat-default", default="test01")
    args = parser.parse_args()

    display_available = bool(os.environ.get("DISPLAY"))
    display_enabled = display_available and not args.no_display

    if not display_available and not args.no_display:
        print("DISPLAY is not set. Run this demo from a desktop/X11-enabled container shell.")
        print("Example: export DISPLAY=:0")
        print("Continuing without local display. Use --stream to push the dashboard remotely.")

    prewarm_dashboard_text()
    rclpy.init()
    node = FusionAvoidanceVizNode(args.obstacle_class, args.fp16)
    streamer = None
    dashboard_worker = None
    frame_id = 0
    last_inference_t = None
    fps = 0.0
    dashboard_stride = max(1, int(args.dashboard_stride))

    try:
        if display_enabled:
            cv2.namedWindow(args.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(args.window_name, DASHBOARD_W, DASHBOARD_H)

        if args.stream:
            stream_width = args.stream_width if args.stream_width > 0 else CAM_W * 2
            stream_height = args.stream_height if args.stream_height > 0 else int(
                round(stream_width * CAM_H / (CAM_W * 2))
            )
            if stream_height % 2:
                stream_height += 1
            streamer = DashboardStreamer(
                target_url=build_srt_url(args),
                ffmpeg_path=args.ffmpeg_path,
                fps=args.stream_fps,
                width=stream_width,
                height=stream_height,
                codec=args.stream_codec,
                bitrate=args.stream_bitrate,
                loglevel=args.stream_loglevel,
                logger=node.get_logger(),
            )
        dashboard_worker = DashboardRenderWorker(
            streamer=streamer,
            save_frame=args.save_frame,
            logger=node.get_logger(),
        )

        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)
            radar_objects, radar_fresh, radar_closest_m = node.radar_snapshot()
            depth_frame, color_image = node.read_frame()
            if depth_frame is None:
                node.get_logger().warn("No valid RealSense frame")
                dashboard_worker.submit_status(
                    status="彩色/深度相机暂不可用",
                    detail=node.last_realsense_error or "等待RealSense画面",
                    fps=fps,
                    radar_objects=radar_objects,
                    radar_fresh=radar_fresh,
                    radar_closest_m=radar_closest_m,
                )
                if display_enabled:
                    dashboard = dashboard_worker.get_latest()
                    if dashboard is not None:
                        cv2.imshow(args.window_name, dashboard)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (27, ord("q")):
                        break
                time.sleep(0.05)
                continue

            infer_start = time.time()
            outputs = node.sess.run(None, {node.input_name: preprocess_bgr(color_image)})
            mask = postprocess_mask(outputs)
            infer_ms = (time.time() - infer_start) * 1000.0

            confirmed, distance_m, direction, vision_pixels, reason = analyze_fusion(
                depth_frame=depth_frame,
                mask=mask,
                obstacle_class=args.obstacle_class,
            )
            node.publish_decision(confirmed, distance_m, direction)

            now = time.monotonic()
            if last_inference_t is not None:
                dt = now - last_inference_t
                instant_fps = 1.0 / dt if dt > 0.0 else 0.0
                if fps <= 0.0:
                    fps = instant_fps
                else:
                    fps = 0.85 * fps + 0.15 * instant_fps
            last_inference_t = now
            frame_id += 1

            if (frame_id - 1) % dashboard_stride == 0:
                dashboard_worker.submit_dashboard(
                    color_image=color_image.copy(),
                    depth_frame=DepthFrameSnapshot(depth_frame),
                    mask=mask.copy(),
                    obstacle_class=args.obstacle_class,
                    confirmed=confirmed,
                    distance_m=distance_m,
                    direction=direction,
                    vision_pixels=vision_pixels,
                    reason=reason,
                    fps=fps,
                    infer_ms=infer_ms,
                    radar_objects=[dict(obj) for obj in radar_objects],
                    radar_fresh=radar_fresh,
                    radar_closest_m=radar_closest_m,
                )

            if display_enabled:
                dashboard = dashboard_worker.get_latest()
                if dashboard is not None:
                    cv2.imshow(args.window_name, dashboard)

            if display_enabled:
                key = cv2.waitKey(1) & 0xFF
                if key in (27, ord("q")):
                    break
            if args.frames and frame_id >= args.frames:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if dashboard_worker is not None:
            dashboard_worker.stop()
            node.get_logger().info(
                "Performance summary: "
                f"inference={fps:.1f} FPS, "
                f"dashboard={dashboard_worker.dashboard_fps:.1f} FPS, "
                f"dashboard_stride={dashboard_stride}"
            )
        if streamer is not None:
            streamer.stop()
        node.close()
        node.destroy_node()
        if display_enabled:
            cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
