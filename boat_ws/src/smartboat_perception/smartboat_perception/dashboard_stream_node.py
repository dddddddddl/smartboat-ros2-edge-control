"""Independent RealSense/EWaSR dashboard renderer and SRT publisher."""

import math
from pathlib import Path
import shlex
import subprocess
import time

from cv_bridge import CvBridge
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from smartboat_msgs.msg import (
    FusionDetection,
    RadarStatus,
    RadarTargetArray,
)
from smartboat_perception.dashboard_text import CachedChineseText
from smartboat_perception.dashboard_pipeline import (
    ExactFrameSynchronizer,
    LatestOnlyWorker,
)


TEXT_MAIN = (245, 245, 245)
TEXT_NORMAL = (215, 225, 235)
TEXT_MUTED = (180, 195, 210)
TEXT_REASON = (190, 205, 220)
NO_OBSTACLE_COLOR = (60, 220, 80)
RADAR_OBSTACLE_COLOR = (0, 200, 255)
VISION_OBSTACLE_COLOR = (0, 140, 255)
BOTH_OBSTACLE_COLOR = (0, 80, 255)


REASON_CHINESE = {
    "vision_no_obstacle": "视觉未发现障碍",
    "vision_only_depth_invalid_ignore": "视觉目标深度无效",
    "obstacle_far_enough": "视觉目标距离较远",
    "obstacle_on_left_turn_right": "障碍在左侧",
    "obstacle_on_right_turn_left": "障碍在右侧",
    "obstacle_center_left_side_freer": "中央障碍，左侧较空",
    "obstacle_center_right_side_freer": "中央障碍，右侧较空",
    "realsense_start_failed": "深度相机启动失败",
    "realsense_read_failed": "深度相机读取失败",
}


def decision_chinese(decision):
    if decision == FusionDetection.TURN_LEFT:
        return "左转绕行"
    if decision == FusionDetection.TURN_RIGHT:
        return "右转绕行"
    return "保持直行"


def avoidance_type_chinese(radar_detected, vision_detected):
    if radar_detected and vision_detected:
        return "雷达与视觉均探测到障碍", BOTH_OBSTACLE_COLOR
    if vision_detected:
        return "视觉探测到障碍", VISION_OBSTACLE_COLOR
    if radar_detected:
        return "雷达探测到障碍", RADAR_OBSTACLE_COLOR
    return "无障碍", NO_OBSTACLE_COLOR


def dashboard_text_items():
    """Return all Chinese sprites needed during normal frame rendering."""
    items = [
        ("智能船避障状态", 28, TEXT_MAIN, True),
        ("建议动作：保持直行", 24, TEXT_MAIN, False),
        ("建议动作：左转绕行", 24, TEXT_MAIN, False),
        ("建议动作：右转绕行", 24, TEXT_MAIN, False),
        ("雷达：", 21, TEXT_NORMAL, False),
        ("等待雷达数据", 21, TEXT_NORMAL, False),
        ("个目标", 19, TEXT_NORMAL, False),
        ("最近", 19, TEXT_NORMAL, False),
        ("米", 19, TEXT_NORMAL, False),
        ("视觉：已确认障碍", 21, TEXT_NORMAL, False),
        ("视觉：未确认近距离障碍", 21, TEXT_NORMAL, False),
        ("视觉：等待深度相机", 21, TEXT_NORMAL, False),
        ("视觉像素", 19, TEXT_MUTED, False),
        ("推理", 19, TEXT_MUTED, False),
        ("画面", 19, TEXT_MUTED, False),
        ("数据状态", 19, TEXT_MUTED, False),
        ("输出", 19, TEXT_MUTED, False),
        ("语义分割彩色遮罩", 24, TEXT_MAIN, True),
        ("障碍", 17, TEXT_MAIN, False),
        ("水面", 17, TEXT_MAIN, False),
        ("天空", 17, TEXT_MAIN, False),
        ("毫米波雷达目标（3–10米）", 24, TEXT_MAIN, True),
        ("深度相机（0–3米）", 24, TEXT_MAIN, True),
        ("等待彩色相机", 26, TEXT_MUTED, False),
        ("等待深度相机", 26, TEXT_MUTED, False),
    ]
    for label, color in (
        ("无障碍", NO_OBSTACLE_COLOR),
        ("雷达探测到障碍", RADAR_OBSTACLE_COLOR),
        ("视觉探测到障碍", VISION_OBSTACLE_COLOR),
        ("雷达与视觉均探测到障碍", BOTH_OBSTACLE_COLOR),
    ):
        items.append((f"避障类型：{label}", 32, color, True))
    for reason in set(REASON_CHINESE.values()) | {"等待视觉判断"}:
        items.append((f"判定：{reason}", 20, TEXT_REASON, False))
    return items


def read_text_or_default(path, default):
    try:
        value = Path(path).read_text(encoding="utf-8").strip()
        if value:
            return value
    except OSError:
        pass
    return default


def build_srt_url(
    server,
    port,
    identity_dir,
    station_default,
    boat_default,
    stream_suffix="",
    stream_path="",
    stream_url="",
):
    """Preserve the established station/boat SRT publishing convention."""
    if stream_url:
        return stream_url
    if not stream_path:
        base = Path(identity_dir)
        station = read_text_or_default(base / "name.txt", station_default)
        boat = read_text_or_default(base / "boat_name.txt", boat_default)
        suffix = stream_suffix.strip("/")
        stream_path = (
            f"{station}/{boat}_{suffix}" if suffix else f"{station}/{boat}"
        )
    stream_id = f"publish:{stream_path}"
    return (
        f"srt://{server}:{int(port)}"
        f"?streamid={stream_id}&pkt_size=1316&mode=caller"
    )


class DashboardStreamer:
    """Restartable raw-BGR to H.264/MPEG-TS/SRT ffmpeg pipe."""

    def __init__(
        self,
        *,
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
        self.fps = max(1, int(fps))
        self.width = int(width)
        self.height = int(height)
        self.codec = codec
        self.bitrate = bitrate
        self.loglevel = loglevel
        self.logger = logger
        self.restart_delay_sec = float(restart_delay_sec)
        self.process = None
        self.last_start_attempt = 0.0

    def build_command(self):
        command = [
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
            "-framerate",
            str(self.fps),
            "-use_wallclock_as_timestamps",
            "1",
            "-i",
            "pipe:0",
            "-an",
            "-c:v",
            self.codec,
        ]
        if self.codec == "libx264":
            command.extend([
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
        command.extend([
            "-g",
            str(self.fps),
            "-b:v",
            self.bitrate,
            "-maxrate",
            self.bitrate,
            "-bufsize",
            self.bitrate,
            "-pix_fmt",
            "yuv420p",
            "-vsync",
            "vfr",
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
        return command

    def _start(self):
        now = time.monotonic()
        if now - self.last_start_attempt < self.restart_delay_sec:
            return False
        self.last_start_attempt = now
        command = self.build_command()
        self.logger.warning(
            "Starting dashboard stream: "
            + " ".join(shlex.quote(part) for part in command)
        )
        try:
            self.process = subprocess.Popen(command, stdin=subprocess.PIPE)
        except OSError as exc:
            self.logger.error(f"Failed to start dashboard ffmpeg: {exc}")
            self.process = None
            return False
        return True

    def write(self, frame):
        if frame.shape[:2] != (self.height, self.width):
            frame = cv2.resize(
                frame,
                (self.width, self.height),
                interpolation=cv2.INTER_AREA,
            )
        if self.process is None or self.process.poll() is not None:
            self.stop()
            if not self._start():
                return
        try:
            self.process.stdin.write(
                np.ascontiguousarray(frame, dtype=np.uint8).tobytes()
            )
        except (BrokenPipeError, OSError) as exc:
            self.logger.warning(f"Dashboard stream pipe failed: {exc}")
            self.stop()

    def stop(self):
        if self.process is None:
            return
        if self.process.stdin is not None:
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


class DashboardStreamNode(Node):
    """Render telemetry without owning the D435i or safety publications."""

    def __init__(self):
        super().__init__("dashboard_stream")
        self.declare_parameter("stream_url", "")
        self.declare_parameter("stream_server", "111.229.242.175")
        self.declare_parameter("stream_port", 10000)
        self.declare_parameter("stream_path", "")
        self.declare_parameter("stream_suffix", "")
        self.declare_parameter("identity_dir", "/host_boot")
        self.declare_parameter("station_default", "test")
        self.declare_parameter("boat_default", "test01")
        self.declare_parameter("stream_fps", 15)
        self.declare_parameter("stream_width", 1280)
        self.declare_parameter("stream_height", 960)
        self.declare_parameter("stream_bitrate", "1800k")
        self.declare_parameter("stream_codec", "libx264")
        self.declare_parameter("stream_loglevel", "warning")
        self.declare_parameter("ffmpeg_path", "ffmpeg")
        self.declare_parameter("input_timeout_sec", 1.0)

        self.bridge = CvBridge()
        self.color = None
        self.depth_m = None
        self.mask = None
        self.detection = None
        self.latest_detection = None
        self.radar_status = None
        self.radar_targets = []
        self.last_color_time = None
        self.last_detection_time = None
        self.last_radar_time = None
        self.perception_fps = 0.0
        self.dashboard_fps = 0.0
        self.last_render_time = None
        self.last_visual_bundle_time = None
        self.last_status_render_time = None
        self.render_count = 0
        self.max_render_interval_sec = 0.0
        self.max_pipeline_ms = 0.0
        self.text = CachedChineseText()
        self.text.prewarm(dashboard_text_items())
        self.frame_sync = ExactFrameSynchronizer()

        self.create_subscription(
            Image,
            "/smartboat/perception/ewasr/color",
            self._color_callback,
            1,
        )
        self.create_subscription(
            Image,
            "/smartboat/perception/ewasr/depth_m",
            self._depth_callback,
            1,
        )
        self.create_subscription(
            Image,
            "/smartboat/perception/ewasr/mask",
            self._mask_callback,
            1,
        )
        self.create_subscription(
            FusionDetection,
            "/smartboat/perception/fusion/detection",
            self._detection_callback,
            10,
        )
        self.create_subscription(
            RadarTargetArray,
            "/smartboat/perception/radar/targets",
            self._radar_targets_callback,
            10,
        )
        self.create_subscription(
            RadarStatus,
            "/smartboat/perception/radar/typed_status",
            self._radar_status_callback,
            10,
        )

        target_url = build_srt_url(
            str(self.get_parameter("stream_server").value),
            int(self.get_parameter("stream_port").value),
            str(self.get_parameter("identity_dir").value),
            str(self.get_parameter("station_default").value),
            str(self.get_parameter("boat_default").value),
            str(self.get_parameter("stream_suffix").value),
            str(self.get_parameter("stream_path").value),
            str(self.get_parameter("stream_url").value),
        )
        self.width = int(self.get_parameter("stream_width").value)
        self.height = int(self.get_parameter("stream_height").value)
        self.input_timeout_sec = float(
            self.get_parameter("input_timeout_sec").value
        )
        fps = int(self.get_parameter("stream_fps").value)
        self.streamer = DashboardStreamer(
            target_url=target_url,
            ffmpeg_path=str(self.get_parameter("ffmpeg_path").value),
            fps=fps,
            width=self.width,
            height=self.height,
            codec=str(self.get_parameter("stream_codec").value),
            bitrate=str(self.get_parameter("stream_bitrate").value),
            loglevel=str(self.get_parameter("stream_loglevel").value),
            logger=self.get_logger(),
        )
        self.render_worker = LatestOnlyWorker(self._process_bundle)
        self.create_timer(1.0, self._refresh_status_if_stale)

    def _color_callback(self, msg):
        self._accept_visual_message("color", msg)

    def _depth_callback(self, msg):
        self._accept_visual_message("depth", msg)

    def _mask_callback(self, msg):
        self._accept_visual_message("mask", msg)

    def _detection_callback(self, msg):
        now = time.monotonic()
        if self.last_detection_time is not None:
            elapsed = now - self.last_detection_time
            if elapsed > 0.0:
                instant_fps = 1.0 / elapsed
                if self.perception_fps <= 0.0:
                    self.perception_fps = instant_fps
                else:
                    self.perception_fps = (
                        self.perception_fps * 0.85 + instant_fps * 0.15
                    )
        self.latest_detection = msg
        self.last_detection_time = now
        self._accept_visual_message("detection", msg)

    def _accept_visual_message(self, kind, msg):
        bundle = self.frame_sync.add(kind, msg)
        if bundle is None:
            return
        self.last_visual_bundle_time = time.monotonic()
        self.render_worker.submit(bundle)

    def _process_bundle(self, bundle):
        if bundle is None:
            self.color = None
            self.depth_m = None
            self.mask = None
            self.detection = self.latest_detection
        else:
            self.color = self.bridge.imgmsg_to_cv2(
                bundle.color,
                desired_encoding="bgr8",
            )
            self.depth_m = self.bridge.imgmsg_to_cv2(
                bundle.depth,
                desired_encoding="32FC1",
            )
            self.mask = self.bridge.imgmsg_to_cv2(
                bundle.mask,
                desired_encoding="mono8",
            )
            self.detection = bundle.detection
            self.last_color_time = time.monotonic()
        self._render_and_stream(is_visual=bundle is not None)

    def _refresh_status_if_stale(self):
        now = time.monotonic()
        if (
            self.last_visual_bundle_time is not None
            and now - self.last_visual_bundle_time <= self.input_timeout_sec
        ):
            return
        if (
            self.last_status_render_time is not None
            and now - self.last_status_render_time < 1.0
        ):
            return
        self.last_status_render_time = now
        self.render_worker.submit(None)

    def _radar_targets_callback(self, msg):
        self.radar_targets = list(msg.targets)
        self.last_radar_time = time.monotonic()

    def _radar_status_callback(self, msg):
        self.radar_status = msg
        self.last_radar_time = time.monotonic()

    def _recent(self, timestamp):
        return (
            timestamp is not None
            and time.monotonic() - timestamp <= self.input_timeout_sec
        )

    @staticmethod
    def _put_ascii(panel, text, xy, scale=0.55, color=TEXT_NORMAL):
        cv2.putText(
            panel,
            str(text),
            (int(xy[0]), int(xy[1])),
            cv2.FONT_HERSHEY_SIMPLEX,
            float(scale),
            color,
            1,
            cv2.LINE_AA,
        )

    def _title(self, panel, text):
        cv2.rectangle(panel, (0, 0), (panel.shape[1], 44), (12, 15, 21), -1)
        self.text.draw(panel, text, (16, 8), 24, TEXT_MAIN, True)

    def _status_panel(self, width, height):
        detection_fresh = self._recent(self.last_detection_time)
        sensor_ready = bool(
            detection_fresh
            and self.detection is not None
            and self.detection.sensor_ready
        )
        vision_detected = bool(
            sensor_ready and self.detection.obstacle_detected
        )
        radar_fresh = bool(
            self._recent(self.last_radar_time)
            and self.radar_status is not None
            and self.radar_status.connected
            and self.radar_status.data_fresh
        )
        radar_detected = radar_fresh and bool(self.radar_targets)
        obstacle_type, accent = avoidance_type_chinese(
            radar_detected,
            vision_detected,
        )

        panel = np.full((height, width, 3), (28, 32, 40), dtype=np.uint8)
        cv2.rectangle(panel, (0, 0), (10, height), accent, -1)
        cv2.rectangle(panel, (10, 0), (width, 58), (16, 19, 26), -1)
        for y in (132, 198, 264):
            cv2.line(panel, (28, y), (width - 28, y), (62, 68, 78), 1)

        self.text.draw(
            panel,
            "智能船避障状态",
            (28, 12),
            28,
            TEXT_MAIN,
            True,
        )
        self.text.draw(
            panel,
            f"避障类型：{obstacle_type}",
            (30, 72),
            32,
            accent,
            True,
        )
        decision = (
            decision_chinese(self.detection.decision)
            if detection_fresh and self.detection is not None
            else "保持直行"
        )
        self.text.draw(
            panel,
            f"建议动作：{decision}",
            (30, 142),
            24,
            TEXT_MAIN,
        )
        self.text.draw(panel, "雷达：", (30, 174), 21, TEXT_NORMAL)
        if radar_fresh:
            self._put_ascii(panel, len(self.radar_targets), (98, 195))
            self.text.draw(panel, "个目标", (128, 174), 19, TEXT_NORMAL)
            distances = [
                float(target.distance_m)
                for target in self.radar_targets
                if float(target.distance_m) > 0.0
            ]
            if distances:
                self.text.draw(panel, "最近", (220, 174), 19, TEXT_NORMAL)
                self._put_ascii(panel, f"{min(distances):.2f}", (278, 195))
                self.text.draw(panel, "米", (360, 174), 19, TEXT_NORMAL)
        else:
            self.text.draw(
                panel,
                "等待雷达数据",
                (96, 174),
                21,
                TEXT_NORMAL,
            )

        if not sensor_ready:
            vision_label = "视觉：等待深度相机"
        elif vision_detected:
            vision_label = "视觉：已确认障碍"
        else:
            vision_label = "视觉：未确认近距离障碍"
        self.text.draw(panel, vision_label, (30, 208), 21, TEXT_NORMAL)
        if sensor_ready and self.detection.distance_m >= 0.0:
            self._put_ascii(
                panel,
                f"{self.detection.distance_m:.2f} m",
                (375, 229),
            )

        reason = "等待视觉判断"
        if detection_fresh and self.detection is not None:
            reason = REASON_CHINESE.get(
                self.detection.reason,
                "等待视觉判断",
            )
        self.text.draw(
            panel,
            f"判定：{reason}",
            (30, 240),
            20,
            TEXT_REASON,
        )

        self.text.draw(panel, "视觉像素", (30, 278), 19, TEXT_MUTED)
        obstacle_pixels = (
            int(self.detection.obstacle_pixels)
            if detection_fresh and self.detection is not None
            else 0
        )
        self._put_ascii(panel, obstacle_pixels, (120, 299), color=TEXT_MUTED)
        self.text.draw(panel, "推理", (230, 278), 19, TEXT_MUTED)
        inference_ms = (
            float(self.detection.inference_ms)
            if detection_fresh and self.detection is not None
            else 0.0
        )
        self._put_ascii(
            panel,
            f"{self.perception_fps:.1f} FPS / {inference_ms:.1f} ms",
            (282, 299),
            scale=0.52,
            color=TEXT_MUTED,
        )
        self.text.draw(panel, "画面", (30, 310), 19, TEXT_MUTED)
        self._put_ascii(
            panel,
            f"{self.dashboard_fps:.1f} FPS",
            (82, 331),
            scale=0.52,
            color=TEXT_MUTED,
        )
        self.text.draw(panel, "数据状态", (220, 310), 19, TEXT_MUTED)
        data_flags = (
            int(self._recent(self.last_color_time)),
            int(detection_fresh),
            int(radar_fresh),
        )
        self._put_ascii(
            panel,
            f"C/F/R {data_flags[0]}/{data_flags[1]}/{data_flags[2]}",
            (310, 331),
            scale=0.50,
            color=TEXT_MUTED,
        )
        self.text.draw(panel, "输出", (455, 310), 19, TEXT_MUTED)
        self._put_ascii(
            panel,
            f"{self.width}x{self.height}",
            (505, 331),
            scale=0.48,
            color=TEXT_MUTED,
        )
        return panel

    def _semantic_panel(self, width, height):
        if self.color is None:
            panel = np.full((height, width, 3), (25, 25, 25), dtype=np.uint8)
            self.text.draw(
                panel,
                "等待彩色相机",
                (width // 2 - 90, height // 2 - 20),
                26,
                TEXT_MUTED,
            )
        else:
            panel = cv2.resize(self.color, (width, height))
            if self.mask is not None:
                mask = cv2.resize(
                    self.mask,
                    (width, height),
                    interpolation=cv2.INTER_NEAREST,
                )
                colors = np.zeros_like(panel)
                colors[mask == 0] = (0, 0, 255)
                colors[mask == 1] = (255, 0, 0)
                colors[mask == 2] = (255, 0, 255)
                panel = cv2.addWeighted(panel, 0.68, colors, 0.32, 0.0)
        self._title(panel, "语义分割彩色遮罩")
        for x, color, label in (
            (330, (0, 0, 255), "障碍"),
            (420, (255, 0, 0), "水面"),
            (510, (255, 0, 255), "天空"),
        ):
            if x + 75 > width:
                continue
            cv2.rectangle(panel, (x, 13), (x + 18, 31), color, -1)
            self.text.draw(panel, label, (x + 25, 12), 17, TEXT_MAIN)
        return panel

    def _depth_panel(self, width, height):
        if self.depth_m is None:
            panel = np.full((height, width, 3), (25, 25, 25), dtype=np.uint8)
            self.text.draw(
                panel,
                "等待深度相机",
                (width // 2 - 90, height // 2 - 20),
                26,
                TEXT_MUTED,
            )
        else:
            depth = np.nan_to_num(self.depth_m, nan=0.0, posinf=0.0)
            normalized = np.clip(depth / 3.0 * 255.0, 0, 255).astype(np.uint8)
            normalized[depth <= 0.0] = 0
            panel = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
            panel = cv2.resize(panel, (width, height))
        self._title(panel, "深度相机（0–3米）")
        return panel

    def _radar_panel(self, width, height):
        panel = np.full((height, width, 3), (22, 26, 34), dtype=np.uint8)
        self._title(panel, "毫米波雷达目标（3–10米）")
        origin = (width // 2, height - 15)
        radius = int(min(width * 0.43, height * 0.72))
        scale = radius / 10.0
        for distance in (3.0, 5.0, 10.0):
            points = []
            for bearing in np.linspace(-56.0, 56.0, 70):
                angle = math.radians(bearing)
                points.append((
                    int(origin[0] + distance * scale * math.sin(angle)),
                    int(origin[1] - distance * scale * math.cos(angle)),
                ))
            cv2.polylines(
                panel,
                [np.asarray(points, dtype=np.int32)],
                False,
                (75, 85, 100),
                1,
            )
        for target in self.radar_targets[:16]:
            distance = float(target.distance_m)
            if not (0.0 < distance <= 10.0):
                continue
            angle = math.radians(float(target.bearing_deg))
            point = (
                int(origin[0] + distance * scale * math.sin(angle)),
                int(origin[1] - distance * scale * math.cos(angle)),
            )
            cv2.circle(panel, point, 7, (0, 80, 255), -1)
            cv2.circle(panel, point, 10, (235, 235, 235), 1, cv2.LINE_AA)
            self._put_ascii(
                panel,
                f"{int(target.id)}:{distance:.1f}m",
                (point[0] + 10, point[1] - 6),
                scale=0.42,
                color=(240, 240, 240),
            )
        return panel

    def _render_and_stream(self, is_visual=True):
        pipeline_started = time.monotonic()
        now = time.monotonic()
        if is_visual and self.last_render_time is not None:
            elapsed = now - self.last_render_time
            if self.render_count >= 10:
                self.max_render_interval_sec = max(
                    self.max_render_interval_sec,
                    elapsed,
                )
            if elapsed > 0.0:
                instant_fps = 1.0 / elapsed
                if self.dashboard_fps <= 0.0:
                    self.dashboard_fps = instant_fps
                else:
                    self.dashboard_fps = (
                        self.dashboard_fps * 0.85 + instant_fps * 0.15
                    )
        if is_visual:
            self.last_render_time = now
        half_width = self.width // 2
        half_height = self.height // 2
        dashboard = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        dashboard[:half_height, :half_width] = self._status_panel(
            half_width,
            half_height,
        )
        dashboard[:half_height, half_width:] = self._semantic_panel(
            self.width - half_width,
            half_height,
        )
        dashboard[half_height:, :half_width] = self._radar_panel(
            half_width,
            self.height - half_height,
        )
        dashboard[half_height:, half_width:] = self._depth_panel(
            self.width - half_width,
            self.height - half_height,
        )
        self.streamer.write(dashboard)
        if is_visual:
            self.render_count += 1
            if self.render_count >= 10:
                self.max_pipeline_ms = max(
                    self.max_pipeline_ms,
                    (time.monotonic() - pipeline_started) * 1000.0,
                )

    def destroy_node(self):
        self.render_worker.stop(timeout=0.1)
        self.streamer.stop()
        self.render_worker.join(timeout=2.0)
        print(
            "Dashboard pipeline stopped | "
            f"rendered={self.render_count} | "
            f"queue_dropped={self.render_worker.dropped} | "
            f"max_interval_ms={self.max_render_interval_sec * 1000.0:.1f} | "
            f"max_render_and_write_ms={self.max_pipeline_ms:.1f}",
            flush=True,
        )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DashboardStreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
