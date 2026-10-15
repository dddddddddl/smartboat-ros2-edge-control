"""RealSense D435i depth/color acquisition adapter."""

import numpy as np
import pyrealsense2 as rs


class RealSenseSource:
    """Own and recover a D435i pipeline without ROS or streaming concerns."""

    def __init__(self, width=640, height=480, fps=30):
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)
        self.pipeline = None
        self.align = None
        self.depth_scale = 0.001

    @property
    def started(self):
        return self.pipeline is not None

    def start(self):
        self.stop()
        config = rs.config()
        config.enable_stream(
            rs.stream.depth,
            self.width,
            self.height,
            rs.format.z16,
            self.fps,
        )
        config.enable_stream(
            rs.stream.color,
            self.width,
            self.height,
            rs.format.bgr8,
            self.fps,
        )
        pipeline = rs.pipeline()
        profile = pipeline.start(config)
        sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = float(sensor.get_depth_scale())
        self.pipeline = pipeline
        self.align = rs.align(rs.stream.color)

    def read(self, timeout_ms=1000):
        if self.pipeline is None or self.align is None:
            raise RuntimeError("RealSense pipeline is not started")
        frames = self.align.process(
            self.pipeline.wait_for_frames(timeout_ms=int(timeout_ms))
        )
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            raise RuntimeError("RealSense returned an incomplete frame set")
        color = np.asanyarray(color_frame.get_data())
        depth_raw = np.asanyarray(depth_frame.get_data())
        depth_m = depth_raw.astype(np.float32) * self.depth_scale
        return color, depth_m

    def stop(self):
        if self.pipeline is not None:
            try:
                self.pipeline.stop()
            except RuntimeError:
                pass
        self.pipeline = None
        self.align = None
