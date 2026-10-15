"""Cached Chinese text sprites for the production dashboard."""

from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont


REGULAR_FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Regular.otf",
    "/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
)

BOLD_FONT_CANDIDATES = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc",
    "/usr/share/fonts/opentype/noto/NotoSansCJKsc-Bold.otf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
)


class CachedChineseText:
    """Render fixed Unicode labels once and composite them with NumPy."""

    def __init__(self):
        self._fonts = {}
        self._sprites = {}

    def _font(self, size, bold):
        key = (int(size), bool(bold))
        if key in self._fonts:
            return self._fonts[key]
        candidates = BOLD_FONT_CANDIDATES if bold else REGULAR_FONT_CANDIDATES
        for candidate in candidates:
            if Path(candidate).is_file():
                font = ImageFont.truetype(candidate, int(size))
                self._fonts[key] = font
                return font
        font = ImageFont.load_default()
        self._fonts[key] = font
        return font

    def _render(self, text, size, bgr, bold):
        color = tuple(int(channel) for channel in bgr)
        key = (str(text), int(size), color, bool(bold))
        if key in self._sprites:
            return self._sprites[key]

        font = self._font(size, bold)
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
        self._sprites[key] = sprite
        return sprite

    def prewarm(self, items):
        """Use Pillow only during startup to prepare all runtime labels."""
        for text, size, bgr, bold in items:
            self._render(text, size, bgr, bold)

    def draw(self, image, text, xy, size, bgr, bold=False):
        """Composite a previously rendered BGRA sprite onto a BGR image."""
        color = tuple(int(channel) for channel in bgr)
        key = (str(text), int(size), color, bool(bold))
        sprite = self._sprites[key]
        x, y = int(xy[0]), int(xy[1])
        sprite_height, sprite_width = sprite.shape[:2]
        x0 = max(0, x)
        y0 = max(0, y)
        x1 = min(image.shape[1], x + sprite_width)
        y1 = min(image.shape[0], y + sprite_height)
        if x0 >= x1 or y0 >= y1:
            return

        sprite_roi = sprite[y0 - y:y1 - y, x0 - x:x1 - x]
        alpha = sprite_roi[:, :, 3:4].astype(np.float32) / 255.0
        background = image[y0:y1, x0:x1].astype(np.float32)
        foreground = sprite_roi[:, :, :3].astype(np.float32)
        image[y0:y1, x0:x1] = (
            foreground * alpha + background * (1.0 - alpha)
        ).astype(np.uint8)

    @property
    def sprite_count(self):
        return len(self._sprites)
