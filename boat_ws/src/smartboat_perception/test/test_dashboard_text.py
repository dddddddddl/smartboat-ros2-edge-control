import numpy as np

from smartboat_perception.dashboard_text import CachedChineseText


def test_prewarmed_chinese_sprite_draws_without_runtime_rendering():
    renderer = CachedChineseText()
    item = ("毫米波雷达目标（3–10米）", 24, (245, 245, 245), True)
    renderer.prewarm([item])
    assert renderer.sprite_count == 1

    image = np.zeros((80, 420, 3), dtype=np.uint8)
    renderer._render = None
    renderer.draw(image, item[0], (8, 8), item[1], item[2], item[3])
    assert np.count_nonzero(image) > 0
