from types import SimpleNamespace
import threading
import time

from smartboat_perception.dashboard_pipeline import (
    ExactFrameSynchronizer,
    LatestOnlyWorker,
)


def message(stamp_ns, value):
    stamp = SimpleNamespace(
        sec=stamp_ns // 1_000_000_000,
        nanosec=stamp_ns % 1_000_000_000,
    )
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp),
        value=value,
    )


def test_exact_synchronizer_waits_for_one_coherent_frame():
    sync = ExactFrameSynchronizer()
    assert sync.add("color", message(100, "color-1")) is None
    assert sync.add("depth", message(200, "depth-2")) is None
    assert sync.add("mask", message(100, "mask-1")) is None
    assert sync.add("detection", message(100, "detection-1")) is None
    bundle = sync.add("depth", message(100, "depth-1"))
    assert bundle.stamp_ns == 100
    assert bundle.color.value == "color-1"
    assert bundle.depth.value == "depth-1"
    assert bundle.mask.value == "mask-1"
    assert bundle.detection.value == "detection-1"


def test_latest_worker_drops_queued_stale_frame():
    started = threading.Event()
    release = threading.Event()
    processed = []

    def handler(item):
        processed.append(item)
        if item == 1:
            started.set()
            release.wait(timeout=1.0)

    worker = LatestOnlyWorker(handler)
    worker.submit(1)
    assert started.wait(timeout=1.0)
    worker.submit(2)
    worker.submit(3)
    release.set()
    deadline = time.monotonic() + 1.0
    while len(processed) < 2 and time.monotonic() < deadline:
        time.sleep(0.01)
    worker.stop()

    assert processed == [1, 3]
    assert worker.dropped == 1
    assert not worker.alive
