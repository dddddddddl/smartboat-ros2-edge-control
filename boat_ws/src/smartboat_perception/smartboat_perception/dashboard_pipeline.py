"""Frame synchronization and latest-only work queue for the dashboard."""

from dataclasses import dataclass
import queue
import threading


def stamp_nanoseconds(message):
    """Return a ROS message header timestamp as one integer."""
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


@dataclass(frozen=True)
class VisualizationBundle:
    """One coherent inference result and its three visualization images."""

    stamp_ns: int
    detection: object
    color: object
    depth: object
    mask: object


class ExactFrameSynchronizer:
    """Match visualization messages by their exact inference timestamp."""

    REQUIRED = ("detection", "color", "depth", "mask")

    def __init__(self, max_pending=8):
        self.max_pending = max(4, int(max_pending))
        self._pending = {}
        self._last_emitted = -1

    def add(self, kind, message):
        """Add one message and return a complete bundle when available."""
        if kind not in self.REQUIRED:
            raise ValueError(f"unsupported visualization message: {kind}")
        stamp_ns = stamp_nanoseconds(message)
        if stamp_ns <= self._last_emitted:
            return None
        bucket = self._pending.setdefault(stamp_ns, {})
        bucket[kind] = message
        self._prune()
        if not all(name in bucket for name in self.REQUIRED):
            return None

        bundle = VisualizationBundle(
            stamp_ns=stamp_ns,
            detection=bucket["detection"],
            color=bucket["color"],
            depth=bucket["depth"],
            mask=bucket["mask"],
        )
        self._last_emitted = stamp_ns
        self._pending = {
            key: value
            for key, value in self._pending.items()
            if key > stamp_ns
        }
        return bundle

    def _prune(self):
        if len(self._pending) <= self.max_pending:
            return
        keep = sorted(self._pending)[-self.max_pending:]
        self._pending = {key: self._pending[key] for key in keep}


class LatestOnlyWorker:
    """Run slow dashboard work off the ROS executor and drop stale work."""

    _STOP = object()

    def __init__(self, handler, name="smartboat-dashboard-render"):
        self._handler = handler
        self._queue = queue.Queue(maxsize=1)
        self._stopping = False
        self.dropped = 0
        self.processed = 0
        self._thread = threading.Thread(
            target=self._run,
            name=name,
            daemon=True,
        )
        self._thread.start()

    def submit(self, item):
        """Keep the newest item if rendering or streaming falls behind."""
        if self._stopping:
            return False
        try:
            self._queue.put_nowait(item)
            return True
        except queue.Full:
            try:
                self._queue.get_nowait()
                self._queue.task_done()
            except queue.Empty:
                pass
            self.dropped += 1
            self._queue.put_nowait(item)
            return True

    def _run(self):
        while True:
            item = self._queue.get()
            try:
                if item is self._STOP:
                    return
                self._handler(item)
                self.processed += 1
            finally:
                self._queue.task_done()

    def stop(self, timeout=3.0):
        """Stop after discarding any queued stale frame."""
        if self._stopping:
            return
        self._stopping = True
        try:
            item = self._queue.get_nowait()
            self._queue.task_done()
            del item
        except queue.Empty:
            pass
        try:
            self._queue.put_nowait(self._STOP)
        except queue.Full:
            pass
        self._thread.join(timeout=float(timeout))

    def join(self, timeout=3.0):
        """Wait for a previously requested stop to finish."""
        self._thread.join(timeout=float(timeout))

    @property
    def alive(self):
        return self._thread.is_alive()
