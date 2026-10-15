"""Non-blocking JSONL recorder for safety-critical bridge transitions."""

import datetime
import json
import os
import queue
import threading
import time


class SafetyEventRecorder:
    def __init__(self, path, *, enabled=True, queue_size=512):
        self.path = str(path)
        self.enabled = bool(enabled) and bool(self.path)
        self._queue = queue.Queue(maxsize=max(1, int(queue_size)))
        self._stop = threading.Event()
        self._thread = None
        self.error = None
        self.dropped_events = 0
        if self.enabled:
            self._thread = threading.Thread(
                target=self._run,
                name='smartboat_safety_event_recorder',
                daemon=True,
            )
            self._thread.start()

    def record(self, event, **fields):
        if not self.enabled or self._stop.is_set():
            return False
        item = {
            'wall_time_utc': datetime.datetime.now(
                datetime.timezone.utc
            ).isoformat(),
            'monotonic_sec': time.monotonic(),
            'event': str(event),
            **fields,
        }
        try:
            self._queue.put_nowait(item)
            return True
        except queue.Full:
            self.dropped_events += 1
            return False

    def close(self, timeout_sec=2.0):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=max(0.0, float(timeout_sec)))

    def _run(self):
        try:
            parent = os.path.dirname(os.path.abspath(self.path))
            os.makedirs(parent, exist_ok=True)
            with open(self.path, 'a', encoding='utf-8', buffering=1) as stream:
                while not self._stop.is_set() or not self._queue.empty():
                    try:
                        item = self._queue.get(timeout=0.2)
                    except queue.Empty:
                        continue
                    stream.write(
                        json.dumps(
                            item,
                            ensure_ascii=False,
                            separators=(',', ':'),
                        ) + '\n'
                    )
                    self._queue.task_done()
        except Exception as exc:
            self.error = str(exc)
