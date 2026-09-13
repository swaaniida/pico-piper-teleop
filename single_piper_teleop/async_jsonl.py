"""Bounded asynchronous JSONL writer for latency-sensitive control loops."""

from __future__ import annotations

import os
from pathlib import Path
import queue
import threading


class AsyncJsonlWriter:
    """File-like line writer that performs removable-media I/O off-thread."""

    def __init__(self, path: Path, max_pending_lines: int = 5000) -> None:
        self._stream = path.open("w", encoding="utf-8")
        self._queue: queue.Queue[str | None] = queue.Queue(max_pending_lines)
        self._error: Exception | None = None
        self._closed = False
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while True:
            line = self._queue.get()
            try:
                if line is None:
                    return
                if self._error is None:
                    try:
                        self._stream.write(line)
                    except Exception as exc:
                        self._error = exc
            finally:
                self._queue.task_done()

    def _raise_if_failed(self) -> None:
        if self._error is not None:
            raise RuntimeError(f"episode JSONL write failed: {self._error}")

    def write(self, line: str) -> None:
        if self._closed:
            raise RuntimeError("episode JSONL writer is closed")
        self._raise_if_failed()
        try:
            self._queue.put_nowait(line)
        except queue.Full as exc:
            raise RuntimeError("episode JSONL USB queue exceeded 100 seconds") from exc

    def flush(self) -> None:
        if self._closed:
            return
        self._queue.join()
        self._raise_if_failed()
        self._stream.flush()
            os.fsync(self._stream.fileno())
    def close(self) -> None:
        if self._closed:
            return
        error = None
        try:
            self.flush()
        except Exception as exc:
            error = exc
        self._queue.put(None)
        self._thread.join(timeout=5.0)
        self._stream.close()
        self._closed = True
        if error is not None:
            raise error
