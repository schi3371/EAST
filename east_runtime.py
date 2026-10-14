"""Bounded background persistence and diagnostic helpers for the EAST GUI."""

import queue
import sys
import threading
import time
import traceback
from datetime import datetime


class CheckpointWriter:
    def __init__(self, manager, on_error):
        self.manager = manager
        self.on_error = on_error
        self.pending = queue.Queue(maxsize=8)
        self.stop_event = threading.Event()
        self.error = None
        self.last_disk_duration_s = 0.0
        self.thread = threading.Thread(target=self._run, name="east-checkpoint-writer", daemon=True)

    def start(self):
        self.thread.start()

    def submit(self, motion_state, snapshot, extra):
        if self.error is not None:
            raise RuntimeError(f"checkpoint persistence failed: {self.error}")
        if self.stop_event.is_set():
            raise RuntimeError("checkpoint writer is stopped")
        try:
            self.pending.put_nowait(("checkpoint", motion_state, snapshot, extra))
        except queue.Full as exc:
            raise RuntimeError(
                f"checkpoint persistence backlog; last disk write {self.last_disk_duration_s:.3f} s"
            ) from exc

    def submit_event(self, name, details):
        if self.error is not None or self.stop_event.is_set():
            raise RuntimeError("checkpoint writer is unavailable")
        try:
            self.pending.put_nowait(("event", name, details))
        except queue.Full as exc:
            raise RuntimeError("checkpoint audit event backlog") from exc

    def write_and_wait(self, motion_state, snapshot, clean_shutdown=False, extra=None):
        """Serialize a required checkpoint behind queued writes before continuing motion."""
        if self.error is not None or self.stop_event.is_set() or not self.thread.is_alive():
            raise RuntimeError("checkpoint writer is unavailable")
        done = threading.Event()
        outcome = {}
        try:
            self.pending.put(("required", motion_state, snapshot, clean_shutdown,
                              extra, done, outcome), timeout=1.0)
        except queue.Full as exc:
            raise RuntimeError("required checkpoint could not be queued") from exc
        if self.error is not None or not self.thread.is_alive():
            raise RuntimeError(f"checkpoint writer failed: {self.error}")
        while not done.wait(0.1):
            if self.error is not None or not self.thread.is_alive():
                raise RuntimeError(f"checkpoint writer stopped before required write: {self.error}")
        if "error" in outcome:
            raise RuntimeError(f"required checkpoint failed: {outcome['error']}")

    def stop(self, timeout_s=0.0):
        self.stop_event.set()
        if timeout_s and self.thread.is_alive() and self.thread is not threading.current_thread():
            self.thread.join(timeout_s)
        return not self.thread.is_alive() and self.error is None and self.pending.empty()

    def _run(self):
        while not self.stop_event.is_set() or not self.pending.empty():
            try:
                item = self.pending.get(timeout=0.05)
            except queue.Empty:
                continue
            started = time.monotonic()
            try:
                if item[0] == "checkpoint":
                    _, state, snapshot, extra = item
                    self.manager.write_checkpoint(state, snapshot, clean_shutdown=False, extra=extra)
                elif item[0] == "required":
                    _, state, snapshot, clean, extra, _, _ = item
                    self.manager.write_checkpoint(state, snapshot, clean_shutdown=clean, extra=extra)
                else:
                    _, name, details = item
                    self.manager.store.append_event(name, details)
            except Exception as exc:
                self.error = exc
                if item[0] == "required":
                    item[6]["error"] = exc
                while True:
                    try:
                        waiting = self.pending.get_nowait()
                    except queue.Empty:
                        break
                    if waiting[0] == "required":
                        waiting[6]["error"] = exc
                        waiting[5].set()
                    self.pending.task_done()
                self.on_error(exc)
                return
            finally:
                self.last_disk_duration_s = time.monotonic() - started
                if item[0] == "required":
                    item[5].set()
                self.pending.task_done()


def write_worker_diagnostics(path, threads, details):
    """Write a best-effort stack snapshot; callers must run this off the GUI thread."""
    frames = sys._current_frames()
    with open(path, "a", encoding="utf-8") as handle:
        handle.write(f"\n{datetime.now().astimezone().isoformat()} {details}\n")
        for thread in threads:
            if thread is not None and thread.is_alive():
                handle.write(f"THREAD {thread.name} ({thread.ident})\n")
                frame = frames.get(thread.ident)
                if frame is not None:
                    handle.writelines(traceback.format_stack(frame))
        handle.flush()
