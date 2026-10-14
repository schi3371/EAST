"""Bounded background persistence and diagnostic helpers for the EAST GUI."""

import queue
import csv
import os
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

    def write_and_wait(self, motion_state, snapshot, clean_shutdown=False, extra=None,
                       timeout_s=3.0):
        """Serialize a required checkpoint behind queued writes before continuing motion."""
        if self.error is not None or self.stop_event.is_set() or not self.thread.is_alive():
            raise RuntimeError("checkpoint writer is unavailable")
        done = threading.Event()
        outcome = {}
        deadline = time.monotonic() + timeout_s
        try:
            self.pending.put(("required", motion_state, snapshot, clean_shutdown,
                              extra, done, outcome), timeout=min(1.0, max(0.0, deadline - time.monotonic())))
        except queue.Full as exc:
            raise RuntimeError("required checkpoint could not be queued") from exc
        if self.error is not None or not self.thread.is_alive():
            raise RuntimeError(f"checkpoint writer failed: {self.error}")
        while not done.wait(min(0.1, max(0.0, deadline - time.monotonic()))):
            if self.error is not None or not self.thread.is_alive():
                raise RuntimeError(f"checkpoint writer stopped before required write: {self.error}")
            if time.monotonic() >= deadline:
                raise TimeoutError("required checkpoint durability deadline exceeded; worker may still be writing")
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


class CSVRecorder:
    """Ordered bounded row recorder. Acquisition only enqueues; writer owns fsync."""

    def __init__(self, path, flush_rows, on_error, max_pending_rows=2000):
        if flush_rows <= 0 or max_pending_rows <= 0:
            raise ValueError("Recorder limits must be positive")
        self.path = path
        self.flush_rows = flush_rows
        self.on_error = on_error
        self.pending = queue.Queue(maxsize=max_pending_rows)
        self.close_requested = threading.Event()
        self.done = threading.Event()
        self.error = None
        self.written_rows = 0
        self.last_disk_duration_s = 0.0
        self.active_batch = []
        self.state_lock = threading.RLock()
        self.wake = threading.Event()
        self.thread = threading.Thread(target=self._run, name="east-csv-recorder", daemon=True)

    def start(self):
        self.thread.start()

    def append(self, row):
        with self.state_lock:
            if self.error is not None:
                raise RuntimeError(f"CSV recording failed: {self.error}")
            if self.close_requested.is_set():
                raise RuntimeError("CSV recording is closing")
            try:
                self.pending.put_nowait(tuple(row))
            except queue.Full as exc:
                raise RuntimeError("CSV recorder backlog reached its safety limit") from exc
            self.wake.set()

    def finish(self, timeout_s=3.0):
        with self.state_lock:
            self.close_requested.set()
            self.wake.set()
        if not self.done.wait(timeout_s):
            raise TimeoutError("CSV recorder did not finish; buffered rows and writer remain owned")
        if self.error is not None:
            raise RuntimeError(f"CSV recording failed: {self.error}")
        return self.written_rows

    def unsaved_rows(self):
        # This snapshot may include an in-flight batch whose durability is unknown.
        # Recovery copies are separate files, never appended back to the main CSV.
        with self.state_lock, self.pending.mutex:
            return list(self.active_batch) + list(self.pending.queue)

    def _run(self):
        try:
            while True:
                with self.state_lock:
                    try:
                        row = self.pending.get_nowait()
                    except queue.Empty:
                        row = None
                    if row is not None:
                        self.active_batch.append(row)
                        self.pending.task_done()
                    empty = self.pending.empty()
                    closing = self.close_requested.is_set()
                    flush = bool(self.active_batch) and (len(self.active_batch) >= self.flush_rows or closing and empty)
                    self.wake.clear()
                if flush:
                    self._flush()
                if closing and empty:
                    break
                if row is None:
                    self.wake.wait(.05)
        except Exception as exc:
            with self.state_lock:
                self.error = exc
            self.on_error(exc)
        finally:
            self.done.set()

    def _flush(self):
        started = time.monotonic()
        with self.state_lock:
            batch = list(self.active_batch)
        with open(self.path, mode="a", newline="", encoding="utf-8") as handle:
            csv.writer(handle).writerows(batch)
            handle.flush()
            os.fsync(handle.fileno())
        with self.state_lock:
            self.written_rows += len(batch)
            del self.active_batch[:len(batch)]
            self.last_disk_duration_s = time.monotonic() - started
