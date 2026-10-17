"""Bounded lifecycle workers; ROS callbacks await completion without blocking."""

import threading
from collections.abc import Callable
from dataclasses import dataclass, field

from rclpy.task import Future


class TaskCancelled(Exception):
    """A stop or node shutdown cancelled a lifecycle operation."""


@dataclass
class _Task:
    kind: str = "lifecycle"
    cancel: threading.Event = field(default_factory=threading.Event)
    done: threading.Event = field(default_factory=threading.Event)
    future: Future = field(default_factory=Future)
    thread: threading.Thread | None = None


class LifecycleTasks:
    """One normal operation and one interrupting stop, with no pending queue."""

    JOIN_TIMEOUT = 3.0

    def __init__(self):
        self._lock = threading.Lock()
        self._active: _Task | None = None
        self._stop: _Task | None = None
        self._closing = False

    @property
    def busy(self) -> bool:
        with self._lock:
            return self._closing or self._active is not None or self._stop is not None

    @property
    def configuring_speed(self) -> bool:
        with self._lock:
            return (
                not self._closing
                and self._stop is None
                and self._active is not None
                and self._active.kind == "speed"
            )

    def submit(
        self,
        operation: Callable[[threading.Event], str],
        *,
        stop=False,
        kind="lifecycle",
    ) -> Future:
        with self._lock:
            if self._closing or self._stop is not None or (not stop and self._active):
                future = Future()
                future.set_result(
                    (False, "Lifecycle operation busy or node shutting down")
                )
                return future
            active = self._active
            if stop and active is not None:
                active.cancel.set()
            task = _Task(kind=kind)
            if stop:
                self._stop = task
            else:
                self._active = task
            task.thread = threading.Thread(
                target=self._run,
                args=(task, operation, active if stop else None),
                name="ArmLifecycle.stop" if stop else "ArmLifecycle.operation",
                daemon=True,
            )
            try:
                task.thread.start()
            except BaseException:
                if stop:
                    self._stop = None
                else:
                    self._active = None
                raise
            return task.future

    def _run(self, task, operation, interrupted):
        try:
            message = operation(task.cancel)
            if interrupted is not None and not interrupted.done.wait(self.JOIN_TIMEOUT):
                raise RuntimeError("Cancelled lifecycle worker did not finish")
            if task.cancel.is_set():
                raise TaskCancelled("Lifecycle operation cancelled")
            result = (True, message)
        except BaseException as error:
            result = (False, str(error) or type(error).__name__)
        finally:
            with self._lock:
                if self._active is task:
                    self._active = None
                if self._stop is task:
                    self._stop = None
            task.done.set()
        task.future.set_result(result)

    def cancel_for_shutdown(self):
        with self._lock:
            self._closing = True
            tasks = [task for task in (self._active, self._stop) if task is not None]
            for task in tasks:
                task.cancel.set()
        return tasks

    def join(self, tasks):
        for task in tasks:
            thread = task.thread
            if thread is not None and thread is not threading.current_thread():
                thread.join(self.JOIN_TIMEOUT)
                if thread.is_alive():
                    raise RuntimeError(f"Lifecycle worker did not stop: {thread.name}")
