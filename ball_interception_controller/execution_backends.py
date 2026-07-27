from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Callable, Optional, Protocol, Tuple

from rclpy.action import ActionClient
from std_msgs.msg import Float64


class BackendCallbacks(Protocol):
    def on_backend_goal_accepted(self, backend_name: str, generation: int, goal_handle: Any) -> None: ...
    def on_backend_goal_rejected(self, backend_name: str, generation: int, reason: str) -> None: ...
    def on_backend_result(self, backend_name: str, generation: int, status: Any, result: Any) -> None: ...
    def on_backend_cancel_complete(self, backend_name: str, generation: int, result: Any) -> None: ...
    def on_backend_feedback(self, backend_name: str, generation: int, feedback: Any) -> None: ...
    def on_backend_error(self, backend_name: str, generation: int, error: Exception) -> None: ...


@dataclass
class GoalRequest:
    goal: Any
    target_s: float
    generation: int


class ExecutionBackend:
    backend_name = "base"

    def start(self, request: GoalRequest) -> None:
        raise NotImplementedError

    def update_target(self, target_s: float, generation: int) -> None:
        raise NotImplementedError

    def cancel(self, reason: str, generation: int) -> None:
        raise NotImplementedError

    def is_active(self) -> bool:
        raise NotImplementedError

    def shutdown(self) -> None:
        raise NotImplementedError


class _ActionBackend(ExecutionBackend):
    def __init__(
        self,
        node: Any,
        *,
        action_name: str,
        action_type: Any,
        callbacks: BackendCallbacks,
    ) -> None:
        self._node = node
        self._callbacks = callbacks
        self._action_client = ActionClient(node, action_type, action_name)
        self._goal_handle: Optional[Any] = None
        self._goal_request_in_flight = False
        self._goal_request_generation: Optional[int] = None
        self._goal_request: Optional[GoalRequest] = None
        self._cancel_requested = False
        self._cancel_reason = ""

    @property
    def action_client(self) -> ActionClient:
        return self._action_client

    def _generation_matches(self, generation: int) -> bool:
        return self._goal_request_generation == generation

    def _safe_callback(self, name: str, *args: Any) -> None:
        method = getattr(self._callbacks, name, None)
        if callable(method):
            method(self.backend_name, *args)

    def _send_goal_async(self, goal: Any, feedback_callback: Optional[Callable[[Any], None]] = None):
        try:
            if feedback_callback is None:
                return self._action_client.send_goal_async(goal)
            return self._action_client.send_goal_async(goal, feedback_callback=feedback_callback)
        except TypeError:
            return self._action_client.send_goal_async(goal)

    def _cancel_goal_async(self, goal_handle: Any):
        cancel_method = getattr(goal_handle, "cancel_goal_async", None)
        if callable(cancel_method):
            return cancel_method()
        return None

    def is_active(self) -> bool:
        return self._goal_handle is not None or self._goal_request_in_flight

    def shutdown(self) -> None:
        if self._goal_handle is not None:
            try:
                cancel_method = getattr(self._goal_handle, "cancel_goal_async", None)
                if callable(cancel_method):
                    cancel_method()
            except Exception:
                pass
        self._goal_handle = None
        self._goal_request_in_flight = False
        self._goal_request_generation = None
        self._goal_request = None
        self._cancel_requested = False
        self._cancel_reason = ""


class GotoSBackend(_ActionBackend):
    backend_name = "goto_s"

    def start(self, request: GoalRequest) -> None:
        if self.is_active():
            return
        self._goal_request_generation = request.generation
        self._goal_request = request
        self._goal_request_in_flight = True
        future = self._send_goal_async(request.goal)
        future.add_done_callback(lambda fut: self._on_goal_response(fut, request.generation))

    def update_target(self, target_s: float, generation: int) -> None:
        del target_s, generation

    def cancel(self, reason: str, generation: int) -> None:
        if not self._generation_matches(generation):
            return
        self._cancel_requested = True
        self._cancel_reason = reason
        if self._goal_handle is not None:
            future = self._cancel_goal_async(self._goal_handle)
            future.add_done_callback(lambda fut: self._on_cancel_done(fut, generation))

    def _on_goal_response(self, future: Any, generation: int) -> None:
        if not self._generation_matches(generation):
            self._goal_request_in_flight = False
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._goal_request_in_flight = False
            self._safe_callback("on_backend_error", generation, exc)
            return

        self._goal_request_in_flight = False
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            self._safe_callback("on_backend_goal_rejected", generation, "trajectory executor rejected CMD_GOTO_S goal")
            return

        self._goal_handle = goal_handle
        self._safe_callback("on_backend_goal_accepted", generation, goal_handle)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_result(fut, generation))

        if self._cancel_requested:
            future = self._cancel_goal_async(goal_handle)
            future.add_done_callback(lambda fut: self._on_cancel_done(fut, generation))

    def _on_result(self, future: Any, generation: int) -> None:
        if not self._generation_matches(generation):
            return
        try:
            wrapped = future.result()
        except Exception as exc:
            self._safe_callback("on_backend_error", generation, exc)
            return
        self._goal_handle = None
        self._safe_callback("on_backend_result", generation, getattr(wrapped, "status", None), getattr(wrapped, "result", None))

    def _on_cancel_done(self, future: Any, generation: int) -> None:
        if not self._generation_matches(generation):
            return
        try:
            wrapped = future.result()
        except Exception as exc:
            self._safe_callback("on_backend_error", generation, exc)
            return
        self._goal_handle = None
        self._safe_callback("on_backend_cancel_complete", generation, wrapped)


class ContTrackerBackend(_ActionBackend):
    backend_name = "cont_tracker"

    def __init__(
        self,
        node: Any,
        *,
        action_name: str,
        action_type: Any,
        target_topic: str,
        callbacks: BackendCallbacks,
    ) -> None:
        super().__init__(node, action_name=action_name, action_type=action_type, callbacks=callbacks)
        self._target_pub = node.create_publisher(Float64, target_topic, 10)
        self._latest_pending_target_s: Optional[float] = None
        self._last_published_target_s: Optional[float] = None
        self._goal_started_target_s: Optional[float] = None
        self._goal_accepted = False

    @property
    def target_publisher(self) -> Any:
        return self._target_pub

    def start(self, request: GoalRequest) -> None:
        if self.is_active():
            return
        self._goal_request_generation = request.generation
        self._goal_request = request
        self._goal_request_in_flight = True
        self._goal_started_target_s = float(request.target_s)
        future = self._send_goal_async(request.goal, feedback_callback=self._on_feedback)
        future.add_done_callback(lambda fut: self._on_goal_response(fut, request.generation))

    def update_target(self, target_s: float, generation: int) -> None:
        if not self._generation_matches(generation):
            return
        if self._goal_request_in_flight and not self._goal_accepted:
            self._latest_pending_target_s = float(target_s)
            return
        self._publish_target(target_s)

    def cancel(self, reason: str, generation: int) -> None:
        if not self._generation_matches(generation):
            return
        self._cancel_requested = True
        self._cancel_reason = reason
        if self._goal_handle is not None:
            future = self._cancel_goal_async(self._goal_handle)
            future.add_done_callback(lambda fut: self._on_cancel_done(fut, generation))

    def _publish_target(self, target_s: float) -> None:
        if self._last_published_target_s is not None and math.isclose(self._last_published_target_s, float(target_s), rel_tol=0.0, abs_tol=1e-12):
            return
        msg = Float64()
        msg.data = float(target_s)
        self._target_pub.publish(msg)
        self._last_published_target_s = float(target_s)

    def _on_goal_response(self, future: Any, generation: int) -> None:
        if not self._generation_matches(generation):
            self._goal_request_in_flight = False
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._goal_request_in_flight = False
            self._safe_callback("on_backend_error", generation, exc)
            return

        self._goal_request_in_flight = False
        if goal_handle is None or not getattr(goal_handle, "accepted", False):
            self._safe_callback("on_backend_goal_rejected", generation, "cont_tracker rejected goal")
            return

        self._goal_handle = goal_handle
        self._goal_accepted = True
        self._safe_callback("on_backend_goal_accepted", generation, goal_handle)

        if self._latest_pending_target_s is not None:
            if self._goal_started_target_s is None or not math.isclose(self._latest_pending_target_s, self._goal_started_target_s, rel_tol=0.0, abs_tol=1e-12):
                self._publish_target(self._latest_pending_target_s)
            self._latest_pending_target_s = None

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_result(fut, generation))

        if self._cancel_requested:
            future = self._cancel_goal_async(goal_handle)
            future.add_done_callback(lambda fut: self._on_cancel_done(fut, generation))

    def _on_feedback(self, feedback_msg: Any) -> None:
        if self._goal_request_generation is None:
            return
        self._safe_callback("on_backend_feedback", self._goal_request_generation, feedback_msg)

    def _on_result(self, future: Any, generation: int) -> None:
        if not self._generation_matches(generation):
            return
        try:
            wrapped = future.result()
        except Exception as exc:
            self._safe_callback("on_backend_error", generation, exc)
            return
        self._goal_handle = None
        self._goal_accepted = False
        self._safe_callback("on_backend_result", generation, getattr(wrapped, "status", None), getattr(wrapped, "result", None))

    def _on_cancel_done(self, future: Any, generation: int) -> None:
        if not self._generation_matches(generation):
            return
        try:
            wrapped = future.result()
        except Exception as exc:
            self._safe_callback("on_backend_error", generation, exc)
            return
        self._goal_handle = None
        self._goal_accepted = False
        self._safe_callback("on_backend_cancel_complete", generation, wrapped)
