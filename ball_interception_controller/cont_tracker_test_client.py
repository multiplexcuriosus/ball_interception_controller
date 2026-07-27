#!/usr/bin/env python3

from __future__ import annotations

import argparse
import math
import time
from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64

from fr3_husky_msgs.action import LineTrajectory


@dataclass(frozen=True)
class ClientConfig:
    targets: list[float]
    hold_sec: float
    publish_rate_hz: float
    v_max: float
    a_max: float
    j_max: float
    session_sec: float
    max_abs_s: float
    stop_publishing_after_sec: Optional[float]
    action_name: str
    target_topic: str
    ee_name: str
    profile_name: str
    discovery_wait_sec: float
    server_timeout_sec: float
    cancel_wait_sec: float


def _goal_status_name(status: Optional[int]) -> str:
    mapping = {
        GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
        GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
        GoalStatus.STATUS_EXECUTING: "EXECUTING",
        GoalStatus.STATUS_CANCELING: "CANCELING",
        GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
        GoalStatus.STATUS_CANCELED: "CANCELED",
        GoalStatus.STATUS_ABORTED: "ABORTED",
    }
    return mapping.get(status, f"STATUS_{status}")


def _set_if_present(goal: Any, field: str, value: Any) -> None:
    if hasattr(goal, field):
        setattr(goal, field, value)


def _build_goal(target_s: float, config: ClientConfig) -> LineTrajectory.Goal:
    goal = LineTrajectory.Goal()
    goal.command = int(LineTrajectory.Goal.CMD_GOTO_S)
    goal.target_s = float(target_s)
    goal.v_max = float(config.v_max)
    goal.a_max = float(config.a_max)
    goal.j_max = float(config.j_max)
    _set_if_present(goal, "ee_name", config.ee_name)
    _set_if_present(goal, "profile_name", config.profile_name)
    _set_if_present(goal, "hold_before_sec", 0.0)
    _set_if_present(goal, "hold_after_sec", 0.0)
    _set_if_present(goal, "repetitions", 1)
    return goal


def _validate_config(config: ClientConfig) -> None:
    if not config.targets:
        raise ValueError("--targets must contain at least one value")

    if config.publish_rate_hz <= 0.0 or not math.isfinite(config.publish_rate_hz):
        raise ValueError("--publish-rate must be positive and finite")

    if config.hold_sec <= 0.0 or not math.isfinite(config.hold_sec):
        raise ValueError("--hold-sec must be positive and finite")

    if config.session_sec <= 0.0 or not math.isfinite(config.session_sec):
        raise ValueError("--session-sec must be positive and finite")

    if config.server_timeout_sec <= 0.0 or not math.isfinite(config.server_timeout_sec):
        raise ValueError("--server-timeout-sec must be positive and finite")

    if config.discovery_wait_sec < 0.0 or not math.isfinite(config.discovery_wait_sec):
        raise ValueError("--discovery-wait-sec must be non-negative and finite")

    if config.cancel_wait_sec <= 0.0 or not math.isfinite(config.cancel_wait_sec):
        raise ValueError("--cancel-wait-sec must be positive and finite")

    if config.stop_publishing_after_sec is not None:
        if config.stop_publishing_after_sec <= 0.0 or not math.isfinite(config.stop_publishing_after_sec):
            raise ValueError("--stop-publishing-after must be positive and finite")

    for target_s in config.targets:
        if not math.isfinite(target_s):
            raise ValueError("--targets must not contain non-finite values")
        if abs(target_s) > config.max_abs_s:
            raise ValueError(
                f"target_s={target_s} exceeds --max-abs-s={config.max_abs_s}"
            )

    for limit_name, value in (("--v-max", config.v_max), ("--a-max", config.a_max), ("--j-max", config.j_max), ("--max-abs-s", config.max_abs_s)):
        if value < 0.0 or not math.isfinite(value):
            raise ValueError(f"{limit_name} must be non-negative and finite")


class ContTrackerTestClient(Node):
    def __init__(self, config: ClientConfig) -> None:
        super().__init__("cont_tracker_test_client")
        self._config = config
        self._qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._action_client = ActionClient(self, LineTrajectory, config.action_name)
        self._target_pub = None
        self._publish_timer = None
        self._monitor_timer = None
        self._goal_future = None
        self._goal_handle = None
        self._result_future = None
        self._cancel_future = None
        self._pending_cancel_reason = ""
        self._acceptance_pending = False
        self._goal_accepted = False
        self._goal_finished = False
        self._publish_enabled = False
        self._publishing_stopped = False
        self._cancel_sent = False
        self._exit_code = 0
        self._started_monotonic = 0.0
        self._accepted_monotonic = 0.0
        self._session_cancel_monotonic = 0.0
        self._next_target_change_monotonic = 0.0
        self._current_target_index = 0
        self._shutdown_requested = False
        self._shutdown_reason = ""
        self._expected_stale_abort = config.stop_publishing_after_sec is not None

    @property
    def exit_code(self) -> int:
        return self._exit_code

    def run(self) -> int:
        _validate_config(self._config)

        if not self._wait_for_action_server():
            self._fail(f"action server unavailable: {self._config.action_name}")
            return self._exit_code

        if not self._check_target_topic_is_free():
            self._exit_code = 1
            return self._exit_code

        self._target_pub = self.create_publisher(Float64, self._config.target_topic, self._qos)
        self._started_monotonic = time.monotonic()
        self._send_goal()
        self._monitor_timer = self.create_timer(0.05, self._on_monitor_timer)

        return self._exit_code

    def request_shutdown(self, reason: str) -> None:
        if self._shutdown_requested:
            return

        self._shutdown_requested = True
        self._shutdown_reason = reason
        self.get_logger().warn(f"shutdown requested: {reason}")

        if self._publish_timer is not None:
            self._publish_timer.cancel()

        if self._goal_accepted:
            self._request_cancel(f"shutdown: {reason}")
        else:
            self._pending_cancel_reason = f"shutdown: {reason}"

    def wait_for_shutdown_cancel(self) -> None:
        deadline = time.monotonic() + self._config.cancel_wait_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._acceptance_pending:
                rclpy.spin_once(self, timeout_sec=0.1)
                continue

            if self._cancel_future is None or self._cancel_future.done():
                break

            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait_for_action_server(self) -> bool:
        deadline = time.monotonic() + self._config.server_timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._action_client.wait_for_server(timeout_sec=0.1):
                return True
            rclpy.spin_once(self, timeout_sec=0.0)
        return False

    def _check_target_topic_is_free(self) -> bool:
        deadline = time.monotonic() + self._config.discovery_wait_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

        publishers = self.get_publishers_info_by_topic(self._config.target_topic)
        if not publishers:
            return True

        self.get_logger().error(
            f"refusing to start because {self._config.target_topic} already has publishers"
        )
        for info in publishers:
            namespace = getattr(info, "node_namespace", "")
            name = getattr(info, "node_name", "")
            self.get_logger().error(f"publisher: {namespace}/{name}".replace("//", "/"))
        return False

    def _send_goal(self) -> None:
        goal = _build_goal(self._config.targets[0], self._config)
        self.get_logger().info(
            "sending LineTrajectory goal: "
            f"command=CMD_GOTO_S target_s={goal.target_s:.6f} "
            f"v_max={goal.v_max:.3f} a_max={goal.a_max:.3f} j_max={goal.j_max:.3f} "
            f"ee_name={getattr(goal, 'ee_name', '')!r} profile_name={getattr(goal, 'profile_name', '')!r}"
        )
        self._acceptance_pending = True
        self._goal_future = self._action_client.send_goal_async(goal, feedback_callback=self._on_feedback)
        self._goal_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future: Any) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail(f"failed to send goal: {exc}")
            return

        self._acceptance_pending = False
        if goal_handle is None or not goal_handle.accepted:
            self._fail("goal rejected by /cont_tracker")
            return

        self._goal_handle = goal_handle
        self._goal_accepted = True
        self._accepted_monotonic = time.monotonic()
        self._next_target_change_monotonic = self._accepted_monotonic + self._config.hold_sec
        self._session_cancel_monotonic = self._accepted_monotonic + self._config.session_sec
        self.get_logger().info("goal accepted")

        if self._target_pub is None:
            self._target_pub = self.create_publisher(Float64, self._config.target_topic, self._qos)

        self._publish_current_target(force=True)
        self._publish_timer = self.create_timer(1.0 / self._config.publish_rate_hz, self._on_publish_timer)
        self._result_future = goal_handle.get_result_async()
        self._result_future.add_done_callback(self._on_result)

        if self._shutdown_requested:
            self._request_cancel(f"shutdown: {self._shutdown_reason}")

    def _on_feedback(self, msg: Any) -> None:
        feedback = getattr(msg, "feedback", msg)
        phase = getattr(feedback, "phase", None)
        progress = getattr(feedback, "progress", None)
        s_des = getattr(feedback, "s_des", None)
        sdot_des = getattr(feedback, "sdot_des", None)
        tracking_error_pos = getattr(feedback, "tracking_error_pos", None)
        tracking_error_z = getattr(feedback, "tracking_error_z", None)
        self.get_logger().info(
            "feedback: "
            f"phase={phase} progress={progress} s_des={s_des} sdot_des={sdot_des} "
            f"tracking_error_pos={tracking_error_pos} tracking_error_z={tracking_error_z}"
        )

    def _current_target(self) -> float:
        return float(self._config.targets[self._current_target_index])

    def _publish_current_target(self, force: bool = False) -> None:
        del force
        if self._target_pub is None:
            return
        msg = Float64()
        msg.data = self._current_target()
        self._target_pub.publish(msg)

    def _on_publish_timer(self) -> None:
        if not self._goal_accepted or self._goal_finished:
            return

        now = time.monotonic()
        elapsed = now - self._accepted_monotonic

        if self._config.stop_publishing_after_sec is not None and not self._publishing_stopped:
            if elapsed >= self._config.stop_publishing_after_sec:
                self._publishing_stopped = True
                if self._publish_timer is not None:
                    self._publish_timer.cancel()
                self.get_logger().info(
                    f"stopped publishing targets after {self._config.stop_publishing_after_sec:.3f}s"
                )
                return

        if now >= self._next_target_change_monotonic and self._current_target_index < len(self._config.targets) - 1:
            self._current_target_index += 1
            self._next_target_change_monotonic += self._config.hold_sec
            self.get_logger().info(f"target changed to {self._current_target():.6f}")

        self._publish_current_target()

    def _on_monitor_timer(self) -> None:
        if self._goal_finished:
            return

        now = time.monotonic()
        if self._goal_accepted and now >= self._session_cancel_monotonic and not self._cancel_sent:
            self.get_logger().info("session limit reached; requesting cancel")
            self._request_cancel("session_sec elapsed")
            return

        if self._shutdown_requested and self._goal_accepted and not self._cancel_sent:
            self._request_cancel(f"shutdown: {self._shutdown_reason}")

    def _request_cancel(self, reason: str) -> None:
        if self._cancel_sent:
            return

        self._pending_cancel_reason = reason
        if not self._goal_accepted or self._goal_handle is None:
            return

        self._cancel_sent = True
        self.get_logger().info(f"requesting cancel: {reason}")
        try:
            self._cancel_future = self._goal_handle.cancel_goal_async()
        except Exception as exc:
            self._fail(f"failed to request cancel: {exc}")
            return
        self._cancel_future.add_done_callback(self._on_cancel_response)

    def _on_cancel_response(self, future: Any) -> None:
        try:
            result = future.result()
        except Exception as exc:
            self._fail(f"cancel response failed: {exc}")
            return

        self.get_logger().info(f"cancel response received: {result}")

    def _on_result(self, future: Any) -> None:
        try:
            wrapped = future.result()
        except Exception as exc:
            self._fail(f"result future failed: {exc}")
            return

        self._handle_result(wrapped)

    def _handle_result(self, wrapped: Any) -> None:
        status = getattr(wrapped, "status", None)
        result = getattr(wrapped, "result", None)
        status_name = _goal_status_name(status)
        message = getattr(result, "message", "") if result is not None else ""
        self.get_logger().info(f"final result: status={status_name} message={message}")

        self._goal_finished = True
        if self._publish_timer is not None:
            self._publish_timer.cancel()
        if self._monitor_timer is not None:
            self._monitor_timer.cancel()

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._exit_code = 0
        elif status == GoalStatus.STATUS_CANCELED:
            self._exit_code = 0 if self._cancel_sent else 1
        elif status == GoalStatus.STATUS_ABORTED:
            self._exit_code = 0 if self._expected_stale_abort else 1
        else:
            self._exit_code = 1

        if self._config.stop_publishing_after_sec is not None:
            self.get_logger().info("stale-target test complete; stopping after final result")

    def _fail(self, message: str) -> None:
        self.get_logger().error(message)
        self._exit_code = 1
        self._goal_finished = True
        if self._publish_timer is not None:
            self._publish_timer.cancel()
        if self._monitor_timer is not None:
            self._monitor_timer.cancel()


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Standalone cont_tracker diagnostic client")
    parser.add_argument("--targets", nargs="+", type=float, default=[0.0])
    parser.add_argument("--hold-sec", type=float, default=1.0)
    parser.add_argument("--publish-rate", type=float, default=20.0)
    parser.add_argument("--v-max", type=float, default=0.03)
    parser.add_argument("--a-max", type=float, default=0.10)
    parser.add_argument("--j-max", type=float, default=30.0)
    parser.add_argument("--session-sec", type=float, default=2.0)
    parser.add_argument("--max-abs-s", type=float, default=0.02)
    parser.add_argument("--stop-publishing-after", type=float, default=None)
    parser.add_argument("--action-name", type=str, default="/cont_tracker")
    parser.add_argument("--target-topic", type=str, default="/cont_tracker/target_s")
    parser.add_argument("--ee-name", type=str, default="")
    parser.add_argument("--profile-name", type=str, default="interception")
    parser.add_argument("--discovery-wait-sec", type=float, default=0.5)
    parser.add_argument("--server-timeout-sec", type=float, default=5.0)
    parser.add_argument("--cancel-wait-sec", type=float, default=2.0)
    return parser


def main(args: Optional[list[str]] = None) -> None:
    parser = _build_arg_parser()
    parsed_args, ros_args = parser.parse_known_args(args)
    config = ClientConfig(
        targets=[float(value) for value in parsed_args.targets],
        hold_sec=float(parsed_args.hold_sec),
        publish_rate_hz=float(parsed_args.publish_rate),
        v_max=float(parsed_args.v_max),
        a_max=float(parsed_args.a_max),
        j_max=float(parsed_args.j_max),
        session_sec=float(parsed_args.session_sec),
        max_abs_s=float(parsed_args.max_abs_s),
        stop_publishing_after_sec=(
            None if parsed_args.stop_publishing_after is None else float(parsed_args.stop_publishing_after)
        ),
        action_name=str(parsed_args.action_name),
        target_topic=str(parsed_args.target_topic),
        ee_name=str(parsed_args.ee_name),
        profile_name=str(parsed_args.profile_name),
        discovery_wait_sec=float(parsed_args.discovery_wait_sec),
        server_timeout_sec=float(parsed_args.server_timeout_sec),
        cancel_wait_sec=float(parsed_args.cancel_wait_sec),
    )

    try:
        _validate_config(config)
    except ValueError as exc:
        print(f"configuration error: {exc}")
        raise SystemExit(2) from exc

    rclpy.init(args=ros_args)
    node = ContTrackerTestClient(config)
    try:
        node.run()
        while rclpy.ok() and not node._goal_finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.request_shutdown("Ctrl+C")
        node.wait_for_shutdown_cancel()
    finally:
        try:
            node.wait_for_shutdown_cancel()
        finally:
            node.destroy_node()
            rclpy.shutdown()

    raise SystemExit(node.exit_code)


if __name__ == "__main__":
    main()