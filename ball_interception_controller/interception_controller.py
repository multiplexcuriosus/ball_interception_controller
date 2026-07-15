#!/usr/bin/env python3

from __future__ import annotations

import importlib
from enum import Enum
from typing import Any, Optional, Type

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from fr3_husky_msgs.srv import ProjectPointToLine


class InterceptionState(str, Enum):
    FROZEN = "FROZEN"
    RESETTING = "RESETTING"
    ARMED_WAITING = "ARMED_WAITING"
    PROJECTING = "PROJECTING"
    SENDING_ACTION = "SENDING_ACTION"
    EXECUTING = "EXECUTING"


def _clean_frame(frame: str) -> str:
    return str(frame).strip().strip("/")


def _load_action_type(type_string: str) -> Type[Any]:
    """
    Load action type from strings like:
      - fr3_husky_msgs.action.TrajectoryExecutor
      - fr3_husky_msgs.action.MoveToS
    """
    type_string = str(type_string).strip()
    if not type_string:
        raise ValueError("trajectory_action_type parameter is empty")

    module_name, class_name = type_string.rsplit(".", 1)
    module = importlib.import_module(module_name)
    return getattr(module, class_name)


class InterceptionController(Node):
    """
    One-shot ball interception coordinator.

    Flow:
      1. /interception_controller/arm
      2. call /ball_trajectory_estimator/reset
      3. wait for fresh /scene/middle_line_intersection_pose_robot_base
      4. call /trajectory_executor/project_point_to_line
      5. send existing trajectory executor CMD_GOTO_S action
      6. freeze until armed again
    """

    def __init__(self) -> None:
        super().__init__("interception_controller")

        # Topics/services/actions.
        self.declare_parameter(
            "intercept_pose_topic",
            "/scene/middle_line_intersection_pose_robot_base",
        )
        self.declare_parameter(
            "trajectory_reset_service",
            "/ball_trajectory_estimator/reset",
        )
        self.declare_parameter(
            "project_point_service",
            "/trajectory_executor/project_point_to_line",
        )
        self.declare_parameter(
            "trajectory_action_name",
            "/trajectory_executor",
        )

        # This may need adjustment to your actual .action type name.
        self.declare_parameter(
            "trajectory_action_type",
            "fr3_husky_msgs.action.LineTrajectory",
        )

        # Goal fields for trajectory executor.
        self.declare_parameter("cmd_goto_s_constant_name", "CMD_GOTO_S")
        self.declare_parameter("cmd_goto_s_fallback_value", -1)
        self.declare_parameter("ee_name", "")
        self.declare_parameter("profile_name", "interception")
        self.declare_parameter("v_max", 1.0)
        self.declare_parameter("a_max", 2.0)
        self.declare_parameter("j_max", 0.0)
        self.declare_parameter("repetitions", 1)

        # Conservative filters.
        self.declare_parameter("expected_frame", "base")
        self.declare_parameter("max_intercept_pose_age_sec", 0.25)
        self.declare_parameter("max_cross_track_error_m", 0.03)
        self.declare_parameter("allow_out_of_bounds_projection", False)
        self.declare_parameter("max_wait_after_arm_sec", 5.0)

        # Important: scene_localizer may still have the previous valid trajectory
        # cached for trajectory_timeout_sec. Ignore intercept poses briefly after reset.
        self.declare_parameter("post_reset_ignore_sec", 0.60)

        self.declare_parameter("require_reset_service", True)
        self.declare_parameter("status_publish_rate_hz", 2.0)
        self.declare_parameter("debug_log", False)

        action_type_string = str(self.get_parameter("trajectory_action_type").value)
        try:
            self._action_type = _load_action_type(action_type_string)
        except Exception as exc:
            raise RuntimeError(
                f"Failed to load trajectory_action_type='{action_type_string}'. "
                "Fix the parameter to match your trajectory executor action type."
            ) from exc

        self._state = InterceptionState.FROZEN
        self._generation = 0
        self._last_error = ""
        self._last_info = "initialized"
        self._arm_time_sec: Optional[float] = None
        self._accept_after_time_sec: Optional[float] = None
        self._waiting_start_time_sec: Optional[float] = None
        self._active_goal_handle: Optional[Any] = None
        self._active_command_v_max: Optional[float] = None
        self._active_command_a_max: Optional[float] = None

        self._reset_client = self.create_client(
            Trigger,
            str(self.get_parameter("trajectory_reset_service").value),
        )
        self._project_client = self.create_client(
            ProjectPointToLine,
            str(self.get_parameter("project_point_service").value),
        )
        self._action_client = ActionClient(
            self,
            self._action_type,
            str(self.get_parameter("trajectory_action_name").value),
        )

        self._arm_srv = self.create_service(
            Trigger,
            "/interception_controller/arm",
            self._handle_arm,
        )
        self._disarm_srv = self.create_service(
            Trigger,
            "/interception_controller/disarm",
            self._handle_disarm,
        )

        self._intercept_sub = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("intercept_pose_topic").value),
            self._handle_intercept_pose,
            10,
        )

        self._status_pub = self.create_publisher(
            String,
            "/interception_controller/status",
            10,
        )

        status_hz = max(0.2, float(self.get_parameter("status_publish_rate_hz").value))
        self._status_timer = self.create_timer(1.0 / status_hz, self._publish_status)
        self._timeout_timer = self.create_timer(0.05, self._check_wait_timeout)

        self.get_logger().info(
            "interception_controller started: "
            f"intercept_pose_topic={self.get_parameter('intercept_pose_topic').value}, "
            f"reset_service={self.get_parameter('trajectory_reset_service').value}, "
            f"project_service={self.get_parameter('project_point_service').value}, "
            f"action={self.get_parameter('trajectory_action_name').value}, "
            f"action_type={action_type_string}"
        )

    def _debug(self, msg: str) -> None:
        if bool(self.get_parameter("debug_log").value):
            self.get_logger().info(msg)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _stamp_to_sec(msg: PoseStamped) -> float:
        stamp = msg.header.stamp
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _set_state(self, state: InterceptionState, info: str = "") -> None:
        if self._state != state:
            self.get_logger().info(f"state {self._state.value} -> {state.value}: {info}")
        self._state = state
        if info:
            self._last_info = info

    def _freeze(self, reason: str, error: bool = False) -> None:
        self._active_goal_handle = None
        self._waiting_start_time_sec = None
        self._accept_after_time_sec = None

        if error:
            self._last_error = reason
            self.get_logger().warn(f"freezing after error: {reason}")
        else:
            self.get_logger().info(f"freezing: {reason}")

        self._set_state(InterceptionState.FROZEN, reason)
        self._active_command_v_max = None
        self._active_command_a_max = None

    def _handle_arm(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request

        if self._state in {
            InterceptionState.RESETTING,
            InterceptionState.PROJECTING,
            InterceptionState.SENDING_ACTION,
            InterceptionState.EXECUTING,
        }:
            response.success = False
            response.message = f"Cannot arm while state={self._state.value}"
            return response

        self._generation += 1
        generation = self._generation
        self._last_error = ""
        self._arm_time_sec = self._now_sec()
        self._active_goal_handle = None

        require_reset = bool(self.get_parameter("require_reset_service").value)
        reset_service_name = str(self.get_parameter("trajectory_reset_service").value)

        if require_reset and not self._reset_client.wait_for_service(timeout_sec=0.2):
            self._freeze(f"reset service unavailable: {reset_service_name}", error=True)
            response.success = False
            response.message = self._last_error
            return response

        if self._reset_client.service_is_ready():
            self._set_state(InterceptionState.RESETTING, "calling trajectory estimator reset")
            future = self._reset_client.call_async(Trigger.Request())
            future.add_done_callback(lambda fut: self._on_reset_done(fut, generation))
            response.success = True
            response.message = "Interception arming started; trajectory estimator reset requested."
            return response

        # Only reachable when require_reset_service is false.
        self._enter_waiting_after_reset(generation, "reset service skipped/unavailable")
        response.success = True
        response.message = "Interception armed without reset service."
        return response

    def _on_reset_done(self, future: Any, generation: int) -> None:
        if generation != self._generation:
            return

        try:
            result = future.result()
        except Exception as exc:
            self._freeze(f"trajectory reset call failed: {exc}", error=True)
            return

        if result is not None and hasattr(result, "success") and not result.success:
            self._freeze(f"trajectory reset returned failure: {result.message}", error=True)
            return

        self._enter_waiting_after_reset(generation, "trajectory reset complete")

    def _enter_waiting_after_reset(self, generation: int, reason: str) -> None:
        if generation != self._generation:
            return

        now_sec = self._now_sec()
        post_reset_ignore_sec = max(0.0, float(self.get_parameter("post_reset_ignore_sec").value))

        self._waiting_start_time_sec = now_sec
        self._accept_after_time_sec = now_sec + post_reset_ignore_sec
        self._set_state(
            InterceptionState.ARMED_WAITING,
            f"{reason}; accepting intercept poses after {post_reset_ignore_sec:.3f}s",
        )

    def _handle_disarm(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request

        if self._state == InterceptionState.EXECUTING:
            response.success = False
            response.message = "Already executing; not canceling automatically. Use trajectory executor cancel if needed."
            return response

        self._generation += 1
        self._freeze("disarmed by service call", error=False)
        response.success = True
        response.message = "Interception controller disarmed/frozen."
        return response

    def _handle_intercept_pose(self, msg: PoseStamped) -> None:
        self.get_logger().info("In _handle_intercept_pose")
        if self._state != InterceptionState.ARMED_WAITING:
            return

        now_sec = self._now_sec()

        if self._accept_after_time_sec is not None and now_sec < self._accept_after_time_sec:
            self._debug("ignoring intercept pose during post-reset ignore window")
            return

        expected_frame = _clean_frame(str(self.get_parameter("expected_frame").value))
        msg_frame = _clean_frame(msg.header.frame_id)
        if expected_frame and msg_frame != expected_frame:
            self.get_logger().warn(
                f"ignoring intercept pose with frame_id='{msg_frame}', expected='{expected_frame}'"
            )
            return

        stamp_sec = self._stamp_to_sec(msg)
        if stamp_sec > 0.0:
            max_age = max(0.0, float(self.get_parameter("max_intercept_pose_age_sec").value))
            age = now_sec - stamp_sec
            if age > max_age:
                self._debug(f"ignoring stale intercept pose age={age:.3f}s max={max_age:.3f}s")
                return

        p = msg.pose.position
        if not all(map(lambda x: x == x and abs(x) != float("inf"), [p.x, p.y, p.z])):
            self.get_logger().warn("ignoring non-finite intercept pose")
            return

        generation = self._generation
        self._set_state(InterceptionState.PROJECTING, "projecting intercept pose to executor line")

        if not self._project_client.wait_for_service(timeout_sec=0.05):
            self._freeze(
                f"projection service unavailable: {self.get_parameter('project_point_service').value}",
                error=True,
            )
            return

        req = ProjectPointToLine.Request()
        req.point = PointStamped()
        req.point.header = msg.header
        req.point.point = msg.pose.position
        req.max_cross_track_error_m = float(self.get_parameter("max_cross_track_error_m").value)
        req.allow_out_of_bounds = bool(self.get_parameter("allow_out_of_bounds_projection").value)

        future = self._project_client.call_async(req)
        future.add_done_callback(lambda fut: self._on_projection_done(fut, generation))

    def _on_projection_done(self, future: Any, generation: int) -> None:
        if generation != self._generation:
            return

        try:
            result = future.result()
        except Exception as exc:
            self._freeze(f"projection service call failed: {exc}", error=True)
            return

        if result is None:
            self._freeze("projection service returned no result", error=True)
            return

        if not bool(result.success):
            # Conservative behavior: do not freeze. Keep waiting for next valid intercept pose.
            self.get_logger().warn(f"projection rejected; waiting for next intercept pose: {result.message}")
            self._set_state(InterceptionState.ARMED_WAITING, "projection rejected; waiting again")
            return

        self.get_logger().info(
            f"projection ok: s={result.s:.6f}, "
            f"cross_track={result.cross_track_error_m:.6f}, "
            f"half_length={result.line_half_length_m:.6f}"
        )
        self._send_goto_s_goal(float(result.s), generation)

    def _resolve_goto_s_command(self) -> int:
        const_name = str(self.get_parameter("cmd_goto_s_constant_name").value)
        fallback = int(self.get_parameter("cmd_goto_s_fallback_value").value)

        if hasattr(self._action_type.Goal, const_name):
            return int(getattr(self._action_type.Goal, const_name))

        if fallback >= 0:
            self.get_logger().warn(
                f"Action goal has no constant '{const_name}', using fallback value {fallback}"
            )
            return fallback

        raise RuntimeError(
            f"Could not resolve CMD_GOTO_S. "
            f"Set cmd_goto_s_constant_name or cmd_goto_s_fallback_value."
        )

    def _set_goal_field_if_present(self, goal: Any, field: str, value: Any) -> None:
        if hasattr(goal, field):
            setattr(goal, field, value)

    def _send_goto_s_goal(self, s: float, generation: int) -> None:
        if generation != self._generation:
            return

        if not self._action_client.wait_for_server(timeout_sec=0.2):
            self._freeze(
                f"trajectory action server unavailable: {self.get_parameter('trajectory_action_name').value}",
                error=True,
            )
            return

        try:
            command_value = self._resolve_goto_s_command()
        except Exception as exc:
            self._freeze(str(exc), error=True)
            return

        goal = self._action_type.Goal()

        self._set_goal_field_if_present(goal, "command", command_value)
        self._set_goal_field_if_present(goal, "target_s", float(s))

        ee_name = str(self.get_parameter("ee_name").value)
        if ee_name:
            self._set_goal_field_if_present(goal, "ee_name", ee_name)

        profile_name = str(self.get_parameter("profile_name").value)
        if profile_name:
            self._set_goal_field_if_present(goal, "profile_name", profile_name)

        v_max = float(self.get_parameter("v_max").value)
        a_max = float(self.get_parameter("a_max").value)
        j_max = float(self.get_parameter("j_max").value)
        self._set_goal_field_if_present(goal, "v_max", v_max)
        self._set_goal_field_if_present(goal, "a_max", a_max)
        self._set_goal_field_if_present(goal, "j_max", j_max)
        self._set_goal_field_if_present(goal, "repetitions", int(self.get_parameter("repetitions").value))

        self._set_state(
            InterceptionState.SENDING_ACTION,
            (
                f"sending CMD_GOTO_S target_s={s:.6f}, v_max={v_max:.3f} a_max={a_max:.3f}"
            ),
        )

        self._active_command_v_max = v_max
        self._active_command_a_max = a_max
        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(lambda fut: self._on_goal_response(fut, generation))

    def _on_goal_response(self, future: Any, generation: int) -> None:
        if generation != self._generation:
            return

        try:
            goal_handle = future.result()
        except Exception as exc:
            self._freeze(f"failed to send action goal: {exc}", error=True)
            return

        if goal_handle is None or not goal_handle.accepted:
            self._freeze("trajectory executor rejected CMD_GOTO_S goal", error=True)
            return

        self._active_goal_handle = goal_handle
        self._set_state(InterceptionState.EXECUTING, "trajectory executor accepted goal")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda fut: self._on_action_result(fut, generation))

    def _on_action_result(self, future: Any, generation: int) -> None:
        if generation != self._generation:
            return

        try:
            wrapped = future.result()
        except Exception as exc:
            self._freeze(f"trajectory action result failed: {exc}", error=True)
            return

        status = getattr(wrapped, "status", None)
        result = getattr(wrapped, "result", None)

        limits_suffix = ""
        if (
            self._active_command_v_max is not None
            and self._active_command_a_max is not None
        ):
            limits_suffix = (
                f" | commanded_limits: "
                f"v_max={self._active_command_v_max:.3f}m/s "
                f"a_max={self._active_command_a_max:.3f}m/s^2"
            )

        if status == GoalStatus.STATUS_SUCCEEDED:
            msg = "interception motion completed"
            if result is not None and hasattr(result, "message"):
                msg += f": {result.message}"
            self._freeze(msg + limits_suffix, error=False)
            return

        msg = f"interception motion ended with status={status}"
        if result is not None and hasattr(result, "message"):
            msg += f": {result.message}"
        self._freeze(msg + limits_suffix, error=True)

    def _check_wait_timeout(self) -> None:
        if self._state != InterceptionState.ARMED_WAITING:
            return

        if self._waiting_start_time_sec is None:
            return

        max_wait = float(self.get_parameter("max_wait_after_arm_sec").value)
        if max_wait <= 0.0:
            return

        elapsed = self._now_sec() - self._waiting_start_time_sec
        if elapsed > max_wait:
            self._freeze(
                f"timed out waiting for intercept pose after {elapsed:.3f}s",
                error=True,
            )

    def _publish_status(self) -> None:
        msg = String()
        msg.data = (
            f"state={self._state.value}; "
            f"generation={self._generation}; "
            f"info={self._last_info}; "
            f"last_error={self._last_error}"
        )
        self._status_pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = InterceptionController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()