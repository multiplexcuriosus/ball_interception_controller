#!/usr/bin/env python3

from __future__ import annotations

import importlib
import json
import math
import re
import threading
import time
from enum import Enum
from typing import Any, Dict, Optional, Tuple, Type

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool, Trigger

from fr3_husky_msgs.srv import ProjectPointToLine
from intercept_latency_monitor.msg import LatencyTrace

from ball_interception_controller.execution_backends import (
    ContTrackerBackend,
    ExecutionBackend,
    GoalRequest,
    GotoSBackend,
)


class InterceptionState(str, Enum):
    FROZEN = "FROZEN"
    RESETTING = "RESETTING"
    ROLLOUT_RESET_PENDING = "ROLLOUT_RESET_PENDING"
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


def _all_finite(values: Tuple[float, ...]) -> bool:
    return all(math.isfinite(v) for v in values)


def _require_in_range(name: str, value: float, minimum: float, maximum: float) -> None:
    if value < minimum or value > maximum:
        raise ValueError(f"{name} must be in [{minimum}, {maximum}], got {value}")


def _require_minimum(name: str, value: float, minimum: float) -> None:
    if value < minimum:
        raise ValueError(f"{name} must be >= {minimum}, got {value}")


def _normalize_quaternion(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Optional[Tuple[float, float, float, float]]:
    if not _all_finite((x, y, z, w)):
        return None

    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12:
        return None

    inv = 1.0 / norm
    return (x * inv, y * inv, z * inv, w * inv)


def _quat_to_rot_matrix(
    x: float,
    y: float,
    z: float,
    w: float,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]:
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    return (
        (1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)),
        (2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)),
        (2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)),
    )


class InterceptionController(Node):
    """
    One-shot ball interception coordinator.

    Flow:
            1. ~/arm
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
        self.declare_parameter("command_source", "scene")
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
        self.declare_parameter("execution_backend", "cont_tracker")
        self.declare_parameter(
            "cont_tracker_action_name",
            "/cont_tracker",
        )
        self.declare_parameter(
            "cont_tracker_target_topic",
            "/cont_tracker/target_s",
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

        self.declare_parameter(
            "commanded_target_table_topic",
            "~/commanded_target_table",
        )
        self.declare_parameter(
            "table_pose_robot_base_topic",
            "/scene_localizer/table_pose_robot_base",
        )
        self.declare_parameter("table_frame", "table_frame")
        self.declare_parameter("max_table_pose_age_sec", 1.0)

        self.declare_parameter("require_reset_service", True)
        self.declare_parameter("status_publish_rate_hz", 2.0)
        self.declare_parameter("debug_log", False)
        self.declare_parameter("rollout_prediction_topic", "/act/intercept_prediction_current_abs_s")
        self.declare_parameter("current_tcp_s_topic", "/middle_line/current_tcp_s")
        self.declare_parameter("max_current_tcp_s_age_sec", 0.15)
        self.declare_parameter("rollout_prediction_timeout_sec", 0.25)
        self.declare_parameter("rollout_min_target_s_m", -0.15)
        self.declare_parameter("rollout_max_target_s_m", 0.15)
        self.declare_parameter("rollout_reset_service", "/act/reset_temporal_aggregation")
        self.declare_parameter("rollout_reset_timeout_sec", 0.75)
        self.declare_parameter("rollout_post_reset_guard_sec", 0.05)
        self.declare_parameter("dry_run", True)
        self.declare_parameter("enable_latency_trace", False)
        self.declare_parameter("latency_trace_topic", "/intercept_trace/controller")
        self.declare_parameter("latency_run_id", "")

        action_type_string = str(self.get_parameter("trajectory_action_type").value)
        try:
            self._action_type = _load_action_type(action_type_string)
        except Exception as exc:
            raise RuntimeError(
                f"Failed to load trajectory_action_type='{action_type_string}'. "
                "Fix the parameter to match your trajectory executor action type."
            ) from exc

        self._command_source = self._validate_command_source()
        self._validate_rollout_configuration(validate_bounds=False)
        self._mode_lock = threading.Lock()
        self._dry_run = bool(self.get_parameter("dry_run").value)

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
        self._warn_throttle_last_sec: Dict[str, float] = {}
        self._rollout_last_valid_receive_time_sec: Optional[float] = None
        self._latest_rollout_target_s: Optional[float] = None
        self._latest_current_tcp_s: Optional[float] = None
        self._latest_current_tcp_s_receive_time_sec: Optional[float] = None
        self._rollout_reset_pending = False
        self._rollout_predictions_accepted = False
        self._rollout_reset_request_generation: Optional[int] = None
        self._rollout_reset_request_start_sec: Optional[float] = None
        self._rollout_reset_ack_time_sec: Optional[float] = None
        self._rollout_last_ack_epoch: Optional[int] = None
        self._rollout_post_reset_guard_until_sec: Optional[float] = None
        self._latency_trace_enabled = bool(self.get_parameter("enable_latency_trace").value)
        self._latency_sequence = 0
        self._active_trace_sequence = 0
        self._active_trace_source_stamp_ns = 0
        self._active_trace_receipt_ns = 0
        self._active_trace_start_ns = 0
        self._active_trace_start_steady_ns = 0
        self._latest_selected_target_s: Optional[float] = None

        self._latest_table_t_base_table: Optional[Tuple[float, float, float]] = None
        self._latest_table_r_base_table: Optional[
            Tuple[Tuple[float, float, float], Tuple[float, float, float], Tuple[float, float, float]]
        ] = None
        self._latest_table_pose_header_stamp_sec: Optional[float] = None
        self._latest_table_pose_receive_time_sec: Optional[float] = None
        self._latest_table_pose_frame_id: str = ""

        self._projection_request_generation: Optional[int] = None
        self._projection_request_base_target_xyz: Optional[Tuple[float, float, float]] = None

        self._pending_publish_generation: Optional[int] = None
        self._pending_publish_target_s: Optional[float] = None
        self._pending_publish_cross_track_error_m: Optional[float] = None
        self._pending_publish_base_target_xyz: Optional[Tuple[float, float, float]] = None
        self._pending_publish_table_target_xyz: Optional[Tuple[float, float, float]] = None
        backend_name = str(self.get_parameter("execution_backend").value).strip().lower()
        if backend_name == "goto_s":
            self._execution_backend: ExecutionBackend = GotoSBackend(
                self,
                action_name=str(self.get_parameter("trajectory_action_name").value),
                action_type=self._action_type,
                callbacks=self,
            )
        elif backend_name == "cont_tracker":
            self._execution_backend = ContTrackerBackend(
                self,
                action_name=str(self.get_parameter("cont_tracker_action_name").value),
                action_type=self._action_type,
                target_topic=str(self.get_parameter("cont_tracker_target_topic").value),
                callbacks=self,
            )
        else:
            raise ValueError(
                "execution_backend must be one of {'goto_s', 'cont_tracker'}, "
                f"got '{self.get_parameter('execution_backend').value}'"
            )

        self._action_client = self._execution_backend.action_client

        self._reset_client = self.create_client(
            Trigger,
            str(self.get_parameter("trajectory_reset_service").value),
        )
        self._project_client = self.create_client(
            ProjectPointToLine,
            str(self.get_parameter("project_point_service").value),
        )
        self._rollout_reset_client = self.create_client(
            Trigger,
            str(self.get_parameter("rollout_reset_service").value),
        )

        self._arm_srv = self.create_service(
            Trigger,
            "~/arm",
            self._handle_arm,
        )
        self._disarm_srv = self.create_service(
            Trigger,
            "~/disarm",
            self._handle_disarm,
        )
        self._set_dry_run_srv = self.create_service(
            SetBool,
            "~/set_dry_run",
            self._handle_set_dry_run,
        )

        self._intercept_sub = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("intercept_pose_topic").value),
            self._handle_intercept_pose,
            10,
        )
        self._rollout_prediction_sub = self.create_subscription(
            Float64,
            str(self.get_parameter("rollout_prediction_topic").value),
            self._handle_rollout_prediction,
            1,
        )
        if self._command_source == "rollout":
            # current_tcp_s is high-rate latest-state telemetry used only by rollout mode
            # during live arming after rollout-reset acknowledgement.
            current_tcp_s_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            self._current_tcp_s_sub = self.create_subscription(
                Float64,
                str(self.get_parameter("current_tcp_s_topic").value),
                self._handle_current_tcp_s,
                current_tcp_s_qos,
            )
        else:
            # Scene mode projects Cartesian intercept poses to line coordinates and
            # does not consume /middle_line/current_tcp_s.
            self._current_tcp_s_sub = None

        self._table_pose_sub = self.create_subscription(
            PoseStamped,
            str(self.get_parameter("table_pose_robot_base_topic").value),
            self._handle_table_pose_robot_base,
            10,
        )

        self._status_pub = self.create_publisher(
            String,
            "~/status",
            10,
        )
        self._selected_goto_s_pub = self.create_publisher(
            Float64,
            "~/selected_goto_s",
            10,
        )
        self._commanded_target_table_pub = self.create_publisher(
            PointStamped,
            str(self.get_parameter("commanded_target_table_topic").value),
            10,
        )
        self._latency_trace_pub = None
        if self._latency_trace_enabled:
            trace_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            self._latency_trace_pub = self.create_publisher(
                LatencyTrace,
                str(self.get_parameter("latency_trace_topic").value),
                trace_qos,
            )

        status_hz = max(0.2, float(self.get_parameter("status_publish_rate_hz").value))
        self._status_timer = self.create_timer(1.0 / status_hz, self._publish_status)
        self._timeout_timer = self.create_timer(0.05, self._check_wait_timeout)

        self.get_logger().info(
            "interception_controller started: "
            f"command_source={self._command_source}, "
            f"intercept_pose_topic={self.get_parameter('intercept_pose_topic').value}, "
            f"rollout_prediction_topic={self.get_parameter('rollout_prediction_topic').value}, "
            f"reset_service={self.get_parameter('trajectory_reset_service').value}, "
            f"project_service={self.get_parameter('project_point_service').value}, "
            f"action={self.get_parameter('trajectory_action_name').value}, "
            f"execution_backend={self.get_parameter('execution_backend').value}, "
            f"action_type={action_type_string}, "
            f"dry_run={self._get_dry_run()}, "
            f"current_tcp_s_topic={self.get_parameter('current_tcp_s_topic').value}, "
            f"rollout_reset_service={self.get_parameter('rollout_reset_service').value}, "
            f"rollout_prediction_timeout_sec={float(self.get_parameter('rollout_prediction_timeout_sec').value):.3f}, "
            "rollout_s_positive_direction=+X_in_robot_base"
        )

    def _get_dry_run(self) -> bool:
        with self._mode_lock:
            return bool(self._dry_run)

    def _trace_input(self, event: str, source_stamp_ns: int = 0) -> int:
        """Start a trace sequence for one source callback and record receipt."""
        if not self._latency_trace_enabled:
            return 0
        self._latency_sequence += 1
        sequence = self._latency_sequence
        receipt_ns = self.get_clock().now().nanoseconds
        self._active_trace_sequence = sequence
        self._active_trace_source_stamp_ns = source_stamp_ns
        self._active_trace_receipt_ns = receipt_ns
        self._active_trace_start_ns = 0
        self._active_trace_start_steady_ns = time.monotonic_ns()
        self._trace(
            "input", event, sequence=sequence, source_stamp_ns=source_stamp_ns,
            receipt_ns=receipt_ns, start_ns=receipt_ns, end_ns=receipt_ns,
        )
        return sequence

    def _trace_decision_start(self, sequence: int) -> None:
        if not self._latency_trace_enabled:
            return
        self._active_trace_start_ns = self.get_clock().now().nanoseconds
        self._active_trace_start_steady_ns = time.monotonic_ns()
        self._trace(
            "decision", "start", sequence=sequence,
            receipt_ns=self._active_trace_receipt_ns,
            start_ns=self._active_trace_start_ns,
        )

    def _trace(
        self,
        stage: str,
        event: str,
        *,
        valid: bool = True,
        detail: Optional[Dict[str, Any]] = None,
        scalar_value: float = 0.0,
        sequence: Optional[int] = None,
        source_stamp_ns: Optional[int] = None,
        receipt_ns: Optional[int] = None,
        start_ns: Optional[int] = None,
        end_ns: Optional[int] = None,
    ) -> None:
        """Publish a best-effort trace; the disabled path is one boolean check."""
        if not self._latency_trace_enabled or self._latency_trace_pub is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        steady_ns = time.monotonic_ns()
        msg = LatencyTrace()
        msg.run_id = str(self.get_parameter("latency_run_id").value)
        msg.stage = stage
        msg.event = event
        msg.modality = self._command_source
        msg.node_name = self.get_name()
        resolved_sequence = self._active_trace_sequence if sequence is None else sequence
        if not resolved_sequence:
            self._latency_sequence += 1
            resolved_sequence = self._latency_sequence
        msg.sequence = int(resolved_sequence)
        msg.parent_sequence = 0
        source_ns = self._active_trace_source_stamp_ns if source_stamp_ns is None else source_stamp_ns
        msg.source_stamp_ns = int(source_ns)
        msg.receipt_ros_stamp_ns = int(
            self._active_trace_receipt_ns if receipt_ns is None else receipt_ns
        )
        msg.start_ros_stamp_ns = int(self._active_trace_start_ns if start_ns is None else start_ns)
        msg.end_ros_stamp_ns = int(now_ns if end_ns is None else end_ns)
        msg.start_steady_ns = self._active_trace_start_steady_ns or steady_ns
        msg.end_steady_ns = steady_ns
        msg.valid = bool(valid)
        msg.scalar_value = float(scalar_value)
        payload = {"source": self._command_source}
        if detail:
            payload.update(detail)
        msg.detail_json = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        self._latency_trace_pub.publish(msg)

    def _trace_rejection(self, reason: str, **detail: Any) -> None:
        detail["rejection_reason"] = reason
        self._trace("gating", "rejected", valid=False, detail=detail)

    def _trace_decision_end(self, valid: bool, reason: str = "") -> None:
        detail = {"outcome": "accepted" if valid else "rejected"}
        if reason:
            detail["rejection_reason"] = reason
        self._trace("decision", "end", valid=valid, detail=detail)

    def _set_dry_run(self, dry_run: bool) -> None:
        with self._mode_lock:
            self._dry_run = bool(dry_run)

    def _validate_command_source(self) -> str:
        value = str(self.get_parameter("command_source").value).strip().lower()
        if value not in {"scene", "rollout"}:
            raise ValueError(
                "command_source must be one of {'scene', 'rollout'}, "
                f"got '{self.get_parameter('command_source').value}'"
            )
        return value

    def _validate_rollout_configuration(self, validate_bounds: bool) -> None:
        timeout_sec = float(self.get_parameter("rollout_prediction_timeout_sec").value)
        if timeout_sec <= 0.0:
            raise ValueError(
                f"rollout_prediction_timeout_sec must be > 0, got {timeout_sec}"
            )

        max_current_tcp_s_age_sec = float(self.get_parameter("max_current_tcp_s_age_sec").value)
        _require_minimum("max_current_tcp_s_age_sec", max_current_tcp_s_age_sec, 0.0)

        reset_timeout_sec = float(self.get_parameter("rollout_reset_timeout_sec").value)
        if reset_timeout_sec <= 0.0:
            raise ValueError(
                f"rollout_reset_timeout_sec must be > 0, got {reset_timeout_sec}"
            )

        reset_guard_sec = float(self.get_parameter("rollout_post_reset_guard_sec").value)
        _require_minimum("rollout_post_reset_guard_sec", reset_guard_sec, 0.0)

        if validate_bounds:
            min_target = float(self.get_parameter("rollout_min_target_s_m").value)
            max_target = float(self.get_parameter("rollout_max_target_s_m").value)
            if min_target >= max_target:
                raise ValueError(
                    "rollout_min_target_s_m must be < rollout_max_target_s_m before arming "
                    f"rollout mode, got min={min_target} max={max_target}"
                )

    def _clear_rollout_receive_state(self) -> None:
        self._rollout_last_valid_receive_time_sec = None
        self._latest_rollout_target_s = None

    def _disable_rollout_prediction_acceptance(self) -> None:
        self._rollout_predictions_accepted = False
        self._rollout_post_reset_guard_until_sec = None
        self._clear_rollout_receive_state()

    def _parse_rollout_epoch(self, message: str) -> Optional[int]:
        m = re.search(r"rollout_epoch=(\d+)", str(message))
        if m is None:
            return None
        try:
            return int(m.group(1))
        except (TypeError, ValueError):
            return None

    def _waiting_description(self) -> str:
        if self._command_source == "rollout":
            return "valid rollout prediction"
        return "scene intercept pose"

    def _publish_selected_goto_s(self, s: float, generation: int) -> None:
        if generation != self._generation:
            return
        msg = Float64()
        msg.data = float(s)
        self._selected_goto_s_pub.publish(msg)
        self._latest_selected_target_s = float(s)
        self._trace(
            "selection", "target_created", scalar_value=float(s),
            detail={"generation": generation},
        )

    def _latest_current_tcp_s_if_fresh(self) -> Optional[float]:
        if self._latest_current_tcp_s is None or self._latest_current_tcp_s_receive_time_sec is None:
            return None
        age_sec = self._now_sec() - self._latest_current_tcp_s_receive_time_sec
        max_age_sec = float(self.get_parameter("max_current_tcp_s_age_sec").value)
        if age_sec > max_age_sec:
            return None
        return float(self._latest_current_tcp_s)

    def _warn_throttled(self, key: str, msg: str, period_sec: float = 1.0) -> None:
        now_sec = self._now_sec()
        last_sec = self._warn_throttle_last_sec.get(key)
        if last_sec is None or (now_sec - last_sec) >= period_sec:
            self._warn_throttle_last_sec[key] = now_sec
            self.get_logger().warn(msg)

    def _clear_projection_request_cache(self, generation: Optional[int] = None) -> None:
        if generation is not None and self._projection_request_generation != generation:
            return
        self._projection_request_generation = None
        self._projection_request_base_target_xyz = None

    def _clear_pending_publish_data(self, generation: Optional[int] = None) -> None:
        if generation is not None and self._pending_publish_generation != generation:
            return
        self._pending_publish_generation = None
        self._pending_publish_target_s = None
        self._pending_publish_cross_track_error_m = None
        self._pending_publish_base_target_xyz = None
        self._pending_publish_table_target_xyz = None

    def _cache_projection_request_target(self, generation: int, msg: PoseStamped) -> None:
        p = msg.pose.position
        self._projection_request_generation = generation
        self._projection_request_base_target_xyz = (float(p.x), float(p.y), float(p.z))

    def _resolve_projection_base_target(
        self,
        result: Any,
        generation: int,
    ) -> Optional[Tuple[float, float, float]]:
        projected_point = getattr(result, "projected_point", None)
        if projected_point is not None and hasattr(projected_point, "point"):
            pp = projected_point.point
            projected_xyz = (float(pp.x), float(pp.y), float(pp.z))
            if _all_finite(projected_xyz):
                return projected_xyz

        if self._projection_request_generation == generation and self._projection_request_base_target_xyz is not None:
            # ProjectPointToLine currently returns scalar s but no Cartesian projected point.
            # We still send result.s as the actual CMD_GOTO_S command scalar.
            # The published Cartesian point is then the requested middle-line intersection point,
            # which may differ slightly from the executor's internal projected line point when
            # cross-track error is nonzero.
            return self._projection_request_base_target_xyz

        return None

    def _base_to_table_target(
        self,
        p_base: Tuple[float, float, float],
    ) -> Optional[Tuple[float, float, float]]:
        if self._latest_table_t_base_table is None or self._latest_table_r_base_table is None:
            self._warn_throttled(
                "table_pose_missing",
                "skipping commanded_target_table publish: no valid table pose received yet",
                period_sec=1.0,
            )
            return None

        expected_base_frame = _clean_frame(str(self.get_parameter("expected_frame").value))
        table_pose_frame = _clean_frame(self._latest_table_pose_frame_id)
        if expected_base_frame and table_pose_frame and table_pose_frame != expected_base_frame:
            self._warn_throttled(
                "table_pose_frame_mismatch",
                (
                    "skipping commanded_target_table publish: table pose frame_id "
                    f"'{table_pose_frame}' != expected base frame '{expected_base_frame}'"
                ),
                period_sec=1.0,
            )
            return None

        now_sec = self._now_sec()
        max_age_sec = max(0.0, float(self.get_parameter("max_table_pose_age_sec").value))
        pose_time_sec = self._latest_table_pose_header_stamp_sec
        if pose_time_sec is None or pose_time_sec <= 0.0:
            pose_time_sec = self._latest_table_pose_receive_time_sec

        if pose_time_sec is None:
            self._warn_throttled(
                "table_pose_no_timestamp",
                "skipping commanded_target_table publish: table pose has no usable timestamp",
                period_sec=1.0,
            )
            return None

        age_sec = now_sec - pose_time_sec
        if age_sec > max_age_sec:
            self._warn_throttled(
                "table_pose_stale",
                (
                    "skipping commanded_target_table publish: table pose is stale "
                    f"age={age_sec:.3f}s max={max_age_sec:.3f}s"
                ),
                period_sec=1.0,
            )
            return None

        tx, ty, tz = self._latest_table_t_base_table
        px, py, pz = p_base
        dx = px - tx
        dy = py - ty
        dz = pz - tz

        r = self._latest_table_r_base_table
        p_table = (
            r[0][0] * dx + r[1][0] * dy + r[2][0] * dz,
            r[0][1] * dx + r[1][1] * dy + r[2][1] * dz,
            r[0][2] * dx + r[1][2] * dy + r[2][2] * dz,
        )
        if not _all_finite(p_table):
            self._warn_throttled(
                "table_transform_non_finite",
                "skipping commanded_target_table publish: non-finite table-space target",
                period_sec=1.0,
            )
            return None

        return p_table

    def _handle_table_pose_robot_base(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        values = (
            float(p.x),
            float(p.y),
            float(p.z),
            float(q.x),
            float(q.y),
            float(q.z),
            float(q.w),
        )
        if not _all_finite(values):
            self._warn_throttled(
                "table_pose_non_finite",
                "ignoring non-finite table_pose_robot_base",
                period_sec=1.0,
            )
            return

        normalized = _normalize_quaternion(float(q.x), float(q.y), float(q.z), float(q.w))
        if normalized is None:
            self._warn_throttled(
                "table_pose_zero_quat",
                "ignoring table_pose_robot_base with zero-norm quaternion",
                period_sec=1.0,
            )
            return

        qx, qy, qz, qw = normalized
        self._latest_table_t_base_table = (float(p.x), float(p.y), float(p.z))
        self._latest_table_r_base_table = _quat_to_rot_matrix(qx, qy, qz, qw)
        self._latest_table_pose_header_stamp_sec = self._stamp_to_sec(msg)
        self._latest_table_pose_receive_time_sec = self._now_sec()
        self._latest_table_pose_frame_id = msg.header.frame_id

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
            self._trace(
                "controller_state", "transition",
                detail={"from": self._state.value, "to": state.value, "reason": info},
            )
        self._state = state
        if info:
            self._last_info = info

    def _freeze(self, reason: str, error: bool = False) -> None:
        self._active_goal_handle = None
        self._waiting_start_time_sec = None
        self._accept_after_time_sec = None
        self._rollout_reset_pending = False
        self._rollout_reset_request_generation = None
        self._rollout_reset_request_start_sec = None
        self._clear_projection_request_cache()
        self._clear_pending_publish_data()
        self._disable_rollout_prediction_acceptance()
        self._execution_backend.shutdown()

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
            InterceptionState.ROLLOUT_RESET_PENDING,
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
        self._clear_projection_request_cache()
        self._clear_pending_publish_data()
        self._disable_rollout_prediction_acceptance()
        self._rollout_reset_ack_time_sec = None
        self._rollout_last_ack_epoch = None

        if self._command_source == "rollout":
            try:
                self._validate_rollout_configuration(validate_bounds=True)
            except ValueError as exc:
                self._freeze(str(exc), error=True)
                response.success = False
                response.message = self._last_error
                return response

            if self._execution_backend.backend_name != "cont_tracker":
                self._freeze("rollout mode requires execution_backend=cont_tracker", error=True)
                response.success = False
                response.message = self._last_error
                return response

            reset_service_name = str(self.get_parameter("rollout_reset_service").value)
            if not self._rollout_reset_client.wait_for_service(timeout_sec=0.2):
                self._freeze(f"rollout reset service unavailable: {reset_service_name}", error=True)
                response.success = False
                response.message = self._last_error
                return response

            self._rollout_reset_pending = True
            self._rollout_reset_request_generation = generation
            self._rollout_reset_request_start_sec = self._now_sec()
            self._set_state(InterceptionState.ROLLOUT_RESET_PENDING, "rollout reset requested; awaiting acknowledgement")

            dry_run = self._get_dry_run()
            try:
                future = self._rollout_reset_client.call_async(Trigger.Request())
            except Exception as exc:
                self._freeze(f"rollout reset request failed: {exc}", error=True)
                response.success = False
                response.message = self._last_error
                return response

            future.add_done_callback(lambda fut: self._on_rollout_reset_done(fut, generation, dry_run))
            mode = "dry-run" if dry_run else "live"
            response.success = True
            response.message = (
                f"Interception arm requested in rollout {mode} mode; "
                "waiting for temporal-aggregation reset acknowledgement."
            )
            return response

        require_reset = bool(self.get_parameter("require_reset_service").value)
        reset_service_name = str(self.get_parameter("trajectory_reset_service").value)

        if require_reset and not self._reset_client.wait_for_service(timeout_sec=0.2):
            self._freeze(f"reset service unavailable: {reset_service_name}", error=True)
            response.success = False
            response.message = self._last_error
            return response

        if require_reset and self._reset_client.service_is_ready():
            self._set_state(
                InterceptionState.RESETTING,
                "calling trajectory estimator reset",
            )
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

    def _on_rollout_reset_done(self, future: Any, generation: int, dry_run: bool) -> None:
        if generation != self._generation:
            return
        if not self._rollout_reset_pending:
            return
        if self._rollout_reset_request_generation != generation:
            return

        self._rollout_reset_pending = False
        self._rollout_reset_request_start_sec = None

        try:
            result = future.result()
        except Exception as exc:
            self._freeze(f"rollout reset service call failed: {exc}", error=True)
            return

        if result is None or not bool(getattr(result, "success", False)):
            msg = "rollout reset rejected"
            if result is not None and hasattr(result, "message"):
                msg = f"rollout reset rejected: {result.message}"
            self._freeze(msg, error=True)
            return

        ack_time_sec = self._now_sec()
        self._rollout_reset_ack_time_sec = ack_time_sec
        self._rollout_last_ack_epoch = self._parse_rollout_epoch(getattr(result, "message", ""))
        self._rollout_predictions_accepted = True
        guard_sec = float(self.get_parameter("rollout_post_reset_guard_sec").value)
        self._rollout_post_reset_guard_until_sec = ack_time_sec + guard_sec
        self._arm_time_sec = ack_time_sec
        self._waiting_start_time_sec = ack_time_sec
        self._rollout_last_valid_receive_time_sec = None
        self._latest_rollout_target_s = None
        self._accept_after_time_sec = None

        if dry_run:
            self._set_state(
                InterceptionState.ARMED_WAITING,
                "rollout reset acknowledged; awaiting post-reset predictions",
            )
            return

        current_tcp_s = self._latest_current_tcp_s_if_fresh()
        if current_tcp_s is None:
            self._freeze(
                "rollout live arming requires a fresh finite /middle_line/current_tcp_s sample after reset acknowledgement",
                error=True,
            )
            return

        min_target = float(self.get_parameter("rollout_min_target_s_m").value)
        max_target = float(self.get_parameter("rollout_max_target_s_m").value)
        if current_tcp_s < min_target or current_tcp_s > max_target:
            self._freeze(
                "rollout live arming current_tcp_s outside configured bounds after reset acknowledgement: "
                f"s={current_tcp_s:.6f} min={min_target:.6f} max={max_target:.6f}",
                error=True,
            )
            return

        self._publish_selected_goto_s(current_tcp_s, generation)
        self._latest_rollout_target_s = float(current_tcp_s)
        self._send_goto_s_goal(float(current_tcp_s), generation)

    def _handle_disarm(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request

        if self._state == InterceptionState.EXECUTING and self._execution_backend.backend_name != "cont_tracker":
            response.success = False
            response.message = "Already executing; not canceling automatically. Use trajectory executor cancel if needed."
            return response

        generation = self._generation
        if self._execution_backend.is_active():
            self._execution_backend.cancel("disarmed by service call", generation)
        self._rollout_reset_pending = False
        self._rollout_reset_request_generation = None
        self._rollout_reset_request_start_sec = None
        self._generation += 1
        self._clear_projection_request_cache()
        self._clear_pending_publish_data()
        self._disable_rollout_prediction_acceptance()
        self._freeze("disarmed by service call", error=False)
        response.success = True
        response.message = "Interception controller disarmed/frozen."
        return response

    def _handle_set_dry_run(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        requested = bool(request.data)
        if self._state != InterceptionState.FROZEN:
            response.success = False
            response.message = (
                "Rejected: controller must be FROZEN; "
                f"current state={self._state.value}; dry_run={self._get_dry_run()}"
            )
            self.get_logger().warn(
                f"set_dry_run rejected: requested={requested} state={self._state.value}"
            )
            return response

        self._set_dry_run(requested)
        response.success = True
        response.message = f"dry_run={self._get_dry_run()}"
        self.get_logger().info(
            f"set_dry_run accepted: requested={requested} resulting={self._get_dry_run()}"
        )
        return response

    def _handle_intercept_pose(self, msg: PoseStamped) -> None:
        source_stamp_ns = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )
        sequence = self._trace_input("scene_input_received", source_stamp_ns)
        self._trace_decision_start(sequence)
        if self._command_source != "scene":
            self._trace_rejection("inactive_command_source", received_source="scene")
            self._trace_decision_end(False, "inactive_command_source")
            return

        if self._state != InterceptionState.ARMED_WAITING:
            if self._execution_backend.backend_name != "cont_tracker" or not self._execution_backend.is_active():
                self._trace_rejection("controller_not_armed", state=self._state.value)
                self._trace_decision_end(False, "controller_not_armed")
                return

        if self._state not in {InterceptionState.ARMED_WAITING, InterceptionState.EXECUTING}:
            self._trace_rejection("controller_state_gate", state=self._state.value)
            self._trace_decision_end(False, "controller_state_gate")
            return

        now_sec = self._now_sec()

        if self._accept_after_time_sec is not None and now_sec < self._accept_after_time_sec:
            self._debug("ignoring intercept pose during post-reset ignore window")
            self._trace_rejection("post_reset_ignore_window")
            self._trace_decision_end(False, "post_reset_ignore_window")
            return

        expected_frame = _clean_frame(str(self.get_parameter("expected_frame").value))
        msg_frame = _clean_frame(msg.header.frame_id)
        if expected_frame and msg_frame != expected_frame:
            self.get_logger().warn(
                f"ignoring intercept pose with frame_id='{msg_frame}', expected='{expected_frame}'"
            )
            self._trace_rejection(
                "frame_mismatch", frame_id=msg_frame, expected_frame=expected_frame
            )
            self._trace_decision_end(False, "frame_mismatch")
            return

        stamp_sec = self._stamp_to_sec(msg)
        if stamp_sec > 0.0:
            max_age = max(0.0, float(self.get_parameter("max_intercept_pose_age_sec").value))
            age = now_sec - stamp_sec
            if age > max_age:
                self._debug(f"ignoring stale intercept pose age={age:.3f}s max={max_age:.3f}s")
                self._trace_rejection("stale_input", age_sec=age, max_age_sec=max_age)
                self._trace_decision_end(False, "stale_input")
                return

        p = msg.pose.position
        if not all(map(lambda x: x == x and abs(x) != float("inf"), [p.x, p.y, p.z])):
            self.get_logger().warn("ignoring non-finite intercept pose")
            self._trace_rejection("non_finite_input")
            self._trace_decision_end(False, "non_finite_input")
            return

        generation = self._generation
        self._set_state(InterceptionState.PROJECTING, "projecting intercept pose to executor line")

        if not self._project_client.wait_for_service(timeout_sec=0.05):
            self._trace_rejection("projection_service_unavailable")
            self._trace_decision_end(False, "projection_service_unavailable")
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
        self._cache_projection_request_target(generation, msg)

        future = self._project_client.call_async(req)
        future.add_done_callback(lambda fut: self._on_projection_done(fut, generation))
        self._trace("projection", "submitted", detail={"generation": generation})

    def _handle_current_tcp_s(self, msg: Float64) -> None:
        value = float(msg.data)
        if not math.isfinite(value):
            self._warn_throttled(
                "current_tcp_s_non_finite",
                "ignoring non-finite /middle_line/current_tcp_s",
                period_sec=1.0,
            )
            return
        self._latest_current_tcp_s = value
        self._latest_current_tcp_s_receive_time_sec = self._now_sec()

    def _handle_rollout_prediction(self, msg: Float64) -> None:
        sequence = self._trace_input("act_rollout_prediction_received")
        self._trace_decision_start(sequence)
        if self._command_source != "rollout":
            self._trace_rejection("inactive_command_source", received_source="rollout")
            self._trace_decision_end(False, "inactive_command_source")
            return

        if self._rollout_reset_pending or not self._rollout_predictions_accepted:
            self._trace_rejection("rollout_reset_gate", reset_pending=self._rollout_reset_pending)
            self._trace_decision_end(False, "rollout_reset_gate")
            return

        now_sec = self._now_sec()
        if (
            self._rollout_post_reset_guard_until_sec is not None
            and now_sec < self._rollout_post_reset_guard_until_sec
        ):
            self._trace_rejection("post_reset_guard")
            self._trace_decision_end(False, "post_reset_guard")
            return

        if self._state not in {
            InterceptionState.ARMED_WAITING,
            InterceptionState.SENDING_ACTION,
            InterceptionState.EXECUTING,
        }:
            self._trace_rejection("controller_state_gate", state=self._state.value)
            self._trace_decision_end(False, "controller_state_gate")
            return

        target_s_m = float(msg.data)
        if not math.isfinite(target_s_m):
            self._warn_throttled(
                "rollout_non_finite",
                "ignoring non-finite rollout target scalar",
                period_sec=1.0,
            )
            self._trace_rejection("non_finite_prediction", probability_available=False)
            self._trace_decision_end(False, "non_finite_prediction")
            return

        min_target = float(self.get_parameter("rollout_min_target_s_m").value)
        max_target = float(self.get_parameter("rollout_max_target_s_m").value)
        if target_s_m < min_target or target_s_m > max_target:
            self._warn_throttled(
                "rollout_oob",
                "ignoring rollout prediction outside configured target_s bounds",
                period_sec=1.0,
            )
            self._trace_rejection(
                "target_out_of_bounds", target_s=target_s_m,
                minimum=min_target, maximum=max_target, probability_available=False,
            )
            self._trace_decision_end(False, "target_out_of_bounds")
            return

        self._rollout_last_valid_receive_time_sec = now_sec
        self._latest_rollout_target_s = float(target_s_m)
        self._publish_selected_goto_s(float(target_s_m), self._generation)
        self._trace_decision_end(True)
        mode = "dry-run" if self._get_dry_run() else "live"
        self.get_logger().info(
            f"accepted rollout target_s={target_s_m:.6f} mode={mode} generation={self._generation}"
        )

        if self._get_dry_run():
            return

        self._execution_backend.update_target(float(target_s_m), self._generation)
        self._trace(
            "command", "target_update_published", scalar_value=float(target_s_m),
            detail={"backend": self._execution_backend.backend_name},
        )

    def _on_projection_done(self, future: Any, generation: int) -> None:
        if generation != self._generation:
            self._clear_projection_request_cache(generation)
            self._clear_pending_publish_data(generation)
            return

        try:
            result = future.result()
        except Exception as exc:
            self._trace_rejection("projection_service_exception", error=str(exc))
            self._trace_decision_end(False, "projection_service_exception")
            self._freeze(f"projection service call failed: {exc}", error=True)
            return

        if result is None:
            self._trace_rejection("projection_no_result")
            self._trace_decision_end(False, "projection_no_result")
            self._freeze("projection service returned no result", error=True)
            return

        if not bool(result.success):
            # Conservative behavior: do not freeze. Keep waiting for next valid intercept pose.
            self.get_logger().warn(f"projection rejected; waiting for next intercept pose: {result.message}")
            self._trace_rejection("projection_rejected", service_message=str(result.message))
            self._trace_decision_end(False, "projection_rejected")
            self._clear_projection_request_cache(generation)
            self._clear_pending_publish_data(generation)
            self._set_state(InterceptionState.ARMED_WAITING, "projection rejected; waiting again")
            return

        base_target_xyz = self._resolve_projection_base_target(result, generation)
        self._trace_decision_end(True)
        self._clear_projection_request_cache(generation)
        if base_target_xyz is None:
            self._warn_throttled(
                "projection_base_target_missing",
                "projection succeeded but no base target is available for commanded_target_table publication",
                period_sec=1.0,
            )
            self._clear_pending_publish_data(generation)
            self._send_goto_s_goal(float(result.s), generation)
            return

        if self._execution_backend.backend_name == "cont_tracker" and self._execution_backend.is_active():
            self._execution_backend.update_target(float(result.s), generation)
            self._trace(
                "command", "target_update_published", scalar_value=float(result.s),
                detail={"backend": self._execution_backend.backend_name},
            )
            self._clear_pending_publish_data(generation)
            self._set_state(InterceptionState.EXECUTING, f"cont_tracker update target_s={float(result.s):.6f}")
            return

        table_target_xyz = self._base_to_table_target(base_target_xyz)
        self._pending_publish_generation = generation
        self._pending_publish_target_s = float(result.s)
        self._pending_publish_cross_track_error_m = float(result.cross_track_error_m)
        self._pending_publish_base_target_xyz = base_target_xyz
        self._pending_publish_table_target_xyz = table_target_xyz

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
            self._clear_pending_publish_data(generation)
            return

        self._publish_selected_goto_s(s, generation)

        if self._execution_backend.backend_name == "cont_tracker" and self._execution_backend.is_active():
            self._execution_backend.update_target(float(s), generation)
            self._trace(
                "command", "target_update_published", scalar_value=float(s),
                detail={"backend": self._execution_backend.backend_name},
            )
            self._set_state(
                InterceptionState.EXECUTING,
                f"updating cont_tracker target_s={float(s):.6f}",
            )
            return

        if self._get_dry_run():
            source_label = self._command_source.upper()
            self._freeze(
                f"{source_label}_DRY_RUN selected target_s={float(s):.6f}",
                error=False,
            )
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
        self._trace(
            "command", "goto_s_submitted", scalar_value=float(s),
            detail={"backend": self._execution_backend.backend_name, "generation": generation},
        )
        self._execution_backend.start(GoalRequest(goal=goal, target_s=float(s), generation=generation))

    def on_backend_goal_accepted(self, backend_name: str, generation: int, goal_handle: Any) -> None:
        if generation != self._generation:
            self._clear_pending_publish_data(generation)
            return

        if (
            self._pending_publish_generation == generation
            and self._pending_publish_table_target_xyz is not None
            and self._pending_publish_base_target_xyz is not None
            and self._pending_publish_target_s is not None
            and self._pending_publish_cross_track_error_m is not None
        ):
            table_xyz = self._pending_publish_table_target_xyz
            msg = PointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = str(self.get_parameter("table_frame").value)
            msg.point.x = table_xyz[0]
            msg.point.y = table_xyz[1]
            msg.point.z = table_xyz[2]
            self._commanded_target_table_pub.publish(msg)

            base_xyz = self._pending_publish_base_target_xyz
            self.get_logger().info(
                "published accepted CMD_GOTO_S target: "
                f"s={self._pending_publish_target_s:.6f}, "
                f"base=[{base_xyz[0]:.6f}, {base_xyz[1]:.6f}, {base_xyz[2]:.6f}], "
                f"table=[{table_xyz[0]:.6f}, {table_xyz[1]:.6f}, {table_xyz[2]:.6f}], "
                f"cross_track={self._pending_publish_cross_track_error_m:.6f}"
            )

        self._clear_pending_publish_data(generation)

        self._active_goal_handle = goal_handle
        accepted_s = self._latest_selected_target_s
        self._trace(
            "command", "target_acceptance_acknowledged",
            scalar_value=0.0 if accepted_s is None else float(accepted_s),
            detail={"backend": backend_name, "generation": generation},
        )
        self._set_state(InterceptionState.EXECUTING, "trajectory executor accepted goal")

    def on_backend_goal_rejected(self, backend_name: str, generation: int, reason: str) -> None:
        self._trace(
            "command", "target_acceptance_rejected", valid=False,
            detail={"backend": backend_name, "generation": generation, "rejection_reason": reason},
        )
        self._clear_pending_publish_data()
        self._freeze(reason, error=True)

    def on_backend_result(self, backend_name: str, generation: int, status: Any, result: Any) -> None:
        del backend_name
        if generation != self._generation:
            self._clear_pending_publish_data(generation)
            return

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

    def on_backend_cancel_complete(self, backend_name: str, generation: int, result: Any) -> None:
        del backend_name, result
        if generation != self._generation:
            return
        self._freeze("interception motion canceled", error=False)

    def on_backend_feedback(self, backend_name: str, generation: int, feedback: Any) -> None:
        del backend_name, generation, feedback

    def on_backend_error(self, backend_name: str, generation: int, error: Exception) -> None:
        del backend_name, generation
        self._freeze(f"backend error: {error}", error=True)

    def _check_wait_timeout(self) -> None:
        if self._command_source == "rollout":
            if self._rollout_reset_pending:
                if self._rollout_reset_request_start_sec is None:
                    return
                timeout_sec = float(self.get_parameter("rollout_reset_timeout_sec").value)
                elapsed = self._now_sec() - self._rollout_reset_request_start_sec
                if elapsed > timeout_sec:
                    self._freeze(
                        f"rollout reset acknowledgement timeout after {elapsed:.3f}s (limit {timeout_sec:.3f}s)",
                        error=True,
                    )
                return

            if self._state not in {
                InterceptionState.ARMED_WAITING,
                InterceptionState.SENDING_ACTION,
                InterceptionState.EXECUTING,
            }:
                return

            timeout_sec = float(self.get_parameter("rollout_prediction_timeout_sec").value)
            now_sec = self._now_sec()

            if self._rollout_last_valid_receive_time_sec is None:
                if self._arm_time_sec is None:
                    return
                elapsed = now_sec - self._arm_time_sec
                if elapsed > timeout_sec:
                    self._warn_throttled(
                        "rollout_first_prediction_timeout",
                        (
                            "rollout watchdog timeout waiting for first valid scalar: "
                            f"elapsed={elapsed:.3f}s timeout={timeout_sec:.3f}s"
                        ),
                        period_sec=1.0,
                    )
                    self._freeze(
                        f"timed out waiting for first valid rollout scalar after {elapsed:.3f}s",
                        error=True,
                    )
                return

            gap = now_sec - self._rollout_last_valid_receive_time_sec
            if gap > timeout_sec:
                self._warn_throttled(
                    "rollout_prediction_gap_timeout",
                    (
                        "rollout watchdog timeout: "
                        f"last valid scalar age={gap:.3f}s timeout={timeout_sec:.3f}s"
                    ),
                    period_sec=1.0,
                )
                self._freeze(
                    f"rollout scalar stream timed out: age={gap:.3f}s > {timeout_sec:.3f}s",
                    error=True,
                )
            return

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
                f"timed out waiting for {self._waiting_description()} after {elapsed:.3f}s",
                error=True,
            )

    def _publish_status(self) -> None:
        now_sec = self._now_sec()
        rollout_stream_age = "n/a"
        if self._rollout_last_valid_receive_time_sec is not None:
            rollout_stream_age = f"{(now_sec - self._rollout_last_valid_receive_time_sec):.3f}s"

        has_valid_current_tcp_s = self._latest_current_tcp_s_if_fresh() is not None
        current_tcp_s_age = "n/a"
        if self._latest_current_tcp_s_receive_time_sec is not None:
            current_tcp_s_age = f"{(now_sec - self._latest_current_tcp_s_receive_time_sec):.3f}s"

        latest_rollout_target = "n/a"
        if self._latest_rollout_target_s is not None:
            latest_rollout_target = f"{self._latest_rollout_target_s:.6f}"

        reset_ack_age = "n/a"
        if self._rollout_reset_ack_time_sec is not None:
            reset_ack_age = f"{(now_sec - self._rollout_reset_ack_time_sec):.3f}s"

        reset_epoch = "n/a"
        if self._rollout_last_ack_epoch is not None:
            reset_epoch = str(self._rollout_last_ack_epoch)

        msg = String()
        msg.data = (
            f"state={self._state.value}; "
            f"generation={self._generation}; "
            f"command_source={self._command_source}; "
            f"execution_backend={self._execution_backend.backend_name}; "
            f"dry_run={self._get_dry_run()}; "
            f"rollout_stream_age={rollout_stream_age}; "
            f"rollout_prediction_timeout_sec={float(self.get_parameter('rollout_prediction_timeout_sec').value):.3f}; "
            f"has_valid_current_tcp_s={has_valid_current_tcp_s}; "
            f"current_tcp_s_age={current_tcp_s_age}; "
            f"latest_rollout_target_s={latest_rollout_target}; "
            f"rollout_reset_service={self.get_parameter('rollout_reset_service').value}; "
            f"rollout_reset_pending={self._rollout_reset_pending}; "
            f"rollout_last_ack_epoch={reset_epoch}; "
            f"rollout_reset_ack_age={reset_ack_age}; "
            f"rollout_predictions_accepted={self._rollout_predictions_accepted}; "
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
