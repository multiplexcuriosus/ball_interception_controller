import importlib
import math
from pathlib import Path
import sys
import types

import pytest


class FakeParameter:
    def __init__(self, value):
        self.value = value


class FakeTimeMsg:
    def __init__(self, sec: int = 0, nanosec: int = 0):
        self.sec = sec
        self.nanosec = nanosec


class FakeTime:
    def __init__(self, seconds: float):
        self.nanoseconds = int(seconds * 1e9)

    def to_msg(self):
        sec = int(self.nanoseconds // 1_000_000_000)
        nanosec = int(self.nanoseconds % 1_000_000_000)
        return FakeTimeMsg(sec=sec, nanosec=nanosec)


class FakeClock:
    def __init__(self):
        self.current_time_sec = 0.0

    def now(self):
        return FakeTime(self.current_time_sec)


class FakeLogger:
    def __init__(self):
        self.records = []

    def info(self, message):
        self.records.append(("info", message))

    def warn(self, message):
        self.records.append(("warn", message))


class FakePublisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeFuture:
    def __init__(self, result=None):
        self._result = result
        self.callbacks = []

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def result(self):
        return self._result


class FakeClient:
    def __init__(self, name):
        self.name = name
        self.wait_ready = True
        self.ready = True
        self.calls = []
        self.future = FakeFuture(types.SimpleNamespace(success=True, message="ok"))

    def wait_for_service(self, timeout_sec=0.0):
        del timeout_sec
        return self.wait_ready

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.calls.append(request)
        return self.future


class FakeGoalHandle:
    accepted = True

    def get_result_async(self):
        return FakeFuture()


class FakeActionClient:
    def __init__(self, node, action_type, name):
        del node, action_type
        self.name = name
        self.wait_ready = True
        self.goals = []

    def wait_for_server(self, timeout_sec=0.0):
        del timeout_sec
        return self.wait_ready

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return FakeFuture(FakeGoalHandle())


class FakeNode:
    def __init__(self, name):
        self._node_name = name
        self._parameters = {}
        self._clock = FakeClock()
        self._logger = FakeLogger()
        self._subscriptions = []

    def declare_parameter(self, name, value):
        self._parameters[name] = value
        return FakeParameter(value)

    def get_parameter(self, name):
        return FakeParameter(self._parameters[name])

    def create_client(self, srv_type, name):
        del srv_type
        return FakeClient(name)

    def create_service(self, srv_type, name, callback):
        del srv_type, callback
        return types.SimpleNamespace(name=name)

    def create_subscription(self, msg_type, topic, callback, qos):
        del msg_type, callback
        sub = types.SimpleNamespace(topic=topic, qos=qos)
        self._subscriptions.append(sub)
        return sub

    def create_publisher(self, msg_type, topic, depth):
        del msg_type, depth
        return FakePublisher(topic)

    def create_timer(self, period_sec, callback):
        del period_sec, callback
        return object()

    def get_logger(self):
        return self._logger

    def get_clock(self):
        return self._clock

    def get_name(self):
        return self._node_name

    def destroy_node(self):
        return None


class FakeHeader:
    def __init__(self):
        self.stamp = FakeTimeMsg()
        self.frame_id = ""


class FakePoint:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class FakeQuaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class FakePose:
    def __init__(self):
        self.position = FakePoint()
        self.orientation = FakeQuaternion()


class FakePointStamped:
    def __init__(self):
        self.header = FakeHeader()
        self.point = FakePoint()


class FakePoseStamped:
    def __init__(self):
        self.header = FakeHeader()
        self.pose = FakePose()


class FakeString:
    def __init__(self):
        self.data = ""


class FakeFloat32MultiArray:
    def __init__(self):
        self.data = []


class FakeFloat64:
    def __init__(self):
        self.data = 0.0


class FakeLatencyTrace:
    pass


class FakeQoSHistoryPolicy:
    KEEP_LAST = "KEEP_LAST"


class FakeQoSReliabilityPolicy:
    BEST_EFFORT = "BEST_EFFORT"
    RELIABLE = "RELIABLE"


class FakeQoSDurabilityPolicy:
    VOLATILE = "VOLATILE"
    TRANSIENT_LOCAL = "TRANSIENT_LOCAL"


class FakeQoSProfile:
    def __init__(self, history, depth, reliability, durability):
        self.history = history
        self.depth = depth
        self.reliability = reliability
        self.durability = durability


class FakeTriggerRequest:
    pass


class FakeTriggerResponse:
    def __init__(self):
        self.success = False
        self.message = ""


class FakeTrigger:
    Request = FakeTriggerRequest
    Response = FakeTriggerResponse


class FakeSetBoolRequest:
    def __init__(self):
        self.data = False


class FakeSetBoolResponse:
    def __init__(self):
        self.success = False
        self.message = ""


class FakeSetBool:
    Request = FakeSetBoolRequest
    Response = FakeSetBoolResponse


class FakeProjectPointRequest:
    def __init__(self):
        self.point = None
        self.max_cross_track_error_m = 0.0
        self.allow_out_of_bounds = False


class FakeProjectPointToLine:
    Request = FakeProjectPointRequest


class FakeGoal:
    CMD_GOTO_S = 7

    def __init__(self):
        self.command = None
        self.target_s = None
        self.ee_name = None
        self.profile_name = None
        self.v_max = None
        self.a_max = None
        self.j_max = None
        self.repetitions = None


class FakeAction:
    Goal = FakeGoal


@pytest.fixture(scope="module")
def controller_module():
    package_root = Path(__file__).resolve().parents[1]
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))

    modules = {
        "rclpy": types.ModuleType("rclpy"),
        "rclpy.node": types.ModuleType("rclpy.node"),
        "rclpy.qos": types.ModuleType("rclpy.qos"),
        "rclpy.action": types.ModuleType("rclpy.action"),
        "action_msgs": types.ModuleType("action_msgs"),
        "action_msgs.msg": types.ModuleType("action_msgs.msg"),
        "geometry_msgs": types.ModuleType("geometry_msgs"),
        "geometry_msgs.msg": types.ModuleType("geometry_msgs.msg"),
        "std_msgs": types.ModuleType("std_msgs"),
        "std_msgs.msg": types.ModuleType("std_msgs.msg"),
        "std_srvs": types.ModuleType("std_srvs"),
        "std_srvs.srv": types.ModuleType("std_srvs.srv"),
        "fr3_husky_msgs": types.ModuleType("fr3_husky_msgs"),
        "fr3_husky_msgs.srv": types.ModuleType("fr3_husky_msgs.srv"),
        "fr3_husky_msgs.action": types.ModuleType("fr3_husky_msgs.action"),
        "intercept_latency_monitor": types.ModuleType("intercept_latency_monitor"),
        "intercept_latency_monitor.msg": types.ModuleType("intercept_latency_monitor.msg"),
    }

    modules["rclpy"].init = lambda args=None: None
    modules["rclpy"].shutdown = lambda: None
    modules["rclpy"].spin = lambda node: None
    modules["rclpy.node"].Node = FakeNode
    modules["rclpy.qos"].QoSProfile = FakeQoSProfile
    modules["rclpy.qos"].QoSHistoryPolicy = FakeQoSHistoryPolicy
    modules["rclpy.qos"].QoSReliabilityPolicy = FakeQoSReliabilityPolicy
    modules["rclpy.qos"].QoSDurabilityPolicy = FakeQoSDurabilityPolicy
    modules["rclpy.action"].ActionClient = FakeActionClient
    modules["action_msgs.msg"].GoalStatus = types.SimpleNamespace(STATUS_SUCCEEDED=4)
    modules["geometry_msgs.msg"].PointStamped = FakePointStamped
    modules["geometry_msgs.msg"].PoseStamped = FakePoseStamped
    modules["std_msgs.msg"].String = FakeString
    modules["std_msgs.msg"].Float32MultiArray = FakeFloat32MultiArray
    modules["std_msgs.msg"].Float64 = FakeFloat64
    modules["std_srvs.srv"].Trigger = FakeTrigger
    modules["std_srvs.srv"].SetBool = FakeSetBool
    modules["fr3_husky_msgs.srv"].ProjectPointToLine = FakeProjectPointToLine
    modules["fr3_husky_msgs.action"].LineTrajectory = FakeAction
    modules["intercept_latency_monitor.msg"].LatencyTrace = FakeLatencyTrace

    sys.modules.update(modules)
    return importlib.import_module("ball_interception_controller.interception_controller")


@pytest.fixture()
def controller(controller_module):
    node = controller_module.InterceptionController()
    node.get_clock().current_time_sec = 100.0
    return node


def create_controller_with_command_source(controller_module, command_source: str):
    original_declare = FakeNode.declare_parameter

    def declare_with_source(self, name, value):
        if name == "command_source":
            value = command_source
        return original_declare(self, name, value)

    FakeNode.declare_parameter = declare_with_source
    try:
        node = controller_module.InterceptionController()
    finally:
        FakeNode.declare_parameter = original_declare
    node.get_clock().current_time_sec = 100.0
    return node


def create_controller_with_parameters(controller_module, **overrides):
    original_declare = FakeNode.declare_parameter

    def declare_with_overrides(self, name, value):
        return original_declare(self, name, overrides.get(name, value))

    FakeNode.declare_parameter = declare_with_overrides
    try:
        node = controller_module.InterceptionController()
    finally:
        FakeNode.declare_parameter = original_declare
    node.get_clock().current_time_sec = 100.0
    return node


@pytest.fixture()
def rollout_controller(controller_module):
    return create_controller_with_command_source(controller_module, "rollout")


def set_param(controller, name, value):
    controller._parameters[name] = value
    if name == "command_source":
        controller._command_source = str(value)
    if name == "dry_run":
        controller._dry_run = bool(value)


def make_pose(controller_module, stamp_sec=100.0, frame_id="base"):
    msg = controller_module.PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = int(stamp_sec)
    msg.header.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1e9)
    msg.pose.position.x = 0.1
    msg.pose.position.y = 0.2
    msg.pose.position.z = 0.3
    return msg


def make_rollout(controller_module, target_s, probability):
    msg = controller_module.Float32MultiArray()
    msg.data = [target_s, probability]
    return msg


def prepare_rollout(controller, controller_module, dry_run=True):
    set_param(controller, "command_source", "rollout")
    set_param(controller, "rollout_min_target_s_m", 0.1)
    set_param(controller, "rollout_max_target_s_m", 1.0)
    set_param(controller, "rollout_post_arm_ignore_sec", 0.0)
    set_param(controller, "dry_run", dry_run)
    controller._state = controller_module.InterceptionState.ARMED_WAITING
    controller._accept_after_time_sec = 0.0


def test_default_command_source_is_scene(controller):
    assert controller._command_source == "scene"


def test_scene_controller_does_not_subscribe_current_tcp_s(controller):
    assert controller._command_source == "scene"
    assert controller._current_tcp_s_sub is None


def test_rollout_controller_subscribes_current_tcp_s_with_best_effort_qos(controller_module, rollout_controller):
    assert rollout_controller._command_source == "rollout"
    sub = rollout_controller._current_tcp_s_sub
    assert sub is not None
    assert sub.topic == rollout_controller.get_parameter("current_tcp_s_topic").value
    assert sub.qos.reliability == controller_module.QoSReliabilityPolicy.BEST_EFFORT
    assert sub.qos.history == controller_module.QoSHistoryPolicy.KEEP_LAST
    assert sub.qos.depth == 1
    assert sub.qos.durability == controller_module.QoSDurabilityPolicy.VOLATILE


def test_scene_behavior_is_unchanged_by_default(controller_module, controller):
    controller._state = controller_module.InterceptionState.ARMED_WAITING
    controller._waiting_start_time_sec = 99.0
    controller._accept_after_time_sec = 0.0

    controller._handle_intercept_pose(make_pose(controller_module))

    assert controller._state == controller_module.InterceptionState.PROJECTING
    assert len(controller._project_client.calls) == 1


def test_scene_pose_handling_still_calls_project_point_to_line(controller_module, controller):
    controller._state = controller_module.InterceptionState.ARMED_WAITING
    controller._waiting_start_time_sec = 99.0
    controller._accept_after_time_sec = 0.0

    controller._handle_intercept_pose(make_pose(controller_module))

    assert len(controller._project_client.calls) == 1


def test_live_rollout_arming_rejects_missing_current_tcp_s(controller_module, rollout_controller):
    set_param(rollout_controller, "dry_run", False)
    response = controller_module.Trigger.Response()

    rollout_controller._handle_arm(controller_module.Trigger.Request(), response)

    assert response.success is True
    assert rollout_controller._state == controller_module.InterceptionState.ROLLOUT_RESET_PENDING
    rollout_controller._on_rollout_reset_done(
        FakeFuture(types.SimpleNamespace(success=True, message="rollout_epoch=11")),
        rollout_controller._generation,
        False,
    )

    assert rollout_controller._state == controller_module.InterceptionState.FROZEN
    assert "requires a fresh finite /middle_line/current_tcp_s" in rollout_controller._last_error


def test_live_rollout_arming_rejects_stale_current_tcp_s(controller_module, rollout_controller):
    set_param(rollout_controller, "dry_run", False)
    set_param(rollout_controller, "max_current_tcp_s_age_sec", 0.10)
    rollout_controller._latest_current_tcp_s = 0.02
    rollout_controller._latest_current_tcp_s_receive_time_sec = 99.0
    response = controller_module.Trigger.Response()

    rollout_controller._handle_arm(controller_module.Trigger.Request(), response)
    rollout_controller._on_rollout_reset_done(
        FakeFuture(types.SimpleNamespace(success=True, message="rollout_epoch=12")),
        rollout_controller._generation,
        False,
    )

    assert rollout_controller._state == controller_module.InterceptionState.FROZEN
    assert "requires a fresh finite /middle_line/current_tcp_s" in rollout_controller._last_error


def test_live_rollout_arming_accepts_fresh_finite_in_bounds_current_tcp_s(controller_module, rollout_controller):
    set_param(rollout_controller, "dry_run", False)
    set_param(rollout_controller, "rollout_min_target_s_m", -0.15)
    set_param(rollout_controller, "rollout_max_target_s_m", 0.15)
    rollout_controller._latest_current_tcp_s = 0.03
    rollout_controller._latest_current_tcp_s_receive_time_sec = rollout_controller._now_sec()
    sent = []
    rollout_controller._send_goto_s_goal = lambda s, generation: sent.append((s, generation))
    response = controller_module.Trigger.Response()

    rollout_controller._handle_arm(controller_module.Trigger.Request(), response)
    rollout_controller._on_rollout_reset_done(
        FakeFuture(types.SimpleNamespace(success=True, message="rollout_epoch=13")),
        rollout_controller._generation,
        False,
    )

    assert sent == [(0.03, rollout_controller._generation)]
    assert rollout_controller._selected_goto_s_pub.messages[-1].data == pytest.approx(0.03)


def test_rollout_dry_run_arming_does_not_depend_on_current_tcp_s(controller_module, rollout_controller):
    set_param(rollout_controller, "dry_run", True)
    rollout_controller._latest_current_tcp_s = None
    rollout_controller._latest_current_tcp_s_receive_time_sec = None
    sent = []
    rollout_controller._send_goto_s_goal = lambda s, generation: sent.append((s, generation))
    response = controller_module.Trigger.Response()

    rollout_controller._handle_arm(controller_module.Trigger.Request(), response)
    rollout_controller._on_rollout_reset_done(
        FakeFuture(types.SimpleNamespace(success=True, message="rollout_epoch=14")),
        rollout_controller._generation,
        True,
    )

    assert rollout_controller._state == controller_module.InterceptionState.ARMED_WAITING
    assert sent == []


def test_rollout_messages_are_ignored_in_scene_mode(controller_module, controller):
    controller._state = controller_module.InterceptionState.ARMED_WAITING
    controller._accept_after_time_sec = 0.0

    controller._handle_rollout_prediction(make_rollout(controller_module, 0.2, 0.95))

    assert controller._rollout_qualifying_count() == 0


def test_rollout_messages_are_ignored_while_frozen(controller_module, controller):
    prepare_rollout(controller, controller_module)
    controller._state = controller_module.InterceptionState.FROZEN

    controller._handle_rollout_prediction(make_rollout(controller_module, 0.2, 0.95))

    assert controller._rollout_qualifying_count() == 0


@pytest.mark.parametrize(
    "data",
    [
        [0.2],
        [math.nan, 0.95],
        [0.2, math.inf],
        [1.2, 0.95],
    ],
)
def test_invalid_rollout_messages_cannot_trigger(controller_module, controller, data):
    prepare_rollout(controller, controller_module)
    sent = []
    controller._send_goto_s_goal = lambda s, generation: sent.append((s, generation))

    msg = controller_module.Float32MultiArray()
    msg.data = data
    controller._handle_rollout_prediction(msg)

    assert sent == []
    assert controller._rollout_qualifying_count() == 0
    assert controller._state == controller_module.InterceptionState.ARMED_WAITING


def test_probability_below_threshold_resets_sequence(controller_module, controller):
    prepare_rollout(controller, controller_module)

    controller._handle_rollout_prediction(make_rollout(controller_module, 0.3, 0.95))
    controller._handle_rollout_prediction(make_rollout(controller_module, 0.31, 0.80))
    controller._handle_rollout_prediction(make_rollout(controller_module, 0.32, 0.96))

    assert controller._rollout_qualifying_count() == 1


def test_excessive_receive_time_gap_resets_sequence(controller_module, controller):
    prepare_rollout(controller, controller_module)
    set_param(controller, "rollout_max_prediction_gap_sec", 0.25)

    controller._handle_rollout_prediction(make_rollout(controller_module, 0.3, 0.95))
    controller.get_clock().current_time_sec += 0.3
    controller._handle_rollout_prediction(make_rollout(controller_module, 0.31, 0.96))

    assert controller._rollout_qualifying_count() == 1


def test_fewer_than_required_consecutive_samples_cannot_trigger(controller_module, controller):
    prepare_rollout(controller, controller_module)
    set_param(controller, "rollout_required_consecutive", 3)
    sent = []
    controller._send_goto_s_goal = lambda s, generation: sent.append((s, generation))

    controller._handle_rollout_prediction(make_rollout(controller_module, 0.3, 0.95))
    controller._handle_rollout_prediction(make_rollout(controller_module, 0.31, 0.96))

    assert sent == []
    assert controller._state == controller_module.InterceptionState.ARMED_WAITING


def test_unstable_target_predictions_cannot_trigger(controller_module, controller):
    prepare_rollout(controller, controller_module)
    set_param(controller, "rollout_max_target_spread_m", 0.02)
    sent = []
    controller._send_goto_s_goal = lambda s, generation: sent.append((s, generation))

    controller._handle_rollout_prediction(make_rollout(controller_module, 0.30, 0.95))
    controller._handle_rollout_prediction(make_rollout(controller_module, 0.34, 0.96))
    controller._handle_rollout_prediction(make_rollout(controller_module, 0.32, 0.97))

    assert sent == []
    assert controller._rollout_qualifying_count() == 3


def test_stable_consecutive_predictions_select_median_target(controller_module, controller):
    prepare_rollout(controller, controller_module, dry_run=False)
    sent = []

    def record_send(target_s, generation):
        sent.append((target_s, generation))
        controller._state = controller_module.InterceptionState.SENDING_ACTION

    controller._send_goto_s_goal = record_send

    controller._handle_rollout_prediction(make_rollout(controller_module, 0.33, 0.95))
    controller._handle_rollout_prediction(make_rollout(controller_module, 0.31, 0.96))
    controller._handle_rollout_prediction(make_rollout(controller_module, 0.32, 0.97))

    assert sent == [(0.32, controller._generation)]


def test_rollout_dry_run_publishes_selected_s_and_never_sends_action(controller_module, controller):
    prepare_rollout(controller, controller_module, dry_run=True)

    controller._handle_rollout_prediction(make_rollout(controller_module, 0.31, 0.95))
    controller._handle_rollout_prediction(make_rollout(controller_module, 0.32, 0.96))
    controller._handle_rollout_prediction(make_rollout(controller_module, 0.33, 0.97))

    assert len(controller._selected_goto_s_pub.messages) == 1
    assert controller._selected_goto_s_pub.messages[0].data == pytest.approx(0.32)
    assert controller._action_client.goals == []
    assert controller._state == controller_module.InterceptionState.FROZEN


def test_live_rollout_calls_send_goal_exactly_once(controller_module, controller):
    prepare_rollout(controller, controller_module, dry_run=False)
    sent = []

    def record_send(target_s, generation):
        sent.append((target_s, generation))
        controller._state = controller_module.InterceptionState.SENDING_ACTION

    controller._send_goto_s_goal = record_send

    controller._handle_rollout_prediction(make_rollout(controller_module, 0.31, 0.95))
    controller._handle_rollout_prediction(make_rollout(controller_module, 0.32, 0.96))
    controller._handle_rollout_prediction(make_rollout(controller_module, 0.33, 0.97))
    controller._handle_rollout_prediction(make_rollout(controller_module, 0.34, 0.98))

    assert len(sent) == 1


def test_scene_dry_run_publishes_projected_s_and_never_sends_action(controller_module, controller):
    set_param(controller, "command_source", "scene")
    set_param(controller, "dry_run", True)

    result = types.SimpleNamespace(
        success=True,
        s=0.27,
        cross_track_error_m=0.001,
        line_half_length_m=0.2,
    )
    controller._on_projection_done(FakeFuture(result), controller._generation)

    assert len(controller._selected_goto_s_pub.messages) == 1
    assert controller._selected_goto_s_pub.messages[0].data == pytest.approx(0.27)
    assert controller._action_client.goals == []
    assert controller._state == controller_module.InterceptionState.FROZEN


def test_scene_live_mode_publishes_selected_s_and_sends_action(controller_module, controller):
    set_param(controller, "command_source", "scene")
    set_param(controller, "dry_run", False)

    controller._send_goto_s_goal(0.19, controller._generation)

    assert len(controller._selected_goto_s_pub.messages) == 1
    assert controller._selected_goto_s_pub.messages[0].data == pytest.approx(0.19)
    assert len(controller._action_client.goals) == 1


def test_cont_tracker_backend_publishes_updates_after_goal_acceptance(controller_module, controller):
    backend_module = importlib.import_module("ball_interception_controller.execution_backends")
    backend = backend_module.ContTrackerBackend(
        controller,
        action_name="/cont_tracker",
        action_type=FakeAction,
        target_topic="/cont_tracker/target_s",
        callbacks=controller,
    )

    goal = FakeAction.Goal()
    request = backend_module.GoalRequest(goal=goal, target_s=0.25, generation=controller._generation)
    backend.start(request)

    assert len(backend.action_client.goals) == 1

    backend._on_goal_response(FakeFuture(FakeGoalHandle()), controller._generation)
    backend.update_target(0.31, controller._generation)
    backend.update_target(0.31, controller._generation)

    assert len(backend.target_publisher.messages) == 1
    assert backend.target_publisher.messages[0].data == pytest.approx(0.31)


def test_invalid_rollout_predictions_do_not_publish_selected_s(controller_module, controller):
    prepare_rollout(controller, controller_module, dry_run=True)

    controller._handle_rollout_prediction(make_rollout(controller_module, 1.2, 0.95))
    assert controller._selected_goto_s_pub.messages == []


def test_private_controller_interfaces_are_instance_scoped(controller):
    assert controller._arm_srv.name == "~/arm"
    assert controller._disarm_srv.name == "~/disarm"
    assert controller._status_pub.topic == "~/status"
    assert controller._selected_goto_s_pub.topic == "~/selected_goto_s"
    assert controller.get_parameter("commanded_target_table_topic").value == "~/commanded_target_table"


def test_disarm_and_freeze_clear_rollout_filter_state(controller_module, controller):
    prepare_rollout(controller, controller_module)
    controller._rollout_predictions.append((0.3, 0.95))
    controller._rollout_last_receive_time_sec = 100.0

    controller._freeze("done", error=False)

    assert controller._rollout_qualifying_count() == 0
    assert controller._rollout_last_receive_time_sec is None

    prepare_rollout(controller, controller_module)
    controller._rollout_predictions.append((0.3, 0.95))
    controller._rollout_last_receive_time_sec = 100.0
    response = controller_module.Trigger.Response()
    controller._handle_disarm(controller_module.Trigger.Request(), response)

    assert response.success is True
    assert controller._rollout_qualifying_count() == 0
    assert controller._rollout_last_receive_time_sec is None


def test_arming_rollout_mode_never_calls_reset(controller_module, controller):
    set_param(controller, "command_source", "rollout")
    set_param(controller, "rollout_min_target_s_m", 0.1)
    set_param(controller, "rollout_max_target_s_m", 1.0)
    set_param(controller, "rollout_post_arm_ignore_sec", 0.0)
    response = controller_module.Trigger.Response()

    controller._handle_arm(controller_module.Trigger.Request(), response)

    assert response.success is True
    assert controller._reset_client.calls == []
    assert controller._state == controller_module.InterceptionState.ARMED_WAITING


def test_set_dry_run_rejected_when_not_frozen(controller_module, controller):
    controller._state = controller_module.InterceptionState.ARMED_WAITING
    request = controller_module.SetBool.Request()
    request.data = False
    response = controller_module.SetBool.Response()

    controller._handle_set_dry_run(request, response)

    assert response.success is False
    assert "FROZEN" in response.message


def test_set_dry_run_updates_runtime_mode_when_frozen(controller_module, controller):
    controller._state = controller_module.InterceptionState.FROZEN
    controller._set_dry_run(True)

    request = controller_module.SetBool.Request()
    request.data = False
    response = controller_module.SetBool.Response()
    controller._handle_set_dry_run(request, response)

    assert response.success is True
    assert controller._get_dry_run() is False
    assert "dry_run=False" in response.message


def test_scene_mode_without_required_reset_skips_ready_reset(controller_module, controller):
    set_param(controller, "command_source", "scene")
    set_param(controller, "require_reset_service", False)
    controller._reset_client.ready = True
    response = controller_module.Trigger.Response()

    controller._handle_arm(controller_module.Trigger.Request(), response)

    assert response.success is True
    assert controller._reset_client.calls == []
    assert controller._state == controller_module.InterceptionState.ARMED_WAITING


def trace_events(node):
    return [(msg.stage, msg.event, msg.valid) for msg in node._latency_trace_pub.messages]


def test_latency_tracing_disabled_creates_no_publisher(controller_module, controller):
    assert controller.get_parameter("enable_latency_trace").value is False
    assert controller._latency_trace_pub is None
    controller._state = controller_module.InterceptionState.ARMED_WAITING
    controller._accept_after_time_sec = 0.0
    controller._handle_intercept_pose(make_pose(controller_module))
    assert len(controller._project_client.calls) == 1


def test_scene_trace_records_receipt_rejection_and_decision(controller_module):
    node = create_controller_with_parameters(
        controller_module, enable_latency_trace=True, latency_run_id="scene-run"
    )
    node._state = controller_module.InterceptionState.ARMED_WAITING
    node._accept_after_time_sec = 0.0
    node._handle_intercept_pose(make_pose(controller_module, frame_id="wrong"))

    events = trace_events(node)
    assert ("input", "scene_input_received", True) in events
    assert ("gating", "rejected", False) in events
    assert ("decision", "end", False) in events
    assert all(
        msg.run_id == "scene-run" and msg.modality == "scene"
        for msg in node._latency_trace_pub.messages
    )
    assert "frame_mismatch" in node._latency_trace_pub.messages[-2].detail_json


def test_scene_trace_records_accepted_target_and_submission(controller_module):
    node = create_controller_with_parameters(
        controller_module, enable_latency_trace=True, dry_run=False,
        execution_backend="goto_s",
    )
    node._state = controller_module.InterceptionState.ARMED_WAITING
    node._accept_after_time_sec = 0.0
    node._handle_intercept_pose(make_pose(controller_module))
    result = types.SimpleNamespace(
        success=True, s=0.12, cross_track_error_m=0.001, line_half_length_m=0.2,
    )
    node._on_projection_done(FakeFuture(result), node._generation)

    events = trace_events(node)
    assert ("decision", "end", True) in events
    assert ("selection", "target_created", True) in events
    assert ("command", "goto_s_submitted", True) in events
    selected = next(
        msg for msg in node._latency_trace_pub.messages
        if msg.event == "target_created"
    )
    assert selected.scalar_value == pytest.approx(0.12)

    node.on_backend_goal_accepted(
        "goto_s", node._generation, FakeGoalHandle()
    )
    acknowledgement = next(
        msg for msg in node._latency_trace_pub.messages
        if msg.event == "target_acceptance_acknowledged"
    )
    assert acknowledgement.scalar_value == pytest.approx(0.12)


def test_rollout_trace_records_accepted_and_rejected_predictions(controller_module):
    node = create_controller_with_parameters(
        controller_module, enable_latency_trace=True, command_source="rollout", dry_run=True,
    )
    node._state = controller_module.InterceptionState.ARMED_WAITING
    node._rollout_predictions_accepted = True
    node._rollout_post_reset_guard_until_sec = 0.0

    bad = controller_module.Float64()
    bad.data = 2.0
    node._handle_rollout_prediction(bad)
    good = controller_module.Float64()
    good.data = 0.05
    node._handle_rollout_prediction(good)

    events = trace_events(node)
    assert ("input", "act_rollout_prediction_received", True) in events
    assert ("gating", "rejected", False) in events
    assert ("decision", "end", True) in events
    assert ("selection", "target_created", True) in events
    rejection = next(msg for msg in node._latency_trace_pub.messages if not msg.valid)
    assert "target_out_of_bounds" in rejection.detail_json
    assert "probability_available" in rejection.detail_json
