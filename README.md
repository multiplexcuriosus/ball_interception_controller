# ball_interception_controller

Coordinates one-shot scene-based or ACT-rollout interception using the existing
`GOTO_S` and continuous-tracker interfaces.

## Optional latency tracing

Tracing is disabled by default and does not alter controller topics, services,
actions, safety gates, modes, or their QoS. Enable it through the launch file:

```bash
ros2 launch ball_interception_controller interception_controller.launch.py \
  enable_latency_trace:=true latency_run_id:=trial_001
```

The controller publishes best-effort
`intercept_latency_monitor/msg/LatencyTrace` records on
`/intercept_trace/controller` by default. `latency_trace_topic` changes only
this new trace topic. Records cover scene and rollout receipt, decision timing,
gate/projection rejection, target selection, command submission/update, and
action-goal acceptance. `modality` and `detail_json.source` identify `scene` or
`rollout`; `scalar_value` carries the selected line target when applicable.

The existing rollout input is `std_msgs/Float64`, so it has no probability or
source timestamp. Traces record `probability_available=false` for rollout
validation failures and leave `source_stamp_ns` at zero. Scene inputs carry
their header timestamp.
