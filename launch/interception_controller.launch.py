from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    intercept_pose_topic = LaunchConfiguration("intercept_pose_topic")
    trajectory_reset_service = LaunchConfiguration("trajectory_reset_service")
    project_point_service = LaunchConfiguration("project_point_service")
    trajectory_action_name = LaunchConfiguration("trajectory_action_name")
    trajectory_action_type = LaunchConfiguration("trajectory_action_type")
    cmd_goto_s_constant_name = LaunchConfiguration("cmd_goto_s_constant_name")
    cmd_goto_s_fallback_value = LaunchConfiguration("cmd_goto_s_fallback_value")
    ee_name = LaunchConfiguration("ee_name")
    profile_name = LaunchConfiguration("profile_name")
    v_max = LaunchConfiguration("v_max")
    a_max = LaunchConfiguration("a_max")
    j_max = LaunchConfiguration("j_max")
    repetitions = LaunchConfiguration("repetitions")
    expected_frame = LaunchConfiguration("expected_frame")
    max_intercept_pose_age_sec = LaunchConfiguration("max_intercept_pose_age_sec")
    max_cross_track_error_m = LaunchConfiguration("max_cross_track_error_m")
    allow_out_of_bounds_projection = LaunchConfiguration("allow_out_of_bounds_projection")
    max_wait_after_arm_sec = LaunchConfiguration("max_wait_after_arm_sec")
    post_reset_ignore_sec = LaunchConfiguration("post_reset_ignore_sec")
    require_reset_service = LaunchConfiguration("require_reset_service")
    status_publish_rate_hz = LaunchConfiguration("status_publish_rate_hz")
    debug_log = LaunchConfiguration("debug_log")

    interception_node = Node(
        package="ball_interception_controller",
        executable="interception_controller",
        name="interception_controller",
        output="screen",
        parameters=[
            {
                "intercept_pose_topic": intercept_pose_topic,
                "trajectory_reset_service": trajectory_reset_service,
                "project_point_service": project_point_service,
                "trajectory_action_name": trajectory_action_name,
                "trajectory_action_type": trajectory_action_type,
                "cmd_goto_s_constant_name": cmd_goto_s_constant_name,
                "cmd_goto_s_fallback_value": cmd_goto_s_fallback_value,
                "ee_name": ee_name,
                "profile_name": profile_name,
                "v_max": v_max,
                "a_max": a_max,
                "j_max": j_max,
                "repetitions": repetitions,
                "expected_frame": expected_frame,
                "max_intercept_pose_age_sec": max_intercept_pose_age_sec,
                "max_cross_track_error_m": max_cross_track_error_m,
                "allow_out_of_bounds_projection": allow_out_of_bounds_projection,
                "max_wait_after_arm_sec": max_wait_after_arm_sec,
                "post_reset_ignore_sec": post_reset_ignore_sec,
                "require_reset_service": require_reset_service,
                "status_publish_rate_hz": status_publish_rate_hz,
                "debug_log": debug_log,
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "intercept_pose_topic",
                default_value="/scene/middle_line_intersection_pose_robot_base",
                description="Pose topic with predicted middle-line intersection in robot base frame.",
            ),
            DeclareLaunchArgument(
                "trajectory_reset_service",
                default_value="/ball_trajectory_estimator/reset",
                description="Reset service called on arm to clear stale trajectory estimates.",
            ),
            DeclareLaunchArgument(
                "project_point_service",
                default_value="/trajectory_executor/project_point_to_line",
                description="Service that projects intercept point to line coordinate s.",
            ),
            DeclareLaunchArgument(
                "trajectory_action_name",
                default_value="/trajectory_executor",
                description="Trajectory action server name.",
            ),
            DeclareLaunchArgument(
                "trajectory_action_type",
                default_value="fr3_husky_msgs.action.LineTrajectory",
                description="Python action type path for the trajectory action.",
            ),
            DeclareLaunchArgument(
                "cmd_goto_s_constant_name",
                default_value="CMD_GOTO_S",
                description="Goal constant name for goto-s command.",
            ),
            DeclareLaunchArgument(
                "cmd_goto_s_fallback_value",
                default_value="-1",
                description="Fallback command value when constant lookup fails (-1 disables fallback).",
            ),
            DeclareLaunchArgument(
                "ee_name",
                default_value="",
                description="Optional end-effector name forwarded to LineTrajectory goal.",
            ),
            DeclareLaunchArgument(
                "profile_name",
                default_value="interception",
                description="Optional profile name forwarded to LineTrajectory goal.",
            ),
            DeclareLaunchArgument("v_max", default_value="1.0"),
            DeclareLaunchArgument("a_max", default_value="2.0"),
            DeclareLaunchArgument("j_max", default_value="0.0"),
            DeclareLaunchArgument("repetitions", default_value="1"),
            DeclareLaunchArgument("expected_frame", default_value="base"),
            DeclareLaunchArgument("max_intercept_pose_age_sec", default_value="0.25"),
            DeclareLaunchArgument("max_cross_track_error_m", default_value="0.03"),
            DeclareLaunchArgument("allow_out_of_bounds_projection", default_value="false"),
            DeclareLaunchArgument("max_wait_after_arm_sec", default_value="5.0"),
            DeclareLaunchArgument("post_reset_ignore_sec", default_value="0.6"),
            DeclareLaunchArgument("require_reset_service", default_value="true"),
            DeclareLaunchArgument("status_publish_rate_hz", default_value="2.0"),
            DeclareLaunchArgument("debug_log", default_value="false"),
            interception_node,
        ]
    )
