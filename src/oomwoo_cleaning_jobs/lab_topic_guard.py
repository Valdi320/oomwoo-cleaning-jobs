"""Fail closed when an experimental ROS2 publisher resolves outside the lab.

ROS2 launch/CLI remapping applies *after* validating a node parameter. Check
the actual rclpy Publisher.topic_name immediately after create_publisher and
before installing any timer capable of producing Twist. This is a *lab only*
containment check, not a guarantee of physical motor safety or exclusivity.
"""
from __future__ import annotations

LAB_OUTPUT_TOPIC = "/valdi/lab_safe_cmd_vel_output"


class LabTopicGuardError(ValueError):
    """A lab publisher resolved to an unexpected ROS2 topic."""


def require_lab_output_publisher(publisher: object) -> str:
    """Check the rclpy-resolved topic, not the nominal publisher argument.

    Any missing/malformed/erroring topic_name is unsafe, including relative
    names, namespaces, aliases and ROS2 remappings to /cmd_vel. Only the one
    absolute lab name is permitted. No messages should be sent before this.
    """
    try:
        resolved = publisher.topic_name
    except Exception as exc:
        raise LabTopicGuardError(
            "unable to verify resolved ROS2 lab output topic") from exc
    if type(resolved) is not str or resolved != LAB_OUTPUT_TOPIC:
        raise LabTopicGuardError(
            "resolved ROS2 velocity output is not the isolated lab topic")
    return resolved


# Names are isolation constraints, not publisher authentication.
LAB_INPUT_TOPICS = (
    "/valdi/nav2_cmd_vel",
    "/valdi/dock_drive_cmd_vel",
    "/odom",
    "/battery_state",
    "/navigate_to_pose/_action/status",
    "/dock_drive/state",
    "/dock_drive/docked",
)


LAB_THREE_WAY_INPUT_TOPICS = LAB_INPUT_TOPICS + (
    "/valdi/sweep_cmd_vel", "/coverage_planner/motion_state", "/map",
    "/clock", "/coverage_meter/sim_unstable", "/tf", "/tf_static",
)


LAB_MANAGED_NAV_INPUT_TOPICS = LAB_THREE_WAY_INPUT_TOPICS + (
    "/navigate_to_pose/_action/feedback",
)

LAB_KEEPOUT_INPUT_TOPICS = (
    "/oomwoo/keepout_mask", "/oomwoo/keepout_filter_info",
    "/global_costmap/costmap",
)


def require_lab_input_subscriptions(subscriptions: object, *, three_way=False,
                                    managed_nav2=False, keepout_required=False) -> None:
    """Deny ROS2 subscriber remaps or source collisions before timers."""
    expected = (LAB_MANAGED_NAV_INPUT_TOPICS if managed_nav2 else
                LAB_THREE_WAY_INPUT_TOPICS if three_way else LAB_INPUT_TOPICS)
    if keepout_required:
        if not three_way:
            raise LabTopicGuardError("keepout requires the three-way lab policy")
        expected += LAB_KEEPOUT_INPUT_TOPICS
    if (type(subscriptions) is not dict
            or set(subscriptions) != set(expected)
            or len(set(map(id, subscriptions.values()))) != len(expected)):
        raise LabTopicGuardError("lab relay requires distinct expected input handles")
    for nominal in expected:
        try:
            resolved = subscriptions[nominal].topic_name
        except Exception as exc:
            raise LabTopicGuardError("unable to inspect resolved input topic") from exc
        if type(resolved) is not str or resolved != nominal:
            raise LabTopicGuardError(
                "ROS2 lab input remapped away from its isolated topic")
