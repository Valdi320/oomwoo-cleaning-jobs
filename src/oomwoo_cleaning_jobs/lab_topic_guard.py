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
