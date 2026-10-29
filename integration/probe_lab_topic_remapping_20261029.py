"""Real ROS2 Jazzy topic-resolution probe, never emits a Twist or starts a timer.

Run ONLY in a networkless container with no physical robot connectivity.
LAB_EXPECT and LAB_EXPECT_RESOLVED are used solely for a test assertion.
"""
from __future__ import annotations

import json
import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from oomwoo_cleaning_jobs.lab_topic_guard import (
    LAB_OUTPUT_TOPIC, LabTopicGuardError, require_lab_output_publisher)


def main():
    rclpy.init(args=sys.argv[1:])
    node = None
    report = None
    try:
        node = Node("valdi_lab_output_topic_probe")
        publisher = node.create_publisher(Twist, LAB_OUTPUT_TOPIC, 10)
        try:
            try:
                require_lab_output_publisher(publisher)
                actual_result = "allowed"
            except LabTopicGuardError:
                actual_result = "rejected"
            report = {
                "nominal": LAB_OUTPUT_TOPIC,
                "actual_resolved_topic": publisher.topic_name,
                "guard_result": actual_result,
                "no_messages_sent": True,
            }
        finally:
            node.destroy_publisher(publisher)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report, sort_keys=True))
    expected_result = os.environ.get("LAB_EXPECT", "allowed")
    expected_topic = os.environ.get("LAB_EXPECT_RESOLVED", LAB_OUTPUT_TOPIC)
    if report["guard_result"] != expected_result or (
            report["actual_resolved_topic"] != expected_topic):
        raise SystemExit("ROS2 lab output guard result mismatched expected topic")


if __name__ == "__main__":
    main()
