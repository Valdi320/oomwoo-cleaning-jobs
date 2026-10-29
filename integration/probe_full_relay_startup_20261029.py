"""Actual ROS2 Jazzy full-relay startup/stop and remap test; NO motor output.

Only execute from the paired networkless Podman runner. A nominal relay emits
zero Twist to an isolated lab-only topic. Rejected starts do not arm a timer.
No motion inputs, robot, Gazebo, or hardware devices are involved.
"""
from __future__ import annotations

import argparse
import json
import time
from unittest.mock import patch

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from oomwoo_cleaning_jobs.lab_topic_guard import (
    LAB_OUTPUT_TOPIC, LabTopicGuardError)
from oomwoo_cleaning_jobs.ros_dock_velocity_relay import ExperimentalDockVelocityRelay

CASE_EXPECTED = {
    "nominal": "passive_zero",
    "namespace": "passive_zero",
    "remap_motor": "denied",
    "remap_controller": "denied",
    "remap_other": "denied",
    "simulation_off": "denied",
    "override_motor": "denied",
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--case", choices=tuple(CASE_EXPECTED), required=True)
    opts, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    relay = None
    observer = None
    executor = None
    report = {
        "case": opts.case,
        "expected": CASE_EXPECTED[opts.case],
        "full_relay_imported": True,
        "published_motor_twists": 0,
        "physical_motor_connected": False,
        "test_scope": "real ROS2 Jazzy, synthetic STOP only",
    }
    try:
        enabled = opts.case != "simulation_off"
        overrides = [Parameter("simulation_lab", value=enabled)]
        if opts.case == "override_motor":
            overrides.append(Parameter("output_topic", value="/cmd_vel"))
        if CASE_EXPECTED[opts.case] == "denied":
            # Test the *real* constructor. A rejected start must never
            # register the output timer, regardless of the rejection path.
            with patch.object(
                ExperimentalDockVelocityRelay,
                "create_timer",
                side_effect=AssertionError("unsafe relay armed a timer"),
            ) as timer:
                try:
                    relay = ExperimentalDockVelocityRelay(
                        parameter_overrides=overrides)
                except (ValueError, LabTopicGuardError) as exc:
                    report["exception"] = type(exc).__name__
                    report["denied_before_timer"] = not timer.called
                else:
                    raise AssertionError("unsafe full relay startup succeeded")
            assert report["denied_before_timer"], report
            report["result"] = "denied"
        else:
            relay = ExperimentalDockVelocityRelay(
                parameter_overrides=overrides)
            assert relay.safe.topic_name == LAB_OUTPUT_TOPIC
            assert relay.mux.owner == relay.mux.STOP
            observer = Node("valdi_lab_relay_stop_observer")
            received = []
            observer.create_subscription(
                Twist, LAB_OUTPUT_TOPIC,
                lambda m: received.append((m.linear.x, m.angular.z)), 10)
            executor = SingleThreadedExecutor()
            executor.add_node(observer)
            executor.add_node(relay)
            deadline = time.monotonic() + 2.5
            while len(received) < 3 and time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.05)
            assert len(received) >= 3, "no actual ROS2 STOP output observed"
            assert all(v == (0.0, 0.0) for v in received), received
            assert not observer.get_publishers_info_by_topic("/cmd_vel"), (
                "full relay exposed a /cmd_vel publisher")
            assert relay.mux.owner == relay.mux.STOP
            report["result"] = "passive_zero"
            report["observed_safe_twists"] = len(received)
            report["all_observed_twists_zero"] = True
            report["resolved_output_topic"] = relay.safe.topic_name
        print("FULL_RELAY_STARTUP " + json.dumps(report, sort_keys=True),
              flush=True)
    finally:
        if executor is not None:
            executor.shutdown()
        if observer is not None:
            observer.destroy_node()
        if relay is not None:
            relay.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
