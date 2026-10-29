"""Run a ROS2 Jazzy publisher topic-remap matrix without any network/motors.

Requires already-cached local image docker.io/makerspet/oomwoo:jazzy-dev.
Never publishes a Twist. No image pulls, network, persistent processes,
hardware devices, or changes to the caller's ROS2 system.
"""
from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
IMAGE = "docker.io/makerspet/oomwoo:jazzy-dev"
SAFE = "/valdi/lab_safe_cmd_vel_output"
PROBE = "/work/integration/probe_lab_topic_remapping_20261029.py"

CASES = [
    ("nominal", "allowed", SAFE, ""),
    ("motor_topic", "rejected", "/cmd_vel",
     "--ros-args -r " + SAFE + ":=/cmd_vel"),
    ("diff_drive_controller", "rejected",
     "/diff_drive_controller/cmd_vel_unstamped",
     "--ros-args -r " + SAFE + ":=/diff_drive_controller/cmd_vel_unstamped"),
    ("other_velocity_topic", "rejected", "/valdi/other_velocity",
     "--ros-args -r " + SAFE + ":=/valdi/other_velocity"),
    ("namespace_only", "allowed", SAFE, "--ros-args -r __ns:=/robot"),
]


def main() -> int:
    base = [
        "podman", "run", "--rm", "--pull=never",
        "--network", "none", "--cap-drop", "ALL",
        "--security-opt", "no-new-privileges",
        # SELinux-only bind mount workaround, no image relabel.
        "--security-opt", "label=disable",
        "--memory", "768m", "--pids-limit", "128", "--read-only",
        "--tmpfs", "/tmp:rw,mode=1777",
        "-e", "HOME=/tmp", "-e", "ROS_LOG_DIR=/tmp/roslogs",
        "-e", "ROS_DOMAIN_ID=87", "-e", "PYTHONPATH=/work/src",
        "-v", str(ROOT) + ":/work:ro",
        "--entrypoint", "/bin/bash",
    ]
    success = True
    for label, expected_result, expected_topic, ros_args in CASES:
        command = (
            "source /opt/ros/jazzy/setup.bash && python3 " + PROBE
            + (" " + ros_args if ros_args else "")
        )
        argv = base + [
            "-e", "LAB_EXPECT=" + expected_result,
            "-e", "LAB_EXPECT_RESOLVED=" + expected_topic,
            IMAGE, "-lc", command,
        ]
        try:
            result = subprocess.run(
                argv, capture_output=True, text=True, timeout=40,
                check=False,
            )
            passed = result.returncode == 0
            record = {
                "case": label,
                "pass": passed,
                "stdout": result.stdout.strip()[-1500:],
                "stderr": result.stderr.strip()[-500:],
            }
        except (OSError, subprocess.TimeoutExpired) as exc:
            passed = False
            record = {"case": label, "pass": False, "error": str(exc)}
        print(json.dumps(record, sort_keys=True))
        success = success and passed
    print("ROS2 remap probe:", "PASS 5/5" if success else "FAIL")
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
