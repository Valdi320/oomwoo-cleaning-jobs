"""Run full ROS2 startup matrix in a disposable, offline and unprivileged lab.

No network, image pull, hardware device, motor topic output, persistent process
or upstream change. Requires locally cached makerspet/oomwoo:jazzy-dev image.
"""
from __future__ import annotations

import datetime
import json
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
IMAGE = "docker.io/makerspet/oomwoo:jazzy-dev"
EVIDENCE = ROOT / "docs/evidence/real_full_relay_startup_20261029.json"
SHELL = (
    "set -eo pipefail; "
    "source /opt/ros/jazzy/setup.bash; "
    "mkdir -p /tmp/valdi_ros_ws/src; "
    "cp -r /work/ros2_lab_interfaces "
    "/tmp/valdi_ros_ws/src/oomwoo_cleaning_jobs_lab_interfaces; "
    "cd /tmp/valdi_ros_ws; "
    "colcon build --packages-select oomwoo_cleaning_jobs_lab_interfaces "
    "--parallel-workers 2 --event-handlers console_direct+ "
    "> /tmp/rosidl_build.log 2>&1 "
    "|| { tail -n 70 /tmp/rosidl_build.log; exit 3; }; "
    "source /tmp/valdi_ros_ws/install/setup.bash; "
    "PYTHONPATH=/work/src:${PYTHONPATH:-} "
    "python3 /work/integration/full_relay_matrix_container_20261029.py"
)
ARGS = [
    "podman", "run", "--rm", "--pull=never",
    "--network=none", "--cap-drop=ALL",
    "--security-opt=no-new-privileges",
    "--security-opt=label=disable",
    "--read-only",
    "--tmpfs", "/tmp:rw,exec,size=1024m,mode=1777",
    "--memory=2g", "--cpus=3", "--pids-limit=180",
    "-e", "HOME=/tmp", "-e", "ROS_LOG_DIR=/tmp/roslogs",
    "-e", "ROS_DOMAIN_ID=88", "-e", "ROS_LOCALHOST_ONLY=1",
    "-v", str(ROOT) + ":/work:ro",
    "--entrypoint", "/bin/bash", IMAGE, "-lc", SHELL,
]


def main() -> int:
    start = time.monotonic()
    try:
        result = subprocess.run(
            ARGS, capture_output=True, text=True, timeout=130, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        print(json.dumps({"error": str(exc), "pass": False}))
        return 1
    marker = "FULL_RELAY_MATRIX "
    line = next(
        (s[len(marker):] for s in result.stdout.splitlines()
         if s.startswith(marker)), None)
    try:
        summary = json.loads(line) if line else None
    except ValueError:
        summary = None
    valid = (
        result.returncode == 0
        and isinstance(summary, dict)
        and summary.get("total") == 7
        and summary.get("pass") == 7
    )
    evidence = {
        "timestamp_utc": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "host_arch": __import__("platform").machine(),
        "container_image": IMAGE,
        "offline_no_motor_output": True,
        "cgroup_memory_limit": "2g",
        "cgroup_cpus_limit": 3,
        "process_seconds": round(time.monotonic() - start, 2),
        "container_return_code": result.returncode,
        "pass": valid,
        "ros2_results": summary,
        "error_tail": result.stderr[-1500:] if not valid else "",
        "stdout_tail": result.stdout[-3000:] if not valid else "",
    }
    EVIDENCE.write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n")
    print(json.dumps({
        "pass": valid,
        "passed": summary.get("pass") if isinstance(summary, dict) else 0,
        "total": summary.get("total") if isinstance(summary, dict) else 7,
        "evidence": str(EVIDENCE),
        "seconds": evidence["process_seconds"],
        "error_tail": result.stderr[-700:] if not valid else "",
        "stdout_tail": result.stdout[-1400:] if not valid else "",
    }, sort_keys=True))
    return 0 if valid else 1


if __name__ == "__main__":
    sys.exit(main())
