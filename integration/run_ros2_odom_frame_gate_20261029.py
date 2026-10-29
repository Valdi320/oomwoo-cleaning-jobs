"""Run real Jazzy stamped-odometry DDS probe in a networkless container.

Imports the established 7-case lab runner's immutable isolation arguments;
no image pull, no /cmd_vel publications, no robot devices.
"""
from __future__ import annotations
import datetime
import json
from pathlib import Path
import platform
import subprocess
import sys
import time

from run_real_full_relay_startup_20261029 import ARGS as BASE_ARGS

ROOT = Path(__file__).resolve().parents[1]
EVIDENCE = ROOT / "docs/evidence/ros2_odom_frame_gate_dds_20261029.json"
ARGS = list(BASE_ARGS)
OLD = "/work/integration/full_relay_matrix_container_20261029.py"
NEW = "/work/integration/probe_ros2_odom_frames_20261029.py"
assert ARGS[-1].count(OLD) == 1
ARGS[-1] = ARGS[-1].replace(OLD, NEW)


def main():
    start = time.monotonic()
    try:
        p = subprocess.run(ARGS,capture_output=True,text=True,
                           timeout=130,check=False)
    except (OSError,subprocess.TimeoutExpired) as exc:
        print(json.dumps({"pass":False,"error":str(exc)}))
        return 1
    marker = "ROS2_ODOM_FRAME_PROBE "
    lines = [x[len(marker):] for x in p.stdout.splitlines()
             if x.startswith(marker)]
    try:result = json.loads(lines[-1]) if len(lines)==1 else None
    except ValueError:result = None
    passed = (p.returncode==0 and isinstance(result,dict)
              and result.get("pass")==16 and result.get("total")==16)
    record={
        "timestamp_utc":datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "host_arch":platform.machine(),
        "image":"docker.io/makerspet/oomwoo:jazzy-dev",
        "scope":"real ROS2 Jazzy Odometry DDS frame binding; synthetic zero input; no motors",
        "cgroup_memory_limit":"2g","cgroup_cpu_limit":3,
        "elapsed_s":round(time.monotonic()-start,2),
        "container_exit_code":p.returncode,
        "pass":passed,
        "results":result,
        "error_tail":"" if passed else (p.stderr[-1500:]+p.stdout[-1500:]),
    }
    EVIDENCE.write_text(json.dumps(record,sort_keys=True,indent=2)+"\n")
    print(json.dumps({
        "pass":passed,"tests":result.get("pass") if isinstance(result,dict) else 0,
        "total":16,"seconds":record["elapsed_s"],
        "evidence":str(EVIDENCE),
        "error":record["error_tail"][:1400],
    },sort_keys=True))
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
