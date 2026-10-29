"""Container-side matrix for full ROS2 relay startup, after local rosidl build."""
from __future__ import annotations

import json
import subprocess
import sys

SAFE = "/valdi/lab_safe_cmd_vel_output"
PROBE = "/work/integration/probe_full_relay_startup_20261029.py"
CASES = (
    ("nominal", []),
    ("namespace", ["--ros-args", "-r", "__ns:=/robot"]),
    ("remap_motor", ["--ros-args", "-r", SAFE + ":=/cmd_vel"]),
    ("remap_controller", ["--ros-args", "-r",
                          SAFE + ":=/diff_drive_controller/cmd_vel_unstamped"]),
    ("remap_other", ["--ros-args", "-r", SAFE + ":=/another_output"]),
    ("simulation_off", []),
    ("override_motor", []),
)


def main() -> None:
    records = []
    for name, args in CASES:
        process = subprocess.run(
            [sys.executable, PROBE, "--case", name, *args],
            text=True, capture_output=True, timeout=20, check=False)
        output = next(
            (line.partition("FULL_RELAY_STARTUP ")[2]
             for line in process.stdout.splitlines()
             if line.startswith("FULL_RELAY_STARTUP ")), None)
        try:
            report = json.loads(output) if output else None
        except ValueError:
            report = None
        passed = (
            process.returncode == 0
            and isinstance(report, dict)
            and report.get("case") == name
            and report.get("result") == report.get("expected")
            and report.get("published_motor_twists") == 0
            and report.get("physical_motor_connected") is False
        )
        records.append({
            "case": name, "pass": passed, "return_code": process.returncode,
            "report": report, "error_tail": process.stderr[-500:],
        })
        print(json.dumps({"case": name, "pass": passed,
                          "report": report, "error_tail": process.stderr[-200:]},
                         sort_keys=True), flush=True)
    summary = {
        "scope": "real ROS2 Jazzy full experimental relay, networkless container",
        "pass": sum(bool(x["pass"]) for x in records),
        "total": len(records),
        "tests": records,
        "motor_hardware": False,
        "gazebo": False,
        "simulated_motion": False,
    }
    print("FULL_RELAY_MATRIX " + json.dumps(summary, sort_keys=True), flush=True)
    if summary["pass"] != summary["total"]:
        sys.exit(1)


if __name__ == "__main__":
    main()
