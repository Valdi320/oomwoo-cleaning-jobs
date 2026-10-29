# ROS2 stamped-odometry fail-closed gate — 2026-10-29

## Real defect

The experimental `ros_dock_velocity_relay` previously set `_odom_last = time.monotonic()` whenever an `Odometry` callback *arrived*, even when the ROS message's original `header.stamp` was old. Replaying delayed zero-velocity odometry could therefore create false "fresh quiet" evidence and eventually influence a Nav2-to-docking transition. This is a safety defect in a **lab-only experimental adapter**, not a discovered fault in upstream OOMWOO hardware.

## Correction

New `ros_odom_stamp_gate.OdomStampGate` checks ROS-clock age (max 350 ms), future lead (max 50 ms), strictly increasing header stamps and ROS-clock non-rewind, before updating the monotonic receipt time or mux odometry. Missing/zero/stale/repeated/future/reordered headers reset the relay to STOP and revoke previously fresh odometry. The last accepted stamp survives STOP; a simulator clock rewind requires a fresh process and explicit reconciliation. The two epochs, ROS time and monotonic local arrival time, are never directly subtracted.

This conservative gate may reject low-rate or unstamped Odometry configurations. It deliberately changes the contract: providers must supply trusted, continuously stamped messages. Header fields and graph names are **not authenticated** in ROS2; a malicious or buggy publisher can fabricate a fresh timestamp. The relay remains isolated to `/valdi/lab_safe_cmd_vel_output`; this is not an MCU watchdog, e-stop, physical validation or proof of motor isolation.

## Reproduce

From branch `contrib/lab-ros2-odom-stamp-gate-20261029-1547`:

~~~sh
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_ros_odom_stamp_gate_20261029.py' -q
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py' -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -p 'test_*.py' -q
python3 integration/run_ros2_stamped_odom_20261029.py
git diff --check
~~~

The ROS2 runner requires a previously cached `docker.io/makerspet/oomwoo:jazzy-dev` image. It compiles lab rosidl in disposable `/tmp`, runs with `--network=none --cap-drop=ALL --read-only --pull=never`, no devices, no nonzero `Twist`, and publishes only `nav_msgs/Odometry` on isolated `/odom`. Results are recorded in `docs/evidence/ros2_stamped_odom_dds_20261029.json`.

## Integration and rollback

All edits are on Valdi320's contributor-owned code branch, not upstream. The callback's ROS timestamp check is opt-in with this experimental node; existing synthetic mux semantics are unchanged. Revert the integration commit to restore the previous callback; do **not** enable its old behavior on hardware. Full Nav2 ownership handoff, Gazebo, measured braking, frame integrity, ROS2 source authentication, ARM performance and firmware watchdog remain open.

Maintainer notifications/PR #79 intentionally untouched. Apache-2.0.
