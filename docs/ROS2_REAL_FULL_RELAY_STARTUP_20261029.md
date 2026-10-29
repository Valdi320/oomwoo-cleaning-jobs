# Real ROS2 Jazzy full-relay startup and STOP-only matrix — 2026-10-29

Experimental Apache-2.0 contributor-owned work for isolated OOMWOO labs. Not upstream bringup, a reactive room-sweep executor, a physical motor watchdog or an e-stop. No production/hardware dependencies changed.

## Gap closed

The earlier ROS2 test checked only a standalone publisher remap. This test builds the custom rosidl interfaces in an ephemeral offline container and constructs the ACTUAL ExperimentalDockVelocityRelay node. It confirms that unsafe output remaps fail before the output timer is installed, and that normal startup emits only zero-valued Twist to the isolated lab topic.

| Case | Expectation | Real Jazzy result |
| --- | --- | --- |
| Nominal absolute lab topic | 3+ zero Twist outputs, no /cmd_vel publishers | PASS |
| Namespace change | Same safe STOP output | PASS |
| Remap lab output to /cmd_vel | Rejected before timer | PASS |
| Remap to diff_drive_controller motor topic | Rejected before timer | PASS |
| Remap to unrelated topic | Rejected before timer | PASS |
| simulation_lab false | Rejected before timer | PASS |
| Explicit output_topic=/cmd_vel | Rejected before timer | PASS |

Result: 7/7 PASS in a networkless Podman container with real ROS2 Jazzy middleware on x86_64 Bazzite. A colcon interface build and all tests together took 14.48 s under a 3-CPU, 2-GiB cgroup cap. This wall time is NOT runtime RAM or CPU usage, a SBC minimum, or a speedup claim. Full per-case observations: docs/evidence/real_full_relay_startup_20261029.json.

## Reproduce (no downloads, no connected hardware)

Requires local Podman and a previously cached docker.io/makerspet/oomwoo:jazzy-dev image. The runner refuses to pull, creates no persistent process, uses network=none, read-only host source mount and no added hardware devices.

~~~sh
python3 integration/run_real_full_relay_startup_20261029.py
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_full_relay_matrix_harness_20261029.py' -q
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py' -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -p 'test_*.py' -q
git diff --check
~~~

The container builds the local oomwoo_cleaning_jobs_lab_interfaces package only under /tmp. Individual cases use separate ROS2 processes. Negative cases patch the relay output-timer constructor to reject any attempted arm, then assert the observed constructor exception; positive cases observe multiple zero Twist messages via a real subscription.

## Review and limitations

- Verified full relay startup and STOP-only publication, not motion-source arbitration or a motor.
- No Gazebo, motor driver, MCU braking, Nav2 / dock action handoff, global graph exclusivity, or authenticated sensor provenance was tested in this iteration.
- A separate node outside the isolated lab could still bridge the lab topic to real motors. ROS2 topic containment never replaces physical MCU safety interlocks.
- No upstream PR #79 changes, no maintainer comments, no customer equipment touched.
- Existing real ROS2 publisher-only firewall tests and synthetic mux regressions remain independently scoped.

Checklist: [x] 7/7 real Jazzy cases; [x] full typed relay startup; [x] stop on normal idle; [x] remap/parameter rejection before output timer; [ ] real Nav2/dock takeover fault injection; [ ] Gazebo; [ ] hardware safety.

Integration: only the private contributor test branch is affected. To revert this milestone remove the three new integration full-relay scripts, harness test, this note and evidence. The original lab topic firewall stays unchanged.
