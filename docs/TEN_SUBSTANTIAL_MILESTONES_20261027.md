# Ten substantial OOMWOO cleaning-jobs milestones (27 October 2026)

This is an independent Apache-2.0 OOMWOO contribution. The previous
PR #78 was merged; PR #79 is the follow-up pointer.
The milestone tests use different reality levels. Do not equate
synthetic model tests with real Nav2/Gazebo movement or physical hardware.

## Ten delivered work packages

| # | What was built | Verified result / boundary |
| --- | --- | --- |
| 1 | Reuse the ORIGINAL OOMWOO boustrophedon planner without activating its action executor; conservative waypoint adapter and one Nav2 goal owner | REAL Nav2/Gazebo: 409 upstream source waypoints, 391 adapted distinct safe targets, five rejected original points, five initial REAL goals and 337 externally measured new cells |
| 2 | Durable, source-map-bound multi-step SweepCampaign with bounded goal reservations, retry history, progress and interrupted-job recovery | HEADLESS unit tests, fault injection, persisted checkpoints; not yet the installed ROS2 full-room executor |
| 3 | GapAuditor clusters uncleaned cells, distinguishes safe vs unreachable and proposes fresh costmap-admitted repair | HEADLESS tests incl. real no-go geometry in maps; long-room Nav2 gap-filling not proven |
| 4 | Durable DockCycle orchestration: recharge, empty dustbin and mop wash with nonce, independent station telemetry, two stable confirmations and restart hold | HEADLESS tests; one synthetic station pause inside REAL 6-goal Nav2 run, but no real docking/charging hardware |
| 5 | Selected-room/whole-home MultiRoomProgram with ordered named rooms, exact per-room scopes, no false other-room credit and saved progress | HEADLESS integration and reboots, including 3 combined station services |
| 6 | Twelve seeded three-room end-to-end virtual home scenarios with synthetic disk coverage, 3 service cycles, restart and prohibited zones | SYNTHETIC kinematic robot, 12/12 jobs completed 100% eligible cells, zero keepout entries; NOT twelve Gazebo tests |
| 7 | Versioned transactional live-zone updates with staged masks, independent fresh global/local post-filter costmap observations and map<-odom TF safety gates | HEADLESS PASS and REAL Nav2/Gazebo PASS (later cold boot): old 220-cell region replaced by new 198-cell region, two distinct costmap confirmations, no robot movement during edit. Nav2 planner subsequently respected the replacement |
| 8 | Private, typed ROSIDL StartCleaning / ControlCleaning / GetCleaningStatus service package; bounded request-ID deduplication and conflicting request rejection | Package built offline. REAL ROS2 services with test action server PASS: 2 accepted, 2 canceled, duplicate request produced zero additional goals. Experimental spot-only API |
| 9 | Longer REAL OOMWOO Gazebo upstream waypoint execution with original independent measured ground-truth coverage, service interrupt and resume | REAL Nav2 after station-freshness hardening: 6/6 goals succeeded, 348 external new cells; 246 preserved across SYNTHETIC recharge/bin/mop service after goal 3. Partial sweep, NOT complete room |
| 10 | Fsync CoverageWAL to synchronize externally verified cell events across CleaningJobs and SweepCampaign durable checkpoints, with crash-replay and idempotent event IDs | HEADLESS tests and combined multi-room fault-injection; not yet connected to long-running live ROS2 node |

## Source evidence

- REAL Nav2 first five imported goals: evidence/oomwoo_upstream_sweep_5goals_20261027.json
- REAL Nav2 six goals with synthetic station: evidence/upstream_sweep_6goals_station_20261027.json
- Twelve SYNTHETIC home scenarios: evidence/synthetic_multihome_12cases_20261027.json
- REAL ROS2 typed services with FAKE Nav2 server: evidence/ros2_typed_lab_api_20261027.json
- VERIFIED later REAL hot keepout: evidence/dynamic_keepout_hot_reload_verified_20261027.json
- REAL post-hot-edit planner: evidence/post_hot_keepout_nav2_planner_20261027.json
- Historical failed hot edit: evidence/dynamic_keepout_hot_reload_blocked_20261027.json
- Repeated six-goal test after station freshness hardening: evidence/upstream_sweep_6goals_hardened_20261027.json
- Earlier REAL Gazebo two-node crash: evidence/persistent_node_crash_recovery_20261027.json

## Reproduce

From this repository root, no network and no ROS2 needed for pure Python:

    PYTHONPATH=src python -m unittest discover -s tests -q
    PYTHONPATH=src python -m oomwoo_cleaning_jobs.demo
    PYTHONPATH=src python -m oomwoo_cleaning_jobs.synthetic_home --seed 0

The typed test requires the OOMWOO Jazzy container, a separately built
ros2_lab_interfaces package, and an isolated ROS_DOMAIN_ID. Build with
colcon build --packages-select oomwoo_cleaning_jobs_lab_interfaces,
source the generated install/setup.bash, then run
integration/ros_typed_service_probe.py inside ROS2. Its action server is
a test double, not Gazebo robot motion.

For REAL Gazebo/Nav2 reproduction and source provenance consult
docs/UPSTREAM_SWEEP_SINGLE_NAV2_OWNER.md. Keep other Nav2 action
executors disabled to ensure only one component drives the robot.

## Unresolved issues before 100%

A later cold-boot REAL Nav2 test **did** verify a live mask
replacement, two distinct global/local post-filter observations and
subsequent planner response to the changed forbidden region.
The stock OOMWOO bringup is STILL NOT changed; the successful test
used an isolated Nav2 filter overlay. No physical motor move took
place during the edit and mid-motion changes remain untested.
Detailed protocol: docs/REAL_NAV2_DYNAMIC_KEEPOUT_20261027.md.

The long-running experimental ROS2 node still needs full-room and
multi-room autonomous queues using the original planner, maintainer
approval of an official job service/action API, a live dock-cycle
provider, real sensor/actuator-gated coverage, source-hardened WAL
admission, and large multi-room Gazebo endurance and forced-restart
regressions. Multi-client Nav2 ownership must be coordinated.

Hardware/MCU cliff, bumper, wheel-drop, motor safety, suction
actuation, actual charging and real physical dirt cleaning are NOT
tested. The 12 synthetic homes and six real Nav2 simulation goals
are distinct tests and do not prove 100% of the real robot.


## Subsequent REAL ROS2/Gazebo work after this ten-milestone snapshot

The opt-in persistent ROS2 node now completes **TWO selected digital
regions within one long-running job** using original OOMWOO
CoveragePlanner geometry, actual Nav2 NavigateToPose and independently
measured ground-truth coverage. Result: 80/80 + 40/40 verified cells,
18 accepted Nav2 goals, matching completed durable checkpoints and
zero pending WAL events. Earlier 117/120 and timeout runs remain
separate historical failures. These are open-floor virtual zones,
not walled apartment rooms or physical dust collection.

The new experimental command-source relay also passed actual ROS2
middleware tests with SYNTHETIC Nav2/dock status, odometry and battery:
one output, exact journaled Nav2 UUID lease, terminal-bound handoff,
unknown-goal/low-resource revocation and zero output for stale/empty
status. Its output remains unconnected to robot motors. The unmodified
upstream OOMWOO dock 2D harness separately passed 7/7 simulated
geometric approaches. No electrical charging/bin/mop hardware proof.

[Complete two-zone and dock-relay report](REAL_TWO_ZONES_AND_DOCK_RELAY_20261027.md).
