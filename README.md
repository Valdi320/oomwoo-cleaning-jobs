# OOMWOO cleaning-jobs: measured coverage and bounded Nav2 orchestration

[OOMWOO](https://github.com/makerspet/oomwoo) · [cleaning-jobs RFC](https://github.com/makerspet/oomwoo/tree/main/contributions/cleaning-jobs) · [software interface contract](https://github.com/makerspet/oomwoo/blob/main/docs/SOFTWARE_INTERFACES.md)

An experimental ROS2 cleaning-job core with durable checkpoints, real Nav2 actions, simulator-measured floor coverage and bounded automatic follow-up goals. Start, pause, resume and low-battery recovery have been tested with OOMWOO in Gazebo. This is **not a public canonical API, complete multi-room cleaner or physical safety controller**.

## What works today

- Target sets from a saved occupancy grid: whole mapped floor, named/manual rooms, or a spot radius.
- Explicit cell-index no-go masks and configurable conservative square-cell clearance; unknown and occupied cells are not traversable. A deterministic four-connected BFS produces a route **only through allowed cell centres**, or fails if goals are unreachable.
- Durable coverage checkpoints: only **acknowledged** cells count. Checkpoint writes are atomic + fsync; after process restart an interrupted running job becomes `paused_recovery` until a fresh pose and healthy resources are provided.
- `start`, `waypoint`, `acknowledge_cleaned`, `pause`, `resume`, `cancel`, `observe_resources`, `fail`, `status`; battery/bin/mop service interruptions; priority for low battery; clean resumption without erasing earlier progress.
- Strict fail-closed conditions for missing/unreachable start pose, keepout violations, stale changed maps, corrupted checkpoints, service still needed, and invalid state transitions.
- The job-state and interlock core remain dependency-free; the optional experimental ROS2 controller requires ROS2 Jazzy, Nav2 and TF2 in OOMWOO's official development image.
- **Measured coverage:** only fresh cells from the official OOMWOO simulated ground-truth meter count, never mere Nav2 success. A bounded 3-second post-success meter tail captures delayed measurements without unlocking future navigation. See [measured coverage](docs/MEASURED_COVERAGE_AND_AUTO_CONTINUE.md).
- **Offline saved-map room segmentation:** deterministic bottleneck-based room proposals, editable/persistent labels, source-map fingerprints and fail-closed updates. [Room partition/restart evidence](docs/ROOM_PARTITION_AND_RESTART.md).
- **Prohibited zones now tested with REAL Nav2:** saved-map polygons and walls produce a 0/100 mask. An isolated Nav2 Jazzy variant activates KeepoutFilter in **both global and local costmaps**, computes a detour, and steers OOMWOO around a forbidden rectangle in Gazebo. The private cleaning-jobs node optionally requires verified live filters before accepting a goal. [Measurements and exact reproduction](docs/REAL_NAV2_KEEPOUT_GAZEBO.md). This has **not** been merged into OOMWOO's default navigation launch.
- **Official upstream boustrophedon sweep geometry reused with ONE Nav2 owner:** exported the unmodified planner's **409 waypoints** without spinning its own executor. A conservative plan adapter finds **391 distinct admissible endpoints** under our five-cell clearance (5 original waypoints remain unresolved and are NOT sent). A bounded real Nav2/Gazebo test executed **5 upstream waypoints and measured 337 newly swept cells** from the independent official coverage meter. This is a partial sweep, not an entire room. [Exact protocol and limitations](docs/UPSTREAM_SWEEP_SINGLE_NAV2_OWNER.md).
- **Phase-1 Nav2-facing interlock:** only one outstanding navigation goal; on pause, recovery or low battery request cancellation and wait for a terminal result before allowing another goal.
- An uncertain send/cancel or timeout **latches a fault**, requiring external Nav2 idle verification; late results and stale coverage acknowledgements cannot silently rearm the robot.
- **Real process-crash recovery now tested in Gazebo:** a durable Nav2 goal-UUID journal survives the loss of a controller. A new long-running ROS2 controller remains DISARMED, performs UUID-specific cancellation, requires the terminal canceled status and advancing quiet simulator odometry, restores the checkpoint and allows an **explicit** restart. [Full two-process trial and failure limits](docs/DURABLE_NAV2_CRASH_RECOVERY.md). This does NOT certify real hardware or cover unknown UUIDs.
- **255 pure-Python regression tests** and ROS2 middleware probes. **New Phase 3:** a private, persistent ROS2 node using actual /map, map-to-base_link TF, Nav2 global costmap, lifecycle GetState, /battery_state and NavigateToPose. A [Gazebo end-to-end regression](docs/EXPERIMENTAL_ROS_NODE.md) confirms start/pause/resume/cancel and *automatic battery-low cancellation*, with [machine evidence](docs/evidence/ros_persistent_gazebo_20261027.json).

### Repeatable measured spot completion

**3/3 independent cold-start Gazebo trials** finished the same small
9-cell test-room spot with **9/9 externally measured cells**, two
Nav2 goals and a confirmed terminal cancellation in every run.
[Repetition evidence](docs/evidence/ros_verified_small_spot_repeats_20261027.json)
and [root-cause analysis](docs/SMALL_SPOT_MEASURED_COMPLETION.md)
document the route overshoot and delayed-meter fix. This does **not**
establish multi-room or whole-floor performance.

## What it deliberately does **not** do

- **No physical motor or MCU safety validation; no finalized public ROS2 job action/service API yet.** The long-running controller uses private simulation-only JSON topics, not official OOMWOO interfaces. The tested Gazebo cancel cannot certify stopping on a physical robot. Cliff/bumper/wheel-drop/current safety must remain independently STM32-owned.
- The chosen paths represent *grid cell centres* and do not prove a robot with finite footprint, wheel slip, localization noise, dynamic obstacles or a specific nozzle cleaning width can traverse or clean them. Real execution must use Nav2 costmaps with correctly inflated keepout, robust localization, collision checks, recovery and progress feedback.
- Room partitioning now proposes deterministic areas from saved maps and persists human labels, but **does not guarantee semantic room recognition** or provide an editing UI. The ROS2 controller still runs small spot jobs only. Occupancy `0` is safe in this conservative model; other and unknown values are blocked.
- The existing upstream full-room plan has been validated only through a **bounded standalone action/coverage trial**. Its full 409-waypoint room executor is NOT integrated into the long-running ROS2 job node, and several upstream waypoints fail our strict clearance policy.
- Coverage can be acknowledged from OOMWOO's external ground-truth simulated cleaning-disk meter, but this is **not physical dirt removal**. Hardware requires a separately validated estimator and actuator-state gating.
- The experimental ROS2 node subscribes to real /battery_state and fresh TF in its private test. The official Gazebo test-room has no functional battery publisher, so the harness explicitly injects **synthetic BatteryState telemetry** to test a battery-low pause/recharge. Actual dock services remain unimplemented.
- One job/checkpoint and one writer per instance; new job needs a fresh checkpoint. A UUID journal persists the last Nav2 action before sending it. If the controller dies, no new goal can be sent until that UUID is reconciled. A multi-client robot still needs independent Nav2 action arbitration.

## Run, verify

Requires Python 3.10+.

```sh
PYTHONPATH=src python -m unittest discover -s tests -v
PYTHONPATH=src python -m oomwoo_cleaning_jobs.demo
```

Both commands run in plain Linux without ROS2, Docker, network access or hardware. The demo completes eligible **cell-centre** targets only; it does not measure physical cleaning. CI covers Python 3.10 and 3.12. The separate real ROS2/Gazebo workflow is documented in [EXPERIMENTAL_ROS_NODE.md](docs/EXPERIMENTAL_ROS_NODE.md).

## Integration boundary (tested Nav2 transport; full production node pending)

| In this reference | Source / sink in real integration | Responsibility |
| --- | --- | --- |
| `Grid` + `room` labels | `/map` (`nav_msgs/OccupancyGrid`), zone UI | Convert grid/costmap safely; verify map fingerprint on restore |
| `CleaningJobs.start()` | Action/service from UI / Home Assistant | Validate scope and start pose; do not auto-arm hardware |
| `waypoint(pose_cell)` | Real `NavigateToPose` ActionClient via `AsyncNav2Transport` | Goal submit/cancel/terminal callbacks tested; **advisory cell-centre targets** still need real footprint-aware costmap admission and arbitration |
| `acknowledge_cleaned()` | Verified coverage progress from `clean-and-map` | Only mark cells actually covered; never infer success from goal acceptance |
| `observe_resources()` | `/battery_state` + bin/mop status | In the tested Nav2 interlock, battery-low pauses an active job and **requests cancel of its outstanding Nav2 action**; actual dock orchestration remains external |
| `resume(pose_cell=..., resources=...)` | Localization + dock-cycle + user control | Require fresh localized pose and resolution of interruptions |
| `status()` | Final OOMWOO status/action API (not yet specified) | Machine-readable state and reason; no new project-wide ROS2 topic invented |

**Long-running experimental ROS2 integration is now validated in OOMWOO's test-room Gazebo simulation.** The node checks a live Nav2 costmap, refuses to arm before BT Navigator is lifecycle-ACTIVE, issues at most one active action and gates dispatch until terminal cancel confirmation. A future **public** ROS2 job service/action must still be agreed with maintainers, and crash-time goal reconciliation and independent MCU hard safety remain essential.

## Phase-0 acceptance checks

- Repeatable whole/room/spot selection, disconnected and forbidden-cell rejection.
- Interruptions, manual pause, stale resource gates, recovery-after-restart and map mismatch.
- No double-counting after replay; invalid acknowledgments never mutate checkpoints; cancellation and terminal states are irreversible.
- 100% coverage of eligible cell-centre goals in the headless example, including a mid-job low-battery interruption and re-open.

## Next PRs needed to meet the entire RFC

1. Promote the private experimental controller to a maintainer-approved job API; replace simulator ground truth with hardware-validated coverage estimation and actuator-state gating.
2. Adopt the **tested private KeepoutFilter config** in maintainer-approved OOMWOO bringup, with mask edits and hot-replanning tests; integrate upstream `oomwoo_coverage` full-room sweep with a single Nav2 action owner.
3. Validate room proposals on furnished multi-room maps, add a zone/room editing UI and make the private ROS2 controller schedule complete selected rooms; integrate dock-cycle recharge/bin/mop interruptions and test actual swept-area completeness.
4. Evaluate algorithmic efficiency against upstream's coverage planner; nearest-cell routing here optimizes correctness of a small reference, **not** cleaning time or energy.

## Contribution and AI disclosure

Authored with ChatGPT assistance, exercised on the contributor's Linux PC using deterministic tests and the official OOMWOO Gazebo/Nav2 runtime. Published independently per the OOMWOO contribution guide; the maintainer decides whether to feature or adapt it. Apache-2.0; the project keeps its own code and license terms.
