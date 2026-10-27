# OOMWOO cleaning-jobs: checkpoints + persistent experimental ROS2/Nav2 controller

[OOMWOO](https://github.com/makerspet/oomwoo) · [cleaning-jobs RFC](https://github.com/makerspet/oomwoo/tree/main/contributions/cleaning-jobs) · [software interface contract](https://github.com/makerspet/oomwoo/blob/main/docs/SOFTWARE_INTERFACES.md)

A **testable job-state core, asynchronous Nav2 ActionClient, and long-running experimental ROS2 controller** for OOMWOO. Start, pause, resume, cancel and simulated low-battery interruption have now been tested end-to-end with the real Nav2 BT Navigator moving OOMWOO in Gazebo. This is **not a canonical OOMWOO public job API, completed cleaning-jobs RFC, or physical-robot safety controller**.

## What works today

- Target sets from a saved occupancy grid: whole mapped floor, named/manual rooms, or a spot radius.
- Explicit cell-index no-go masks and configurable conservative square-cell clearance; unknown and occupied cells are not traversable. A deterministic four-connected BFS produces a route **only through allowed cell centres**, or fails if goals are unreachable.
- Durable coverage checkpoints: only **acknowledged** cells count. Checkpoint writes are atomic + fsync; after process restart an interrupted running job becomes `paused_recovery` until a fresh pose and healthy resources are provided.
- `start`, `waypoint`, `acknowledge_cleaned`, `pause`, `resume`, `cancel`, `observe_resources`, `fail`, `status`; battery/bin/mop service interruptions; priority for low battery; clean resumption without erasing earlier progress.
- Strict fail-closed conditions for missing/unreachable start pose, keepout violations, stale changed maps, corrupted checkpoints, service still needed, and invalid state transitions.
- The job-state and interlock core remain dependency-free; the optional experimental ROS2 controller requires ROS2 Jazzy, Nav2 and TF2 in OOMWOO's official development image.
- **Phase-1 Nav2-facing interlock:** only one outstanding navigation goal; on pause, recovery or low battery request cancellation and wait for a terminal result before allowing another goal.
- An uncertain send/cancel or timeout **latches a fault**, requiring external Nav2 idle verification; late results and stale coverage acknowledgements cannot silently rearm the robot.
- **Manual/externally verified rearm after restart** (goal handles may be lost even while Nav2 continues moving). See [navigation handoff contract](docs/NAVIGATION_HANDOFF.md).
- **104 pure-Python regression tests** and ROS2 middleware probes. **New Phase 3:** a private, persistent ROS2 node using actual /map, map-to-base_link TF, Nav2 global costmap, lifecycle GetState, /battery_state and NavigateToPose. A [Gazebo end-to-end regression](docs/EXPERIMENTAL_ROS_NODE.md) confirms start/pause/resume/cancel and *automatic battery-low cancellation*, with [machine evidence](docs/evidence/ros_persistent_gazebo_20261027.json).

## What it deliberately does **not** do

- **No physical motor or MCU safety validation; no finalized public ROS2 job action/service API yet.** The long-running controller uses private simulation-only JSON topics, not official OOMWOO interfaces. The tested Gazebo cancel cannot certify stopping on a physical robot. Cliff/bumper/wheel-drop/current safety must remain independently STM32-owned.
- The chosen paths represent *grid cell centres* and do not prove a robot with finite footprint, wheel slip, localization noise, dynamic obstacles or a specific nozzle cleaning width can traverse or clean them. Real execution must use Nav2 costmaps with correctly inflated keepout, robust localization, collision checks, recovery and progress feedback.
- Rooms are **manually labelled**; automatic room segmentation and editing UI are future work. Occupancy `0` is considered traversable; `-1` (unknown) and all other values are blocked intentionally.
- Coverage is **acknowledged cell-centre progress**, *not* proof that physical floor area or the cleaning swath was cleaned. The integration adapter must transform trusted coverage updates into cell IDs and persist them.
- The experimental ROS2 node subscribes to real /battery_state and fresh TF in its private test. The official Gazebo test-room has no functional battery publisher, so the harness explicitly injects **synthetic BatteryState telemetry** to test a battery-low pause/recharge. Actual dock services remain unimplemented.
- One job/checkpoint and one writer per instance; new job needs a fresh checkpoint. Cross-process coordination should happen at the ROS adapter layer.

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

1. Promote the **tested experimental long-running ROS2 controller** to a maintainer-approved public job action/service API; integrate **trusted measured** clean-and-map swept-area feedback, without granting cleaning credit for mere Nav2 arrival.
2. Nav2 KeepoutFilter mask + inflation tests with robot footprint, dynamic obstacles, cliffs and recovery. Run headless Gazebo regressions on `living_room` and `multi_room`.
3. Automatic room segmentation, persisted human room labels, spot editing and zone UI; integrate dock-cycle recharge/bin/mop interruptions. Assess swath coverage and localization error, not only cell-centre completion.
4. Evaluate algorithmic efficiency against upstream's coverage planner; nearest-cell routing here optimizes correctness of a small reference, **not** cleaning time or energy.

## Contribution and AI disclosure

Authored with ChatGPT assistance, exercised on the contributor's Linux PC using deterministic tests and the official OOMWOO Gazebo/Nav2 runtime. Published independently per the OOMWOO contribution guide; the maintainer decides whether to feature or adapt it. Apache-2.0; the project keeps its own code and license terms.
