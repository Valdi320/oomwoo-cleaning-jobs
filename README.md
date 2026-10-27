# OOMWOO cleaning-jobs: checkpoints + tested ROS2/Nav2 action cancellation

[OOMWOO](https://github.com/makerspet/oomwoo) · [cleaning-jobs RFC](https://github.com/makerspet/oomwoo/tree/main/contributions/cleaning-jobs) · [software interface contract](https://github.com/makerspet/oomwoo/blob/main/docs/SOFTWARE_INTERFACES.md)

A **testable cleaning-job core plus a real ROS2 NavigateToPose ActionClient transport**, proposed for OOMWOO's cleaning-jobs RFC. The adapter was exercised against real Nav2 controlling OOMWOO in Gazebo; the project is **not a complete production ROS2 cleaning node or physical robot safety controller**.

## What works today

- Target sets from a saved occupancy grid: whole mapped floor, named/manual rooms, or a spot radius.
- Explicit cell-index no-go masks and configurable conservative square-cell clearance; unknown and occupied cells are not traversable. A deterministic four-connected BFS produces a route **only through allowed cell centres**, or fails if goals are unreachable.
- Durable coverage checkpoints: only **acknowledged** cells count. Checkpoint writes are atomic + fsync; after process restart an interrupted running job becomes `paused_recovery` until a fresh pose and healthy resources are provided.
- `start`, `waypoint`, `acknowledge_cleaned`, `pause`, `resume`, `cancel`, `observe_resources`, `fail`, `status`; battery/bin/mop service interruptions; priority for low battery; clean resumption without erasing earlier progress.
- Strict fail-closed conditions for missing/unreachable start pose, keepout violations, stale changed maps, corrupted checkpoints, service still needed, and invalid state transitions.
- No runtime Python dependencies. Testable without ROS 2 or Gazebo; standard-library unittest + deterministic demonstration.
- **Phase-1 Nav2-facing interlock:** only one outstanding navigation goal; on pause, recovery or low battery request cancellation and wait for a terminal result before allowing another goal.
- An uncertain send/cancel or timeout **latches a fault**, requiring external Nav2 idle verification; late results and stale coverage acknowledgements cannot silently rearm the robot.
- **Manual/externally verified rearm after restart** (goal handles may be lost even while Nav2 continues moving). See [navigation handoff contract](docs/NAVIGATION_HANDOFF.md).
- **92 pure-Python regression tests**, including real-action callback-future fault injection, plus **one real ROS2 action-server roundtrip and three real OOMWOO Nav2/Gazebo cancellation scenarios**. See [measured validation](docs/ROS2_GAZEBO_VALIDATION.md) and [machine evidence](docs/evidence/gazebo_nav2_20261027.json).

## What it deliberately does **not** do

- **No physical motors or hardware safety; no persistent ROS2 cleaning-job action/service node yet.** A genuine Nav2 ActionClient in our isolated Gazebo test can send/cancel actions, but this cannot certify safe operation on a physical robot. Hard cliff/bumper/wheel-drop/current safety belongs independently to the STM32.
- The chosen paths represent *grid cell centres* and do not prove a robot with finite footprint, wheel slip, localization noise, dynamic obstacles or a specific nozzle cleaning width can traverse or clean them. Real execution must use Nav2 costmaps with correctly inflated keepout, robust localization, collision checks, recovery and progress feedback.
- Rooms are **manually labelled**; automatic room segmentation and editing UI are future work. Occupancy `0` is considered traversable; `-1` (unknown) and all other values are blocked intentionally.
- Coverage is **acknowledged cell-centre progress**, *not* proof that physical floor area or the cleaning swath was cleaned. The integration adapter must transform trusted coverage updates into cell IDs and persist them.
- Resource values and pose are supplied by the caller; no battery/dock services are simulated. Service resolution requires external dock-cycle integration.
- One job/checkpoint and one writer per instance; new job needs a fresh checkpoint. Cross-process coordination should happen at the ROS adapter layer.

## Run, verify

Requires Python 3.10+.

```sh
PYTHONPATH=src python -m unittest discover -s tests -v
PYTHONPATH=src python -m oomwoo_cleaning_jobs.demo
```

Both commands run in plain Linux without ROS2, Docker, network access or robot hardware. The demo simulates an interrupted cleaning job on a two-room grid, reopens the checkpoint, and finishes at 100% of **eligible centre-cell targets**. CI runs Python 3.10 and 3.12.

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

**Cancellation of an existing Nav2 action has been demonstrated in OOMWOO's test-room Gazebo simulation.** The bridge submits cancellation and gates new actions until a terminal result, while simulator odometry independently checks that motion settles. This is not an MCU emergency-stop implementation or proof of safe physical stopping. A future persistent ROS2 service/action node must arbitrate motion with `recovery-safety`, keep the inflated Nav2 costmap active, and implement backend reconciliation after process restart.

## Phase-0 acceptance checks

- Repeatable whole/room/spot selection, disconnected and forbidden-cell rejection.
- Interruptions, manual pause, stale resource gates, recovery-after-restart and map mismatch.
- No double-counting after replay; invalid acknowledgments never mutate checkpoints; cancellation and terminal states are irreversible.
- 100% coverage of eligible cell-centre goals in the headless example, including a mid-job low-battery interruption and re-open.

## Next PRs needed to meet the entire RFC

1. Package the tested ROS2 ActionClient transport in a persistent ROS2 `cleaning-jobs` node and integrate **real measured** `clean-and-map` coverage events. Agree the public action/service contract with the maintainer first.
2. Nav2 KeepoutFilter mask + inflation tests with robot footprint, dynamic obstacles, cliffs and recovery. Run headless Gazebo regressions on `living_room` and `multi_room`.
3. Automatic room segmentation, persisted human room labels, spot editing and zone UI; integrate dock-cycle recharge/bin/mop interruptions. Assess swath coverage and localization error, not only cell-centre completion.
4. Evaluate algorithmic efficiency against upstream's coverage planner; nearest-cell routing here optimizes correctness of a small reference, **not** cleaning time or energy.

## Contribution and AI disclosure

Authored with ChatGPT assistance, exercised on the contributor's Linux PC using deterministic tests and the official OOMWOO Gazebo/Nav2 runtime. Published independently per the OOMWOO contribution guide; the maintainer decides whether to feature or adapt it. Apache-2.0; the project keeps its own code and license terms.
