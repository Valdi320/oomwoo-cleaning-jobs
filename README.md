# OOMWOO cleaning-jobs: Phase-0 headless reference

[OOMWOO](https://github.com/makerspet/oomwoo) · [cleaning-jobs RFC](https://github.com/makerspet/oomwoo/tree/main/contributions/cleaning-jobs) · [software interface contract](https://github.com/makerspet/oomwoo/blob/main/docs/SOFTWARE_INTERFACES.md)

A minimal **testable, hardware-independent job-state core**, proposed as one building block for OOMWOO's currently-unimplemented `cleaning-jobs` RFC. This is an *early prototype*, not a finished ROS 2 package and **not an integrated robot controller**.

## What works today

- Target sets from a saved occupancy grid: whole mapped floor, named/manual rooms, or a spot radius.
- Explicit cell-index no-go masks and configurable conservative square-cell clearance; unknown and occupied cells are not traversable. A deterministic four-connected BFS produces a route **only through allowed cell centres**, or fails if goals are unreachable.
- Durable coverage checkpoints: only **acknowledged** cells count. Checkpoint writes are atomic + fsync; after process restart an interrupted running job becomes `paused_recovery` until a fresh pose and healthy resources are provided.
- `start`, `waypoint`, `acknowledge_cleaned`, `pause`, `resume`, `cancel`, `observe_resources`, `fail`, `status`; battery/bin/mop service interruptions; priority for low battery; clean resumption without erasing earlier progress.
- Strict fail-closed conditions for missing/unreachable start pose, keepout violations, stale changed maps, corrupted checkpoints, service still needed, and invalid state transitions.
- No runtime Python dependencies. Testable without ROS 2 or Gazebo; standard-library unittest + deterministic demonstration.

## What it deliberately does **not** do

- **No motors, no hardware safety, no Nav2/Gazebo integration, no ROS2 action/service yet.** Never rely on this proof-of-concept to safely drive hardware. Hard cliff/bumper/wheel-drop/current safety belongs to the STM32 independently.
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

## Integration boundary (future ROS2 adapter)

| In this reference | Source / sink in real integration | Responsibility |
| --- | --- | --- |
| `Grid` + `room` labels | `/map` (`nav_msgs/OccupancyGrid`), zone UI | Convert grid/costmap safely; verify map fingerprint on restore |
| `CleaningJobs.start()` | Action/service from UI / Home Assistant | Validate scope and start pose; do not auto-arm hardware |
| `waypoint(pose_cell)` | Nav2 `/navigate_to_pose` or coverage planning backend | **Advisory cell-centre route only**; enforce inflated Nav2 keepout and arbitration |
| `acknowledge_cleaned()` | Verified coverage progress from `clean-and-map` | Only mark cells actually covered; never infer success from goal acceptance |
| `observe_resources()` | `/battery_state` + bin/mop status | Stop issuing *new* paths; external adapter cancels active goals and docks |
| `resume(pose_cell=..., resources=...)` | Localization + dock-cycle + user control | Require fresh localized pose and resolution of interruptions |
| `status()` | Final OOMWOO status/action API (not yet specified) | Machine-readable state and reason; no new project-wide ROS2 topic invented |

**Stopping an already active robot trajectory is NOT implemented here.** The ROS2 integration must cancel outstanding Nav2 goals and arbitrate motion with `recovery-safety`; soft e-stop is not a substitute for MCU hardware safety. On low battery this core simply refuses new waypoints. This distinction is intentional to prevent a test harness from being mistaken for a safety system.

## Phase-0 acceptance checks

- Repeatable whole/room/spot selection, disconnected and forbidden-cell rejection.
- Interruptions, manual pause, stale resource gates, recovery-after-restart and map mismatch.
- No double-counting after replay; invalid acknowledgments never mutate checkpoints; cancellation and terminal states are irreversible.
- 100% coverage of eligible cell-centre goals in the headless example, including a mid-job low-battery interruption and re-open.

## Next PRs needed to meet the entire RFC

1. Run inside OOMWOO's `jazzy-dev` Docker image with a ROS2 adapter and **real** `clean-and-map` coverage events. Agree an action/service contract with the maintainer first.
2. Nav2 KeepoutFilter mask + inflation tests with robot footprint, dynamic obstacles, cliffs and recovery. Run headless Gazebo regressions on `living_room` and `multi_room`.
3. Automatic room segmentation, persisted human room labels, spot editing and zone UI; integrate dock-cycle recharge/bin/mop interruptions. Assess swath coverage and localization error, not only cell-centre completion.
4. Evaluate algorithmic efficiency against upstream's coverage planner; nearest-cell routing here optimizes correctness of a small reference, **not** cleaning time or energy.

## Contribution and AI disclosure

Authored with ChatGPT assistance, reviewed and exercised by the contributor through deterministic tests. Published independently per the OOMWOO contribution guide; the maintainer decides whether to feature or adapt it. Apache-2.0; the project keeps its own code and license terms.
