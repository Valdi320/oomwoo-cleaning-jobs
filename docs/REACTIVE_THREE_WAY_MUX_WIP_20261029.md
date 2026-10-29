# Experimental three-way /cmd_vel policy — 29 October 2026

**Status: LOCAL WIP, not published and not production-integrated.**

## Blocker addressed

The OOMWOO maintainer clarified in upstream PR #79 that room coverage
must move to a reactive /cmd_vel executor while Nav2 handles transits,
docking and localization. Nav2/dock-drive already have an experimental
two-way relay; adding a third publisher directly would risk concurrent
wheel commands. This independent, opt-in pure-Python reference policy
models **one shared STOP/NAV/SWEEP/DOCK owner** rather than silently
introducing a separate sweep-only mux.

## Implementation

- `src/oomwoo_cleaning_jobs/reactive_velocity.py` supplies
  `ThreeWayVelocityMux` and immutable `MotionHealth`. This is a
  proof/command policy with NO ROS2 imports, publishers or motors.
- Each claimed owner receives a new in-memory token. Wrong owner or old
  token cannot emit nonzero output; conflicting nonzero commands STOP.
- No owner is armed without fresh health and quiet odometry. Typical
  synthetic gates: 0.30 s max sensor/command age, 0.75 s observed quiet
  interval, 0.25 m/s linear and 0.8 rad/s angular command limits.
  These are *experiment defaults*, not certified robot limits.
- NAV needs a 32-char lowercase Nav2 UUID and a matching observed active
  goal before nonzero output. Transitioning to idle mutes output; release
  requires an exact terminal UUID and independently verified completion.
- SWEEP requires Nav2 idle, source isolation and dock inactive; DOCK
  requires Nav2 idle and sweep inactive. Both require the fresh quiet
  interval after a handoff. Terminal evidence is one-shot.
- Every output call checks current health, odometry and command age.
  Inconsistent map/resource/localization/safety status, foreign Nav2
  action, stale command or backward time fails closed.

## Reproduce, in this contributor-owned local branch

Starting with source commit
`3f3cd7025ab242564d9a940f15009b2edaabca63`:

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_reactive_velocity_20261029.py' -v
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py' -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -p 'test_*.py' -q
```

Focused: 33/33 PASS. Full suite: 724/724 PASS, normal and `-O`,
observed on the user's Linux x86 host. This is entirely **synthetic
unit-test evidence**; no new ROS2/Gazebo, ARM CPU/energy or physical tests.
Machine-readable re-run report is in
`docs/evidence/reactive_mux_synthetic_20261029.json`.

## Safety limits, compatibility and next integration gate

`MotionHealth` flags and `nav_released` / `terminal_verified` are
**claims supplied by the caller, not independently verified proofs**.
Their adapter MUST validate the source and freshness of each signal,
track a persisted exact Nav2 goal UUID, detect foreign controllers,
verify real velocity-source remappings, provide a physical MCU watchdog
and confirm that only ONE mux publishes to the true robot `/cmd_vel`.
In ROS2, raw /cmd_vel messages have no authentication or durable lease
tokens; this Python token is NOT a security credential.

The current source tree's `ExperimentalDockVelocityRelay` is not wired
to this new policy. Its existing 2-source tests are unchanged and pass.
The reactive room-sweep algorithm itself, geometry/obstacle avoidance,
and canonical job API are not implemented by this module.

Next acceptance gate is an isolated ROS2/Gazebo adapter whose output
remains restricted to a safe test topic, with three independent input
streams, recorded action UUID handoffs, fault injections and single-writer
topic graph audit. Do not deploy this module to a real motor topic.

No battery, motor, charging station, firmware or continuous service
has been touched. No claim about cheaper minimum SBC or lower energy
consumption is made.

**Contribution hygiene:** Apache-2.0 owner branch only; upstream #79
remains unchanged for maintainer review. To revert, delete this
experimental branch/worktree or revert only its future contributor
commit; no upstream runtime changes were made.
