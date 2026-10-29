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
```

Historical initial snapshot: 33/33 focal PASS and a previously reported
724/724 full suite, normal and `-O`, on the user's Linux x86 host.
The full suite has NOT been rerun under the current machine instructions. This is entirely **synthetic
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

The current contributor tree now wires this policy into an explicit
`three_way=true` mode of the SAME `ExperimentalDockVelocityRelay`.
The default two-source mode remains available; its active DDS probe passes
again. The opt-in mode has the focused DDS evidence recorded below.
The reactive room-sweep algorithm itself, geometry/obstacle avoidance,
and canonical job API are not implemented by this module.

The isolated three-source DDS adapter now has recorded handoffs and injected
input faults. Remaining integration is a Gazebo run with ACTUAL planner/Nav2/
docking controllers and their phase/UUID coordination, keeping one safe test
output. Do not deploy this module to a real motor topic.

No battery, motor, charging station, firmware or continuous service
has been touched. No claim about cheaper minimum SBC or lower energy
consumption is made.

**Contribution hygiene:** Apache-2.0 owner branch only; upstream #79
remains unchanged for maintainer review. Only local contributor commits are delivered for owner review/integration;
no upstream runtime changes were made.


## Opt-in ROS2 adapter milestone — 29 October 2026

The existing relay selects one policy: the historical two-source default or
`three_way=true`. It retains one publisher on
`/valdi/lab_safe_cmd_vel_output` and one 50ms timer. In three-way mode that
existing timer uses steady time, so a paused ROS simulation clock cannot
suspend output/expiry checks. TF subscriptions use this same node; no extra
application node, timer, tool or worker is introduced. The probe and remap
matrix are extended in their existing files.

| Observed input | Meaning in this disposable simulation |
| --- | --- |
| TF, validated static Raster | Common map-to-base time at most500ms old and a free current cell; no cached-AMCL motion fallback or swept-footprint claim. |
| BatteryState | Finite fraction20% admission /15% active through100%; a simulation battery proxy, without bin/water/charging proof. |
| /clock, meter stability, phases | Advancing clock, recently observed stable meter and known fresh producer phases. This is simulator health, not an MCU safety loop. |
| Command-topic graph | One distinct endpoint per NAV/SWEEP/DOCK input, one self-named lab output, no /cmd_vel publishers; endpoint changes or duplicates stop as observed. |
| Native DDS metadata | Strict publication sequence and same-host wall timestamp, after the current claim; replay history survives STOP. |
| Nav2 status + GoalJournal | Preallocated exact UUID and matching map digest before NAV claim, observed active action and matching durable terminal before handoff. |

Jazzy's actual MessageInfo is a dict with source/received timestamps and
publication/reception sequence numbers. It supplies NO per-packet publisher
GID. The initial assumption failed in a real DDS run and was removed. The
adapter associates packets with unique observed graph endpoints; neither
that association nor a source timestamp is authentication, and DDS discovery
snapshots are not atomic wiring proof. This mode is restricted to the same-host
lab; clocks on separate machines have not been validated.

A native dropout test also found that caching an ACTIVE UUID while refreshing
aggregate health could prolong NAV ownership after status stopped arriving.
The ACTIVE UUID now expires under the existing one-second nonzero-NAV status
budget. The red case reproduces continued ownership before correction; the
final scenario passes **14 checks**, including forced STOP on missing status,
rejection of a release from the lost lease, and a fresh exact-UUID claim before
the subsequent NAV-to-DOCK handoff.

Final native scenario also checks initial missing TF, positive SWEEP/NAV/DOCK
output, foreign commands, invalid battery percentages, TF dropout, command
expiry, overspeed, duplicate writers, paused simulation clock and a latched
map-change fault. **Nine constructor/remap cases** pass, rejecting miswired
sweep/phase/map/clock/stability/TF inputs and motor-output remapping before the
output timer. Four separate focal Python modules pass **12 +6 +24 +10** cases;
six new metadata tests use the real relay methods/core with synthetic clocks,
graph and message metadata. The original active two-way DDS probe also passes.
No full suite was run. Exact commands, negative attempts, hashes and scopes:
[`evidence/three_way_ros2_adapter_codex_20261029.json`](evidence/three_way_ros2_adapter_codex_20261029.json).

Reproduce within the existing isolated Jazzy image with this source at `/work`
and the existing lab interfaces built/sourced (no host tool installation):

```bash
PYTHONPATH=/work/src:$PYTHONPATH python3 \
  /work/integration/ros_dock_velocity_relay_probe.py --three-way
PYTHONPATH=/work/src:$PYTHONPATH python3 \
  /work/integration/input_subscriptions_matrix_20261029.py --three-way
```

This is a real DDS/service/TF-cache bank with SYNTHETIC command, odometry,
clock, map, resource, producer-phase and Nav2-status inputs. It does not run a
Nav2 action server, Gazebo, a detector, a physical robot or actual charging.
In this mode `release_motion` releases software ownership on a fresh zero and
inactive software phase; DONE/Bool does not certify contact or charging.
The historical `finish_dock` operation remains in two-source mode only.

The actual planner still needs a producer phase signal and a pre-send journal/
lease handoff before its Nav2 action. Its0.35m/s and1.0rad/s defaults exceed the
unchanged mux limits0.25/0.8. Actual5Hz simulation cadence must be measured
against the300ms monotonic command budget, and the real Nav2 status/feedback
cadence must be checked before choosing its observation contract. Next work is
this real-controller/Gazebo integration and final single-writer bridge audit;
this milestone does not substitute for it. Hardware availability is explicitly
zero, and no physical strength, RF, charging or robot readiness is inferred.
