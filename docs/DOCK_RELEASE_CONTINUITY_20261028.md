# Revocable Nav2 terminal proof during simulated dock handoff — 2026-10-28

Owner: Valdi320. License: Apache-2.0. Parent: 72ff69144da6ffcd0c426c19bf9ce14c6618f291.
Upstream pointer (already open): https://github.com/makerspet/oomwoo/pull/79

## Fault and scope
After simulated release_for_dock the relay becomes STOPPED, so its ROS2
status callback stopped observing the exact UUID witness. Previously an
already-authorized terminal journal could be reused at claim_dock even if
Nav2 issued a contradictory terminal snapshot or the journal was replaced
before the claim. During DOCK the relay only checked Nav2 idle, not the
original terminal authorization. This is a stale post-release permit.

## Narrow fix
Introduce pure-Python NavLeaseWitness.still_authorizes_dock(snapshot),
which re-observes and checks both the exact journal and latest terminal
snapshot on every post-release decision. The isolated ROS2 relay now
continues witnessing while release is pending or DOCK owns commands,
rechecks before claim_dock, drops the one-shot release flag after claim,
and stops on lost terminal continuity while DOCK owns the software mux.

The relay remains restricted to /valdi/lab_safe_cmd_vel_output; no real
robot topic or MCU code is modified. This is intentionally fail-closed:
if Nav2 prunes the terminal status, DOCK stops until reviewed.

## Reproducibility
    PYTHONPATH=src python3 -m unittest discover -s tests -p test_dock_release_continuity.py -v
    PYTHONPATH=src python3 -m unittest discover -s tests -q
    PYTHONPATH=src python3 -O -m unittest discover -s tests -q
    PYTHONPATH=src python3 -m py_compile src/oomwoo_cleaning_jobs/ros_dock_velocity_relay.py
    PYTHONPATH=src python3 -m oomwoo_cleaning_jobs.demo

Regression includes a documented pre-fix logical counterexample:
an old CANCELED journal plus a later SUCCEEDED snapshot was incorrectly
accepted by the earlier relay's nav_idle + any-terminal predicate.

## Hard limits
Only pure-Python host tests and syntax compilation of the ROS adapter.
No live ROS2/Nav2/Gazebo execution, no physical robot or independent
sensor authentication. No proof of real motor stop, charging, cleaning,
MCU interlocks or source-topic exclusivity. PR #79 is a pointer; the
canonical OOMWOO runtime and external contributors remain unchanged.
