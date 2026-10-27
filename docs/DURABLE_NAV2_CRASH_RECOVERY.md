# Real Nav2 crash recovery with a durable goal UUID

27 October 2026 — official OOMWOO Gazebo test_room + ROS2 Jazzy/Nav2.
This is **simulation-first**, not certified robot hardware safety.

## What changed

A crashed cleaning controller can lose its Nav2 goal handle while
Nav2 continues driving. Worse: the process can die immediately after
sending a request but before the acceptance callback arrives.

- src/oomwoo_cleaning_jobs/goal_journal.py now persists a preallocated
  ROS2 UUID with the job ID, map digest, target cell and unique token.
  The file is atomically written and fsynced BEFORE send_goal_async.
  A nonterminal UUID blocks a second goal until reconciled.
- src/oomwoo_cleaning_jobs/ros_nav2.py sends the preallocated UUID
  to the real ROS2 action client and checks the acceptance handle UUID.
  The terminal outcome is durably recorded before freeing that goal.
- src/oomwoo_cleaning_jobs/restart_witness.py validates the OLD UUID
  observed ACTIVE, a ROS2 cancel response naming it, terminal CANCELED,
  and a continuous advancing-sim-time period of low odometry motion.
  Merely seeing a cancel ACK does not mean the robot stopped.
- A journal can be reconciled only with a ready OrphanNav2Witness
  instance, not by accepting arbitrary external JSON.

## Reproduced on the actual OOMWOO virtual robot

With a NEW private container and a dedicated ROS_DOMAIN_ID:

1. Process A journaled a UUID, sent NavigateToPose, observed that
   same action UUID active and 0.0585m of Gazebo motion.
2. A deliberately exited abruptly without asking Nav2 to cancel.
3. Separate process B loaded that UUID/checkpoint and subscribed to
   Nav2's live status, observing the old UUID ACTIVE.
4. B used the genuine ROS2 CancelGoal service to request cancellation
   of that EXACT UUID. The response named it and a terminal CANCELED
   result arrived.
5. B observed 1.22 simulated seconds of quiet OOMWOO odometry,
   containing 62 fresh low-motion samples.
6. B verified no other Nav2 goal was active, checked fresh TF pose,
   source map fingerprint and previous checkpoint, then reconciled
   the durable UUID and deliberately resumed the same job.
7. A fresh UUID was journaled and dispatched to real Nav2; the
   new goal also finished terminal CANCELED and was durably recorded.
   No unverified cleaning coverage was credited.

Machine-readable evidence:
[evidence/ros_nav2_orphan_restart_20261027.json](evidence/ros_nav2_orphan_restart_20261027.json).

## How to reproduce

Use a fresh isolated Podman container with the official OOMWOO Jazzy
dev image, Gazebo test_room, Nav2 and AMCL ready, as specified in
[ROS2_GAZEBO_VALIDATION.md](ROS2_GAZEBO_VALIDATION.md).
No other action client may command the same simulator.

Run these commands inside that private container:

~~~sh
export PYTHONPATH=/workspace/valdi-cleaning-jobs/src
python3 /workspace/valdi-cleaning-jobs/integration/orphan_nav2_create.py
python3 /workspace/valdi-cleaning-jobs/integration/orphan_nav2_reconcile.py
~~~

The first program intentionally kills only its own test process.
The second must report REAL_NAV2_ORPHAN_RECOVERY with ok=true, the
old and new UUIDs different, terminal_canceled, advancing quiet
odometry, a matching checkpoint and a final real terminal cancel.
On failure, discard only the private simulator container.

## Limitations before autonomous OOMWOO deployment

- This recovery is an independent ROS2 test workflow; the normal
  experimental job node still requires an approved, integrated
  automatic restart/reattachment mechanism.
- If a UUID was persisted but never actually sent, its absence from
  server status is NOT sufficient proof of robot stop. Remain disarmed.
- Other active goals, a changed map, stale localization, sparse
  odometry, missing stop confirmation or unexpected results all
  prohibit resumption.
- Physical MCU bumper, cliff, motor current and watchdog safety
  are not covered. No hardware or real-home testing took place.
