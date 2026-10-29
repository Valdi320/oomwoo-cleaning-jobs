# Orphan Nav2 recovery: validate entire fresh status snapshot

**Owner / license:** Valdi320 experimental Apache-2.0 cleaning-jobs.
**Base:** `69b01e3d09fae953aefca031c4f58a337f259da7`.
**Upstream review pointer:** https://github.com/makerspet/oomwoo/pull/79

## Reproduced failure

In the isolated ROS2 crash-recovery adapter, the final rearm check
previously required only the historical OrphanNav2Witness quiet/cancel
proof and an absence of **foreign ACTIVE** UUIDs in the *latest* packet.
Four concrete status snapshots could therefore pass this final predicate:
an empty GoalStatusArray, foreign-terminal-only history, a foreign
UNKNOWN(0) status, or conflicting duplicated old-UUID statuses.
None proves that the old goal is *currently* terminal CANCELED. A
foreign active goal observed earlier could also disappear and be forgotten.

## Narrow remedy

- Validate the **whole** GoalStatusArray (tuple, valid lowercase UUIDs,
  exactly one entry per UUID, ROS2 codes 1..6; UNKNOWN=0 is rejected).
- Permanently fault this recovery attempt if any foreign active goal,
  malformed/duplicate/UNKNOWN status, or old UUID reactivation is seen.
- At the final resume boundary require the **same old UUID with CANCELED(5)**
  in the most recent snapshot, in addition to the existing exact cancel ACK
  and continuously advancing low-motion simulated odometry.
- Terminal history from unrelated, nonactive goals remains allowed.
- No new Nav2 goals, motor/battery actions or public checkpoint changes.

## Reproduce (local Linux, no ROS2 runtime needed)

```sh
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_orphan_full_status_snapshot_20261029.py' -q
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_restart_witness.py' -q
PYTHONPATH=src python3 -m unittest discover -s tests -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -q
```

**Local PASS:** focal 8/8; existing restart witness 20/20;
full 649/649; Python -O 649/649; `py_compile` and `git diff --check`.
Prior to the patch, the new focal suite failed as expected: the
full-snapshot API and ROS2 integration did not exist. Machine-readable
results: [evidence](evidence/nav2_orphan_full_snapshot_integrity_20261029.json).
The ROS2 adapter wiring has **AST structural** coverage in this iteration,
not a live ROS2/Gazebo integration test.

## Integration, compatibility, review and revert

Maintainer can review two source files and one focal unittest;
import only into an **isolated simulation** controller and rerun the same
cold Gazebo orphan/CancelGoal exercise before considering deployment.
This is strictly more conservative: Nav2 dropping terminal history can
now prevent automatic rearm until explicit safe reconciliation. No on-disk
schema change; preexisting journals remain accepted but resume is harder.
Revert the single source commit normally if integration proves incompatible,
and retain disarmed/manual recovery until alternatives are validated.

**Unresolved:** global Nav2 exclusive owner, DDS authentication, status
provenance, hardware watchdog/interlocks, real SBC performance, physical
safety and real-home cleaning. No ROS2/Gazebo or robot hardware was run
for this patch.
