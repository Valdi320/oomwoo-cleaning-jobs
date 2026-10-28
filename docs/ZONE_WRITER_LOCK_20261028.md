# Fail-closed zone revision concurrency (2026-10-28)

Experimental Valdi320 cleaning-jobs only (Apache-2.0; AI-assisted).
Base: 6f286d01f0ad053223b2338e97418cb495aed140.
Upstream reference: https://github.com/makerspet/oomwoo/pull/79

## Reproduced bug / impact

Two application processes can hold the same active ZoneUpdateTransaction.
Once process A stages an updated virtual-wall mask, process B previously
returned False from must_block_nav() using an outdated in-memory 'active'
snapshot. Worse, two concurrent writers could BOTH pass the revision
compare before either os.replace, losing one change.

This is a defect in the private application interlock / persistence, NOT
proof of a motor, firmware or Nav2 failure.

## Bounded engineering correction

- Under Linux, each transaction has a stable .lock inode with mode 0600.
  fcntl.flock(LOCK_EX | LOCK_NB) serializes revision read/compare/write,
  atomic replace and directory fsync. Contention or I/O failure fails
  CLOSED rather than blocking the executor indefinitely. The .lock file
  intentionally persists; it is not a daemon or background process.
- A concurrent first initializer cannot overwrite a file created since
  its stale existence check. Crash-recovery paths now set the loaded
  revision before doing their locked revision +1 update.
- must_block_nav() reads the durable record, rejects a missing/corrupt
  record and compares the entire persisted snapshot to the owner's
  in-memory version. Stale owners must not authorize navigation.
- No change to third-party packages, upstream OOMWOO, Nav2 motor control,
  firmware, charging, PCB, or physical wiring.

## Reproduce on Linux

PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_zone_writer_lock.py' -v
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_zone_revision.py' -v
PYTHONPATH=src python3 -m unittest discover -s tests -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -q
PYTHONPATH=src python3 -m oomwoo_cleaning_jobs.demo

Machine evidence: docs/evidence/zone_writer_lock_20261028.json.

## Boundaries and remaining integration

The negative controls and regressions are Linux Python with SYNTHETIC
rasters; the process test uses two short-lived isolated Python children.
No ROS2/Nav2/Gazebo, Raspberry Pi timing, physical cliff sensors,
MCU stopping or real actuators were tested. This is not a hard real-time
mutex or an exclusive ROS2 action lease. A race between the last disk
check and a new writer can still exist without upstream canonical
single-owner Nav2 arbitration. A shared filesystem not supporting
flock semantics, malicious file tampering, and system failure after
fsync remain outside this experiment. Maintainer review and real
middleware hot-update tests are still needed.
