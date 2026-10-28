# Fail closed when the Nav2 GoalJournal writer lock inode is replaced (2026-10-28)

Contributor: Valdi320. Independent Apache-2.0 `oomwoo-cleaning-jobs` source. Scope of upstream OOMWOO PR #79: versioned reviewer pointer only.

## Defect and reproducibility

The Linux `GoalJournal._writer_lock()` checked the opened private single-link `.lock` file and acquired `flock`, but never verified that the *name still pointed to that inode*. If a same-user cleanup/rotation renamed the lock between `open` and `flock`, the holder could journal a new Nav2 UUID against an **orphaned lock inode**, while a second process could concurrently lock the replacement. Locking would no longer serialize writer admission. This matters because a new UUID is persisted before ROS2 goal dispatch.

Two deterministic fault-injection controls **FAILED before the fix**, and the stable lock control PASS (2 fails, 1 pass). The injection takes place immediately after `flock` and before the journal write; no actual Nav2 goal is sent.

## Fix

Immediately after nonblocking `flock`, compare `os.stat(lock_path, follow_symlinks=False)` device/inode to the original `fstat(fd)`. Reject replaced, missing or unstat-able lock paths with `GoalJournalError` **before** accepting the goal or modifying the durable JSON. No new schema, services, dependencies, motors, battery, or ROS2 behavior. Existing cross-process locking and fsync/rename remain unchanged.

## Reproduce (Linux, standard Python; from repo root)

```sh
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_goal_journal_lock_identity_20261028.py' -v
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_goal_journal_writer_lock.py' -q
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_goal_journal.py' -q
PYTHONPATH=src python3 -m unittest discover -s tests -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -q
```

**Observed locally:** focal 3/3 PASS, existing lock 4/4 PASS, journal 15/15 PASS, full 633/633 PASS, `python3 -O` 633/633 PASS; `git diff --check` PASS. Before fix two new controls reproduced acceptance of an unsafe renamed lock. Machine evidence: [JSON](evidence/goal_journal_lock_identity_20261028.json). CI must be checked against the exact published SHA, not inferred from local tests.

## Integration, compatibility and safety limits

- No journal JSON or public method signature changes; existing private regular `.lock` files work.
- The fix detects substitution **at the identity check**. A malicious non-cooperating process can still unlink/recreate files *after* the check; protect the directory by permissions and restrict access to the trusted controller. POSIX `flock` is advisory and does not coordinate different journal paths or network filesystems with unreliable locks.
- This does **not** establish global single-owner Nav2 arbitration, nor confirm live Nav2 cancel, reliable ROS2 DDS transport or a safe MCU actuator chain. These are synthetic Linux filesystem tests only, **not** Gazebo/ROS2/ARM-SBC/physical hardware/power measurements.
- Maintainer should review the isolated source/test changes and decide the canonical journal owner and location before selective import. Re-run focal/full tests and ROS2 end-to-end recovery before claiming hardware safety.
- To revert an imported patch, use an ordinary `git revert` on the imported change; **never** discard an unresolved Nav2 UUID or forcibly reset upstream history.
- TODO: integration test where an actual cooperating ROS2 writer experiences stale lock rotation; unique Nav2 action owner across all clients; real MCU-owned motion cutoff.

No new PR or routine PR comment needed; upstream PR #79 points to this independent contribution.
