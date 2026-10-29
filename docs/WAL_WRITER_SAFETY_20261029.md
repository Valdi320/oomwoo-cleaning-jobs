# Coverage WAL: serialize writers and refuse unsafe filesystem nodes (2026-10-29)

## Problem / reproduce (Linux, synthetic)

The durable coverage WAL was revision-checked but **not serialized** across processes. Two writers could read the same revision simultaneously, replace the JSON independently, each report success, and silently lose one verified coverage receipt. A stale instance could also report the WAL drained when another writer had staged a receipt. Reads followed symlinks and blocked indefinitely on a FIFO. A dangling symlink was mistaken for an absent ledger and overwritten.

On the parent commit `e3689b687329ac2470fcbcf091b2ac55c992ba17`, the seven new targeted regressions produced **six FAIL and one ERROR** (FIFO read exceeded the 2.5-second child timeout). The concurrent two-process test observed two "accepted" replies, although a last-writer-wins replacement could preserve only one.

## Change / safety contract

* Use a private persistent `coverage.wal.lock` regular file and nonblocking Linux `flock` around initialization and all `stage`/`project` transitions; verify that the lock inode remains identical after acquisition.
* Reject unsafe lock types, hardlinks, symlinks and permissive modes. WAL reads use `O_NOFOLLOW | O_NONBLOCK`, verify regular single-linked private inodes, and reject files larger than 64 MiB instead of allocating arbitrary RAM.
* Compare **the whole on-disk WAL state** with the loaded snapshot under the lock before changing it. A stale instance must fail closed and explicitly reopen; never auto-merge unverified receipts.
* `pending` and `require_drained` no longer use stale process memory to certify that every durable receipt was reconciled.
* Initialization uses `lstat` under the lock, so a dangling symlink cannot be interpreted as a fresh journal.
* No external dependencies beyond Linux stdlib; no changes to firmware, motor control, Nav2, charging or upstream source files.

## Validation

```bash
git checkout <exact published commit>
PYTHONPATH=src python3 -m unittest discover -s tests -p test_coverage_wal_writer_safety_20261029.py -v
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py' -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -p 'test_*.py' -q
git diff --check
```

**Observed** after patch: 7/7 targeted PASS, 683/683 full PASS, 683/683 `python -O` PASS, diff check PASS on user Linux x86_64, Python 3.14. Baseline: 0/7 PASS; six regression failures plus FIFO timeout error. The test uses two OS processes and an injected scheduling barrier; it is synthetic filesystem evidence, **not** proof of ROS2/Gazebo timing or physical cleaning.

## Compatibility, reviewer checklist and limits

* This is a stricter fail-closed contract: unexpected concurrent writes now raise `CoverageLedgerError`, and callers must reopen/reconcile the authoritative WAL before retrying. Previously permissive, linked or giant WALs are rejected. The new stable `.lock` file must remain on the same local filesystem and must not be removed while the ledger is active.
* Cooperative same-host writers using this implementation are serialized. Cross-host/NFS and noncooperating writers are **NOT protected**; malicious same-UID pathname attacks remain outside the guarantee. This does **not** confer global Nav2 ownership.
* A nonblocking busy lock causes a rejected attempt, not a wait or a fake success. Upper-layer retry/telemetry policy is pending.
* The 64-MiB bound is a safety limit, not a measured SBC memory requirement; actual RSS/CPU/energy on ARM and full ROS2/Gazebo integration are untested.
* Review: [ ] verify the lock identity and nofollow semantics on intended deployment filesystem, [ ] replay crashes around both ledger projections, [ ] run real ROS2 Jazzy/Gazebo/Nav2 and actuator data adapters, [ ] measure peak RAM and enforce a one-owner navigation policy, [ ] maintainer review.
* **Integration:** cherry-pick only this contributor-owned commit after OOMWOO maintainer approval, or port the focused file diff into the selected cleaning-jobs implementation. Do not replace shared upstream Nav2 sources without review. **Revert:** ordinary `git revert <commit SHA>` on the contributor branch, no force-push.
