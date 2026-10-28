# Nav2 journal special-file fail-closed hardening (2026-10-28)

**Scope:** independent Valdi320 cleaning-jobs experiment, Apache-2.0, ChatGPT-assisted.
**Base:** `5f93fa4e43a887a849e1be32b8d30dc3fa589caa` (separate contribution branch).
**Upstream pointer only:** https://github.com/makerspet/oomwoo/pull/79

## Defects reproduced before patch

1. A FIFO with mode 0600 at the persisted goal journal pathname caused `GoalJournal(path)` to wait indefinitely in `os.open(..., O_RDONLY)`. An isolated child exceeded the deliberate two-second safety timeout before the regular-inode validation was reached. A local ROS2 process cannot reach journal-reconciliation logic while blocked there.
2. A lock file with `st_nlink=2` was accepted. An alternate name for the same lock inode allows external substitution/unlink scenarios not excluded by its pre-existing regular-file/mode checks. The journal admitted an action in the negative control.

The **pre-fix test matrix was RED**: 4 tests, one TimeoutExpired error, one failed hardlink rejection, two existing scenarios passing. These are synthetic Linux filesystem adversarial checks, **not** a robot exploit demonstration.

## Small correction

- Open the journal using `O_NONBLOCK` in addition to `O_RDONLY|O_NOFOLLOW|O_CLOEXEC`; only after opening, examine `fstat` and reject anything other than one private regular inode. On regular files the nonblocking flag does not change ordinary JSON reads.
- Open the stable `.lock` with `O_NONBLOCK` as a defensive bound for special files; require `st_nlink == 1` before taking `flock`.
- Preserve previous behavior: missing journal is still a permitted virgin state; corrupt/linked/nonregular journal or lock fails closed; no ROS2 APIs, hardware connections, dependencies, or unrelated modules changed.

## Reproduce (Linux, Python 3.10+)

```sh
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_goal_journal_special_files_20261028.py' -v
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_goal_journal*.py' -q
PYTHONPATH=src python3 -m unittest discover -s tests -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -q
python3 -m compileall -q src tests
PYTHONPATH=src python3 -m oomwoo_cleaning_jobs.demo
```

**Observed after fix:** 4/4 focused, 30/30 goal-journal tests, 601/601 full and 601/601 optimized; compilation and demo PASS on Linux Python 3.14. Machine record: [docs/evidence/goal_journal_special_files_20261028.json](evidence/goal_journal_special_files_20261028.json). Remote CI is reported separately after publication.

## Integration and rollback

Review the targeted `goal_journal.py` change and the four new tests. No new ROS2 type or interface is requested. To rollback a selectively imported version, use a **normal revert commit** and retest journal admission, never delete an unresolved UUID file and never force-push shared history.

**Important limits:** This is not an exclusive multi-process Nav2 action owner, nor a proof against a malicious same-UID attacker who can rewrite the containing directory, mount races, untrusted network filesystems, or unknown in-flight goals after file deletion. It does not validate Nav2/Gazebo execution, STM32 MCU safety, or physical motors. Before hardware adoption, establish authoritative Nav2 action arbitration and crash recovery, and repeat integration/hardware QA with the maintainer.
