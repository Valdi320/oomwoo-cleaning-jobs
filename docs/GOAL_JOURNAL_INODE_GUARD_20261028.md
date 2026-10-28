# OOMWOO Nav2 journal inode provenance (2026-10-28)

Owner-only, Apache-2.0, experimental cleaning-jobs. Based on `a6582801fa9ce88e3b5cce7a4d696aa26645aa90`.
Pointer PR: https://github.com/makerspet/oomwoo/pull/79

## Reproduced failure
The durable UUID journal was checked through `Path.exists()` and `read_text()`,
both of which follow symlinks. A symlink to an unrelated *terminal* journal
could make an unresolved goal appear reconciled; a dangling link looked
like a missing file. A hardlink or world-readable replacement was also
trusted. Large valid JSON could force unbounded parsing.
Five negative-control tests fail before the correction (synthetic).

## Fix
On Linux, every initialization and read uses `os.open(O_NOFOLLOW)`, plus
`fstat` regular-file, single-link, private-permission checks. Reads are
bounded to 64 KiB. Missing actual files remain valid fresh startup;
linked or invalid inodes fail closed with `GoalJournalError`.
Existing atomic fsync+replace writes and interprocess `flock` remain.

## Reproduce
`PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_goal_journal_inode_20261028.py' -v`
`PYTHONPATH=src python3 -m unittest discover -s tests -q`
`PYTHONPATH=src python3 -O -m unittest discover -s tests -q`
Machine evidence: `docs/evidence/goal_journal_inode_guard_20261028.json`.

## Exact limits
Only synthetic Linux filesystem tests. This does NOT solve malicious
same-user path replacement between check and atomic rename; secure
parent-directory ownership and cooperative writers are assumptions.
Advisory locks cannot arbitrate independent journal paths or other
Nav2 clients. No ROS2/Gazebo, hardware, MCU stop or cleaning validation.
No persistent processes, firmware, robot hardware or third-party code touched.
