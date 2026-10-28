# Nav2 durable GoalJournal cross-process writer serialization — 2026-10-28

Owner-only experimental cleaning-jobs contribution (Apache-2.0).
Base: 39bd32612bf40b596c7a7077294939233e5b63e9.
Upstream scope pointer: https://github.com/makerspet/oomwoo/pull/79

## Reproduced failure
Two Python controller processes started different Nav2 UUIDs concurrently
from the SAME durable terminal journal. Without a writer lock, both could
read the terminal revision, overwrite each other via atomic replacement,
and each return success, while the file remembered only one UUID.
Pre-fix deterministic two-process test fails (two 'started' instead of
one 'started', one 'blocked'). No Nav2 network goal was sent.

## Bounded correction
A persistent private .lock inode + Linux flock(LOCK_EX|LOCK_NB)
serializes each begin/accepted/terminal and orphan-reconciliation
read-check-replace into ONE transaction. Busy/unavailable/unsafe inode
returns GoalJournalError, blocking goal issuance. Atomic fsync+rename
and old UUID-before-send ordering remain intact. Reconciliation now
writes its terminal record under the same lock instead of nesting locks.
No background service/daemon; the lock FD is closed after each write.
The .lock file intentionally remains for a stable inode.

## Reproduce (synthetic Linux only)
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_goal_journal_writer_lock.py' -v
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_goal_journal.py' -q
PYTHONPATH=src python3 -m unittest discover -s tests -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -q

Machine evidence: docs/evidence/goal_journal_writer_lock_20261028.json.

## Exact limitations
This is only an application-level duplicate-journal race guard under one
POSIX local filesystem/path. It does NOT prove exclusive Nav2 action
ownership across different journal paths, real ROS2 graph arbitration,
Nav2 cancel/terminal status, MCU braking or safe physical docking.
The absence of a malicious same-user process unlinking the lock file is
assumed. Advisory flock must not be bypassed by another client; network
filesystems without reliable flock are unsupported. Hard-realtime
guarantees and ROS2/Gazebo endurance still require testing.
No motors, chargers, batteries, firmware, hardware, or upstream files touched.
