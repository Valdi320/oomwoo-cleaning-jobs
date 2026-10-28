"""A substituted goal-journal lock inode must never authorize Nav2 issuance.

Synthetic Linux tests: no ROS2 action server, motors, batteries or hardware.
"""
import fcntl
import os
from pathlib import Path
import tempfile
import unittest
from unittest import mock

from oomwoo_cleaning_jobs.goal_journal import GoalJournal, GoalJournalError

_UUID="ab"*16
_TOKEN="cd"*16+"-1"
_MAP="ef"*32

class JournalNamedLockIdentityTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)/"nav2.json"
        self.lock=Path(self.temp.name)/"nav2.json.lock"
        self.journal=GoalJournal(self.path)

    def _begin(self):
        return self.journal.begin(uuid_hex=_UUID,token=_TOKEN,
                job_id="lock-inode-test",map_digest=_MAP,target_cell=8)

    def _inject_lock_path_change(self, *, replace):
        original=fcntl.flock
        changed=[False]
        def after_flock(fd,op):
            outcome=original(fd,op)
            if not changed[0]:
                changed[0]=True
                self.lock.rename(self.lock.with_name("orphaned-lock"))
                if replace:
                    new=os.open(self.lock,os.O_RDWR|os.O_CREAT|os.O_EXCL,0o600)
                    os.close(new)
            return outcome
        with mock.patch("oomwoo_cleaning_jobs.goal_journal.fcntl.flock",
                        side_effect=after_flock):
            with self.assertRaises(GoalJournalError):
                self._begin()
        self.assertTrue(changed[0],"the fault must actually be injected")
        self.assertFalse(self.path.exists(),
                         "unsafe inode change must not persist a new Nav2 UUID")

    def test_lock_inode_replaced_between_open_and_admission(self):
        self._inject_lock_path_change(replace=True)

    def test_lock_name_removed_between_open_and_admission(self):
        self._inject_lock_path_change(replace=False)

    def test_normal_stable_private_lock_allows_one_goal(self):
        self._begin()
        self.assertEqual(GoalJournal(self.path).read()["state"],"submitting")
        self.assertEqual(self.lock.stat().st_mode & 0o777,0o600)


if __name__=="__main__":
    unittest.main()
