"""Linux-only nonblocking Nav2 goal-journal and lock-inode regression.

Safety scope: synthetic filesystem behavior. No ROS2, motor or hardware.
"""
import os
from pathlib import Path
import stat
import subprocess
import sys
import tempfile
import unittest

from oomwoo_cleaning_jobs.goal_journal import GoalJournal, GoalJournalError

A = "ab" * 16
TOKEN = "cd" * 16 + "-1"
DIGEST = "ef" * 32


@unittest.skipUnless(sys.platform.startswith("linux") and hasattr(os, "mkfifo"),
                     "requires Linux FIFOs")
class SpecialFileJournalTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.goal = Path(self.tmp.name) / "goal.json"

    def _begin(self):
        return GoalJournal(self.goal).begin(
            uuid_hex=A, token=TOKEN, job_id="special-file-lab",
            map_digest=DIGEST, target_cell=2)

    def _attempt_in_child(self, code):
        # A blocked read must never stall the calling ROS2 executor forever.
        # subprocess.run kills/reaps only our own child on TimeoutExpired.
        result = subprocess.run(
            [sys.executable, "-c", code, str(self.goal)],
            capture_output=True, text=True, timeout=2.0)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "REFUSED")

    def test_fifo_instead_of_journal_fails_closed_without_hanging(self):
        os.mkfifo(self.goal, mode=0o600)
        code = (
            "import sys\n"
            "from pathlib import Path\n"
            "from oomwoo_cleaning_jobs.goal_journal import GoalJournal,GoalJournalError\n"
            "try: GoalJournal(Path(sys.argv[1]))\n"
            "except GoalJournalError: print('REFUSED')\n"
            "else: raise AssertionError('FIFO accepted')\n"
        )
        self._attempt_in_child(code)
        self.assertTrue(stat.S_ISFIFO(self.goal.lstat().st_mode))

    def test_fifo_lock_refuses_begin_and_never_creates_journal(self):
        lock = self.goal.with_name(self.goal.name + ".lock")
        os.mkfifo(lock, mode=0o600)
        code = (
            "import sys\n"
            "from pathlib import Path\n"
            "from oomwoo_cleaning_jobs.goal_journal import GoalJournal,GoalJournalError\n"
            "j=GoalJournal(Path(sys.argv[1]))\n"
            "try: j.begin(uuid_hex='ab'*16,token='cd'*16+'-1',"
            "job_id='lab',map_digest='ef'*32,target_cell=2)\n"
            "except GoalJournalError: print('REFUSED')\n"
            "else: raise AssertionError('FIFO lock accepted')\n"
        )
        self._attempt_in_child(code)
        self.assertFalse(self.goal.exists())
        self.assertTrue(stat.S_ISFIFO(lock.lstat().st_mode))

    def test_hardlinked_lock_rejected_without_goal_write(self):
        lock = self.goal.with_name(self.goal.name + ".lock")
        lock.touch(mode=0o600)
        lock.chmod(0o600)
        shadow = Path(self.tmp.name) / "lock-alias"
        os.link(lock, shadow)
        self.assertEqual(lock.stat().st_nlink, 2)
        with self.assertRaises(GoalJournalError):
            self._begin()
        self.assertFalse(self.goal.exists())
        self.assertEqual(lock.stat().st_nlink, 2)

    def test_private_regular_journal_and_lock_still_work(self):
        result = self._begin()
        self.assertEqual(result["state"], "submitting")
        lock = self.goal.with_name(self.goal.name + ".lock")
        self.assertTrue(lock.is_file())
        self.assertEqual(lock.stat().st_nlink, 1)
        journal = GoalJournal(self.goal)
        self.assertEqual(journal.read()["uuid"], A)
        journal.accepted(token=TOKEN, uuid_hex=A)
        journal.terminal(token=TOKEN, uuid_hex=A, outcome="succeeded")
        self.assertEqual(GoalJournal(self.goal).read()["outcome"], "succeeded")


if __name__ == "__main__":
    unittest.main()
