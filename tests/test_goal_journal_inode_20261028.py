"""Synthetic Linux O_NOFOLLOW and hardlink protection for Nav2 durable journal."""
from pathlib import Path
import os, stat, tempfile, unittest

from oomwoo_cleaning_jobs.goal_journal import GoalJournal, GoalJournalError

UUID = "a1"*16
NEW = "b2"*16
TOKEN = "c3"*16+"-1"
TOKEN2 = "d4"*16+"-2"
DIGEST = "e5"*32

class JournalInodeGuardTests(unittest.TestCase):
    def setUp(self):
        self.t=tempfile.TemporaryDirectory()
        self.addCleanup(self.t.cleanup)
        self.path=Path(self.t.name)/"goal.json"

    def _terminal_record(self):
        journal=GoalJournal(self.path)
        journal.begin(uuid_hex=UUID,token=TOKEN,job_id="synthetic",
                      map_digest=DIGEST,target_cell=4)
        journal.terminal(token=TOKEN,uuid_hex=UUID,outcome="canceled")
        return self.path.read_bytes()

    def test_symlink_to_valid_terminal_record_is_not_trusted(self):
        saved=self._terminal_record()
        original=self.path.with_name("original-goal.json")
        self.path.rename(original)
        self.path.symlink_to(original)
        with self.assertRaises(GoalJournalError):
            GoalJournal(self.path)
        self.assertEqual(original.read_bytes(),saved)

    def test_dangling_symlink_is_not_treated_as_missing_journal(self):
        self.path.symlink_to(self.path.with_name("missing.json"))
        with self.assertRaises(GoalJournalError):
            GoalJournal(self.path)
        self.assertTrue(self.path.is_symlink())

    def test_hardlinked_terminal_journal_cannot_authorize_new_goal(self):
        saved=self._terminal_record()
        other=self.path.with_name("shared.json")
        os.link(self.path,other)
        with self.assertRaises(GoalJournalError):
            GoalJournal(self.path)
        self.assertEqual(other.read_bytes(),saved)

    def test_world_readable_journal_is_refused_before_action(self):
        saved=self._terminal_record()
        self.path.chmod(0o644)
        with self.assertRaises(GoalJournalError):
            GoalJournal(self.path)
        self.assertEqual(self.path.read_bytes(),saved)

    def test_oversized_valid_json_is_refused_before_goal_issue(self):
        saved=self._terminal_record()
        self.path.write_bytes(saved.rstrip()+b" "*70000)
        with self.assertRaises(GoalJournalError):
            GoalJournal(self.path)

    def test_unmodified_private_regular_journal_survives_restart(self):
        self._terminal_record()
        self.assertEqual(stat.S_IMODE(self.path.stat().st_mode),0o600)
        j=GoalJournal(self.path)
        self.assertEqual(j.read()["state"],"terminal")
        j.begin(uuid_hex=NEW,token=TOKEN2,job_id="synthetic",
                map_digest=DIGEST,target_cell=5)
        self.assertEqual(GoalJournal(self.path).read()["state"],"submitting")

    def test_missing_journal_is_valid_initial_state(self):
        j=GoalJournal(self.path)
        self.assertIsNone(j.read())
        self.assertFalse(self.path.exists())

if __name__=="__main__":
    unittest.main()
