"""Synthetic Linux O_NOFOLLOW and hardlink protection for Nav2 durable journal."""
from pathlib import Path
import os, stat, tempfile, unittest
from unittest import mock
import oomwoo_cleaning_jobs.goal_journal as journal_module

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

    def _read_during_atomic_replace(self, *, churn=False, unsafe=None):
        journal=GoalJournal(self.path)
        journal.begin(uuid_hex=UUID,token=TOKEN,job_id="atomic-read-race",
                      map_digest=DIGEST,target_cell=4)
        original_open=os.open
        in_transition=False
        outer_opens=0
        def raced_open(path,flags,*args,**kwargs):
            nonlocal in_transition,outer_opens
            fd=original_open(path,flags,*args,**kwargs)
            if (Path(path)==self.path and not in_transition
                    and flags & os.O_ACCMODE == os.O_RDONLY):
                outer_opens+=1
                if outer_opens==1 or churn:
                    in_transition=True
                    try:
                        if outer_opens==1:
                            journal.accepted(token=TOKEN,uuid_hex=UUID)
                        else:
                            journal.terminal(token=TOKEN,uuid_hex=UUID,outcome="canceled")
                    finally:
                        in_transition=False
                    self.assertEqual(os.fstat(fd).st_nlink,0)
                    if unsafe=="missing":
                        self.path.unlink()
                    elif unsafe=="mode":
                        self.path.chmod(0o644)
                    elif unsafe=="hardlink":
                        os.link(self.path,self.path.with_name("other-link.json"))
                    elif unsafe=="symlink":
                        target=self.path.with_name("real.json")
                        self.path.rename(target)
                        self.path.symlink_to(target)
            return fd
        with mock.patch.object(journal_module.os,"open",side_effect=raced_open):
            try:
                return journal.read()
            finally:
                self.assertLessEqual(outer_opens,2)

    def test_legitimate_atomic_replace_reopens_current_private_record_once(self):
        record=self._read_during_atomic_replace()
        self.assertEqual(record["state"],"accepted")
        self.assertEqual(record["uuid"],UUID)
        self.assertEqual(record["revision"],2)

    def test_missing_reopened_path_is_not_a_valid_cold_start(self):
        with self.assertRaises(GoalJournalError):
            self._read_during_atomic_replace(unsafe="missing")

    def test_repeated_replacement_still_refuses_without_unbounded_retry(self):
        with self.assertRaises(GoalJournalError):
            self._read_during_atomic_replace(churn=True)

    def test_reopened_path_must_keep_all_original_security_checks(self):
        for unsafe in ("mode","hardlink","symlink"):
            with self.subTest(unsafe=unsafe), tempfile.TemporaryDirectory() as scratch:
                saved=self.path
                self.path=Path(scratch)/"goal.json"
                try:
                    with self.assertRaises(GoalJournalError):
                        self._read_during_atomic_replace(unsafe=unsafe)
                finally:
                    self.path=saved

    def test_missing_journal_is_valid_initial_state(self):
        j=GoalJournal(self.path)
        self.assertIsNone(j.read())
        self.assertFalse(self.path.exists())

if __name__=="__main__":
    unittest.main()
