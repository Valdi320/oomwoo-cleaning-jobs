"""Exact Nav2 action UUID lease evidence for dock handoff (pure Python)."""
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from oomwoo_cleaning_jobs.goal_journal import GoalJournal
from oomwoo_cleaning_jobs.nav_lease_witness import NavLeaseWitness

UUID="b6"*16
OTHER="c7"*16
TOKEN="ab"*16+"-1"
DIGEST="e8"*32


class DockNavUUIDWitnessTests(unittest.TestCase):
    def setUp(self):
        self.tmp=TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.file=Path(self.tmp.name)/"goal.json"
        self.j=GoalJournal(self.file)
        self.w=NavLeaseWitness(self.j)

    def start(self):
        self.j.begin(uuid_hex=UUID,token=TOKEN,job_id="lab",
                     map_digest=DIGEST,target_cell=1)
        self.j.accepted(token=TOKEN,uuid_hex=UUID)

    def end(self,outcome="canceled"):
        self.j.terminal(token=TOKEN,uuid_hex=UUID,outcome=outcome)

    def test_unknown_stale_success_before_our_action_cannot_release(self):
        self.w.observe(((OTHER,4),))
        self.assertFalse(self.w.can_release_for_dock())

    def test_current_exact_active_and_matching_terminal_required(self):
        self.start()
        self.w.observe(((UUID,2),))
        self.assertFalse(self.w.can_release_for_dock())
        self.end()
        self.w.observe(((UUID,5),))
        self.assertTrue(self.w.can_release_for_dock())

    def test_journal_terminal_without_own_active_session_not_sufficient(self):
        self.start()
        self.end("succeeded")
        self.w.observe(((UUID,4),))
        self.assertFalse(self.w.can_release_for_dock())

    def test_callback_terminal_before_fsync_must_wait_for_confirmation(self):
        self.start()
        self.w.observe(((UUID,2),))
        self.w.observe(((UUID,4),))
        self.assertFalse(self.w.can_release_for_dock())
        self.end("succeeded")
        self.w.observe(((UUID,4),))
        self.assertTrue(self.w.can_release_for_dock())

    def test_unknown_active_goal_faults(self):
        self.start()
        self.w.observe(((UUID,2),))
        self.w.observe(((OTHER,2),))
        self.assertFalse(self.w.can_release_for_dock())
        self.assertIsNotNone(self.w.fault)

    def test_simultaneous_owners_fault(self):
        self.start()
        self.w.observe(((UUID,2),(OTHER,1)))
        self.assertFalse(self.w.can_release_for_dock())
        self.assertIsNotNone(self.w.fault)

    def test_wrong_action_goal_not_current_journal(self):
        self.start()
        self.w.observe(((OTHER,2),))
        self.assertIsNotNone(self.w.fault)

    def test_aborted_goal_requires_operator_not_dock_handoff(self):
        self.start()
        self.w.observe(((UUID,2),))
        self.end("aborted")
        self.w.observe(((UUID,6),))
        self.assertFalse(self.w.can_release_for_dock())

    def test_reused_terminal_after_reset_does_not_rearm(self):
        self.start()
        self.w.observe(((UUID,2),))
        self.end("canceled")
        self.w.observe(((UUID,5),))
        self.assertTrue(self.w.can_release_for_dock())
        self.w.reset()
        self.w.observe(((UUID,5),))
        self.assertFalse(self.w.can_release_for_dock())

    def test_missing_journal_refuses_any_active_status(self):
        self.w.observe(((UUID,2),))
        self.assertFalse(self.w.can_release_for_dock())
        self.assertIsNotNone(self.w.fault)


if __name__=="__main__":
    unittest.main()
