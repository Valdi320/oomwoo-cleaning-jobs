"""Exact Nav2 action UUID lease evidence for dock handoff (pure Python)."""
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest

from oomwoo_cleaning_jobs.goal_journal import GoalJournal
from oomwoo_cleaning_jobs.nav_lease_witness import (NavLeaseWitness,nav_status_idle,
                                               valid_nav_status_snapshot)

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


    def test_terminal_journal_blocks_cached_active_motion(self):
        self.start()
        self.w.observe(((UUID,2),))
        self.assertTrue(self.w.current_active_matches())
        self.end("canceled")
        self.assertFalse(self.w.current_active_matches())
        self.assertIsNone(self.w.fault)

    def test_replaced_journal_uuid_blocks_old_active_status(self):
        self.start()
        self.w.observe(((UUID,2),))
        self.end("succeeded")
        self.j.begin(uuid_hex=OTHER,token="cd"*16+"-2",job_id="lab",
                     map_digest=DIGEST,target_cell=2)
        self.j.accepted(token="cd"*16+"-2",uuid_hex=OTHER)
        self.assertFalse(self.w.current_active_matches())
        self.assertIsNotNone(self.w.fault)

    def test_corrupted_journal_blocks_latched_active_motion(self):
        self.start()
        self.w.observe(((UUID,2),))
        self.file.write_text("{",encoding="utf-8")
        self.assertFalse(self.w.current_active_matches())
        self.assertIsNotNone(self.w.fault)

    def test_durable_submitting_goal_cannot_drive(self):
        self.start()
        self.w.observe(((UUID,2),))
        self.end("succeeded")
        self.j.begin(uuid_hex=OTHER,token="de"*16+"-3",job_id="lab",
                     map_digest=DIGEST,target_cell=2)
        self.assertFalse(self.w.current_active_matches())



    def test_unknown_status_revokes_nav_lease(self):
        self.start()
        self.w.observe(((UUID,2),))
        self.assertTrue(self.w.current_active_matches())
        self.w.observe(((UUID,2),(OTHER,0)))
        self.assertFalse(self.w.current_active_matches())
        self.assertIsNotNone(self.w.fault)

    def test_unknown_status_blocks_terminal_dock_release(self):
        self.start()
        self.w.observe(((UUID,2),))
        self.end("canceled")
        self.w.observe(((UUID,5),(OTHER,0)))
        self.assertFalse(self.w.can_release_for_dock())
        self.assertIsNotNone(self.w.fault)

    def test_nav_idle_rejects_unknown_duplicate_and_invalid(self):
        self.assertTrue(nav_status_idle(()))
        self.assertTrue(nav_status_idle(((UUID,4),(OTHER,5))))
        self.assertFalse(nav_status_idle(((UUID,2),)))
        self.assertFalse(nav_status_idle(((OTHER,0),)))
        self.assertFalse(nav_status_idle(((OTHER,7),)))
        self.assertFalse(nav_status_idle(((UUID,4),(UUID,5))))
        self.assertFalse(nav_status_idle((("bad-uuid",5),)))
        self.assertFalse(nav_status_idle(((UUID,True),)))
        self.assertFalse(nav_status_idle(None))

    def test_duplicate_or_malformed_active_snapshot_faults(self):
        self.start()
        for statuses in (((UUID,2),(UUID,2)), ((UUID,2),(UUID,5)),
                         ((UUID,2),(OTHER,-1)), ((UUID,2),("bad",4))):
            w=NavLeaseWitness(self.j)
            w.observe(statuses)
            self.assertIsNotNone(w.fault)
            self.assertFalse(w.can_release_for_dock())

    def test_snapshot_validator_known_terminal_only(self):
        self.assertTrue(valid_nav_status_snapshot(()))
        self.assertTrue(valid_nav_status_snapshot(((UUID,4),(OTHER,6))))
        self.assertFalse(valid_nav_status_snapshot(((UUID,4),(UUID,4))))
        self.assertFalse(valid_nav_status_snapshot(((UUID,0),)))

if __name__=="__main__":
    unittest.main()
