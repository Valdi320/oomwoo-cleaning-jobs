"""Post-release Nav2 proof must remain CURRENT until dock-drive has stopped.

Pure Python; does not import ROS, connect to motors, or authenticate sensors.
"""
import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
from oomwoo_cleaning_jobs.goal_journal import GoalJournal
from oomwoo_cleaning_jobs.nav_lease_witness import NavLeaseWitness, nav_status_idle

UUID = "b6" * 16
OTHER = "c7" * 16
TOKEN = "ab" * 16 + "-1"
MAP = "e8" * 32

class DockReleaseContinuityTests(unittest.TestCase):
    def setUp(self):
        self.tmp = TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.file = Path(self.tmp.name) / "goal.json"
        self.journal = GoalJournal(self.file)
        self.witness = NavLeaseWitness(self.journal)
        self.journal.begin(uuid_hex=UUID, token=TOKEN, job_id="lab",
                           map_digest=MAP, target_cell=1)
        self.journal.accepted(token=TOKEN, uuid_hex=UUID)

    def terminal(self, outcome="canceled"):
        self.witness.observe(((UUID, 2),))
        self.journal.terminal(token=TOKEN, uuid_hex=UUID, outcome=outcome)
        self.witness.observe(((UUID, {"canceled": 5, "succeeded": 4,
                                      "aborted": 6}[outcome]),))

    def test_matching_terminal_remains_authorized(self):
        self.terminal()
        self.assertTrue(self.witness.still_authorizes_dock(((UUID, 5),)))

    def test_stale_legacy_release_true_but_changed_outcome_must_fail(self):
        self.terminal()
        snapshot = ((UUID, 4),)
        # Legacy relay only checked idle + *any* terminal for same UUID;
        # it did not refresh the witness after leaving NAV ownership.
        legacy = (nav_status_idle(snapshot) and
                  self.witness.can_release_for_dock() and
                  any(u == UUID and s in (4, 5, 6) for u, s in snapshot))
        self.assertTrue(legacy)
        self.assertFalse(self.witness.still_authorizes_dock(snapshot))
        self.assertIsNotNone(self.witness.fault)

    def test_foreign_active_goal_revokes_release(self):
        self.terminal()
        self.assertFalse(self.witness.still_authorizes_dock(((UUID, 5), (OTHER, 2))))
        self.assertIsNotNone(self.witness.fault)

    def test_missing_own_terminal_cannot_reuse_past_release(self):
        self.terminal()
        self.assertFalse(self.witness.still_authorizes_dock(()))

    def test_duplicate_status_revokes_release(self):
        self.terminal()
        self.assertFalse(self.witness.still_authorizes_dock(((UUID, 5), (UUID, 5))))
        self.assertIsNotNone(self.witness.fault)

    def test_unknown_status_revokes_release(self):
        self.terminal()
        self.assertFalse(self.witness.still_authorizes_dock(((UUID, 5), (OTHER, 0))))
        self.assertIsNotNone(self.witness.fault)

    def test_corrupt_journal_revokes_release(self):
        self.terminal()
        self.file.write_text("{", encoding="utf-8")
        self.assertFalse(self.witness.still_authorizes_dock(((UUID, 5),)))

    def test_replaced_journal_revokes_release(self):
        self.terminal()
        self.journal.begin(uuid_hex=OTHER, token="cd"*16+"-2",
                           job_id="lab", map_digest=MAP, target_cell=2)
        self.assertFalse(self.witness.still_authorizes_dock(((UUID, 5),)))

    def test_aborted_terminal_is_never_dock_permit(self):
        self.terminal("aborted")
        self.assertFalse(self.witness.still_authorizes_dock(((UUID, 6),)))

    def test_repeated_status_and_unrelated_finished_goal_remain_ok(self):
        self.terminal("succeeded")
        for _ in range(3):
            self.assertTrue(self.witness.still_authorizes_dock(((UUID, 4), (OTHER, 5))))
        self.assertIsNone(self.witness.fault)

    def test_fault_latched_after_old_status_returns(self):
        self.terminal()
        self.witness.still_authorizes_dock(((UUID, 4),))
        self.assertFalse(self.witness.still_authorizes_dock(((UUID, 5),)))
        self.assertIsNotNone(self.witness.fault)

    def test_without_terminal_observation_cannot_authorize(self):
        self.witness.observe(((UUID, 2),))
        self.assertFalse(self.witness.still_authorizes_dock(((UUID, 5),)))

if __name__ == "__main__":
    unittest.main()
