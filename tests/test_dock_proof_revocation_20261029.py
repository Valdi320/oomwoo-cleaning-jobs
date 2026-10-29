"""Detect stale docking completion after later contradictory lab signals.

These isolated pure Python tests send no motor commands.
"""
import unittest

from oomwoo_cleaning_jobs.dock_handoff import ExclusiveVelocityMux, DockHandoffError


class DockProofRevocationTests(unittest.TestCase):
    def setUp(self):
        self.m = ExclusiveVelocityMux()
        for i in range(8):
            self.m.observe_odom(now=1 + .12*i, linear_mps=0., angular_rps=0.)
        self.m.claim_dock(now=1.9, nav2_relinquished=True,
                          dock_drive_input_isolated=True)
        self.m.observe_dock(now=2.0, drive_state="SEARCH", docked=False)
        self.m.observe_dock(now=2.1, drive_state="DONE", docked=True)

    def test_later_search_undocked_revokes_completion(self):
        # Previously SEARCH false silently preserved _docked and _dock_done.
        with self.assertRaises(DockHandoffError):
            self.m.observe_dock(now=2.2, drive_state="SEARCH", docked=False)
        self.assertEqual(self.m.owner, self.m.STOP)
        self.assertFalse(self.m.docking_proven)
        for i in range(8):
            self.m.observe_odom(now=3+i*.12, linear_mps=0., angular_rps=0.)
        with self.assertRaises(DockHandoffError):
            self.m.finish_dock(now=4., dock_drive_terminated=True)

    def test_later_undocked_idling_revokes_completion(self):
        with self.assertRaises(DockHandoffError):
            self.m.observe_dock(now=2.2, drive_state="IDLE", docked=False)
        self.assertEqual(self.m.owner, self.m.STOP)
        self.assertFalse(self.m._dock_done)
        self.assertFalse(self.m._docked)

    def test_later_conflicting_docked_signal_revokes_completion(self):
        with self.assertRaises(DockHandoffError):
            self.m.observe_dock(now=2.2, drive_state="GOTO", docked=True)
        self.assertEqual(self.m.owner, self.m.STOP)
        self.assertFalse(self.m._dock_done)
        self.assertFalse(self.m._docked)

    def test_identical_confirmations_are_allowed(self):
        self.m.observe_dock(now=2.2, drive_state="DONE", docked=True)
        self.assertTrue(self.m._dock_done)
        self.assertTrue(self.m._docked)
        self.assertTrue(self.m.docking_proven)

    def test_docked_false_retracts_even_if_state_still_done(self):
        with self.assertRaises(DockHandoffError):
            self.m.observe_dock(now=2.2, drive_state="DONE", docked=False)
        self.assertEqual(self.m.owner, self.m.STOP)
        self.assertFalse(self.m.docking_proven)

    def test_ordinary_intermediate_states_before_done_still_work(self):
        m = ExclusiveVelocityMux()
        for i in range(8):
            m.observe_odom(now=1 + .12*i, linear_mps=0., angular_rps=0.)
        m.claim_dock(now=1.9, nav2_relinquished=True,
                     dock_drive_input_isolated=True)
        m.observe_dock(now=2., drive_state="SEARCH", docked=False)
        m.observe_dock(now=2.1, drive_state="GOTO", docked=False)
        m.observe_dock(now=2.2, drive_state="DONE", docked=True)
        self.assertTrue(m.docking_proven)

    def test_clock_rollback_stays_stop_default(self):
        with self.assertRaises(DockHandoffError):
            self.m.observe_dock(now=1., drive_state="SEARCH", docked=False)
        self.assertEqual(self.m.owner, self.m.STOP)


if __name__ == "__main__":
    unittest.main()
