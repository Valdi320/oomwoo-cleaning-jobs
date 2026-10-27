"""Sim-only motor command ownership and OOMWOO dock signal safety tests."""
import unittest
from oomwoo_cleaning_jobs.dock_handoff import (
    ExclusiveVelocityMux,Velocity,DockHandoffError)


class DockHandoffTests(unittest.TestCase):
    def setUp(self):
        self.m=ExclusiveVelocityMux()

    def quiet(self,t0=1.):
        for i in range(8):
            self.m.observe_odom(now=t0+i*.12,
                                linear_mps=0.,angular_rps=0.)

    def nav(self):
        self.quiet()
        self.m.claim_nav(now=1.9,nav2_idle=True,
             pose_fresh=True,dock_process_inactive=True,
             resources_verified=True)

    def test_initial_state_is_stop(self):
        self.assertEqual(self.m.owner,self.m.STOP)
        self.assertEqual(self.m.output(now=0.),Velocity())

    def test_nav_start_requires_zero_motion(self):
        with self.assertRaises(DockHandoffError):
            self.m.claim_nav(now=1.,nav2_idle=True,pose_fresh=True,
                             dock_process_inactive=True,resources_verified=True)
        self.assertEqual(self.m.owner,self.m.STOP)

    def test_nav2_solo_owner_blocks_dock_stream(self):
        self.nav()
        self.m.accept_twist(self.m.NAV,Velocity(.15,.02),now=2.)
        self.m.accept_twist(self.m.DOCK,Velocity(.5,1.0),now=2.01)
        self.assertEqual(self.m.output(now=2.02),Velocity(.15,.02))

    def test_expired_owner_command_goes_zero(self):
        self.nav()
        self.m.accept_twist(self.m.NAV,Velocity(.15,0.),now=2.)
        self.assertEqual(self.m.output(now=2.5),Velocity())

    def test_reject_claim_dock_before_nav_terminal(self):
        self.nav()
        with self.assertRaises(DockHandoffError):
            self.m.release_for_dock(now=2.0,nav2_terminal=False,
                                    action_uuid_reconciled=True,
                                    nav2_input_stopped=True)
        self.assertEqual(self.m.owner,self.m.NAV)

    def test_complete_dock_handoff_and_no_false_charging_proof(self):
        self.nav()
        self.m.release_for_dock(now=2.,nav2_terminal=True,
                                action_uuid_reconciled=True,
                                nav2_input_stopped=True)
        self.assertEqual(self.m.owner,self.m.STOP)
        with self.assertRaises(DockHandoffError):
            self.m.claim_dock(now=2.02,nav2_relinquished=True,
                              dock_drive_input_isolated=True)
        self.quiet(2.1)
        self.m.claim_dock(now=3.,nav2_relinquished=True,
                          dock_drive_input_isolated=True)
        self.m.accept_twist(self.m.NAV,Velocity(.3,0.),now=3.02)
        self.m.accept_twist(self.m.DOCK,Velocity(-.12,0.),now=3.04)
        self.assertEqual(self.m.output(now=3.05),Velocity(-.12,0.))
        self.m.observe_dock(now=3.1,drive_state="SEARCH",docked=False)
        self.m.observe_dock(now=3.15,drive_state="DONE",docked=True)
        with self.assertRaises(DockHandoffError):
            self.m.finish_dock(now=3.2,dock_drive_terminated=True)
        self.quiet(3.3)
        done=self.m.finish_dock(now=4.2,dock_drive_terminated=True)
        self.assertTrue(done["geometric_docked"])
        self.assertFalse(done["electrical_charging_confirmed"])
        self.assertFalse(done["dustbin_emptied_confirmed"])
        self.assertFalse(done["mop_washed_confirmed"])
        self.assertEqual(self.m.output(now=4.21),Velocity())
        with self.assertRaises(DockHandoffError):
            self.m.claim_nav(now=4.22,nav2_idle=True,pose_fresh=True,
                             dock_process_inactive=True,resources_verified=False)

    def test_fake_latched_docked_bool_unproven(self):
        self.quiet()
        self.m.claim_dock(now=1.9,nav2_relinquished=True,
                          dock_drive_input_isolated=True)
        with self.assertRaises(DockHandoffError):
            self.m.observe_dock(now=2.0,drive_state="DONE",docked=True)

    def test_dock_drive_cannot_claim_without_mux_input_isolation(self):
        self.quiet()
        with self.assertRaises(DockHandoffError):
            self.m.claim_dock(now=1.9,nav2_relinquished=True,
                          dock_drive_input_isolated=False)

    def test_process_restart_resets_to_stop(self):
        self.nav()
        other=ExclusiveVelocityMux()
        self.assertEqual(other.output(now=2.),Velocity())
        self.assertEqual(other.owner,other.STOP)

    def test_invalid_sensor_and_unbounded_twist_fail_closed(self):
        self.nav()
        with self.assertRaises(DockHandoffError):
            self.m.accept_twist(self.m.NAV,Velocity(float('nan'),0.),now=2.)
        self.assertEqual(self.m.owner,self.m.STOP)
        self.quiet(3.)
        self.m.claim_nav(now=4.,nav2_idle=True,pose_fresh=True,
             dock_process_inactive=True,resources_verified=True)
        with self.assertRaises(DockHandoffError):
            self.m.accept_twist(self.m.NAV,Velocity(2.0,0.),now=4.1)
        self.assertEqual(self.m.owner,self.m.STOP)

    def test_backwards_monotonic_clock_fails_closed(self):
        self.nav()
        with self.assertRaises(DockHandoffError):
            self.m.output(now=1.)
        self.assertEqual(self.m.owner,self.m.STOP)

    def test_odom_motion_resets_quiet_timer(self):
        self.quiet()
        self.m.observe_odom(now=2.,linear_mps=.3,angular_rps=0.)
        with self.assertRaises(DockHandoffError):
            self.m.claim_dock(now=2.03,nav2_relinquished=True,
                              dock_drive_input_isolated=True)

    def test_dock_state_not_new_search_done_sequence_rejected(self):
        self.quiet()
        self.m.claim_dock(now=1.9,nav2_relinquished=True,
                          dock_drive_input_isolated=True)
        with self.assertRaises(DockHandoffError):
            self.m.observe_dock(now=2.,drive_state="DONE",docked=True)
        self.assertEqual(self.m.owner,self.m.DOCK)

if __name__=="__main__":
    unittest.main()
