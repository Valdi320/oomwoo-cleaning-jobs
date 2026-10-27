"""Fail-closed restart witness tests: never assert stop from cancel ACK alone."""
import unittest

from oomwoo_cleaning_jobs.restart_witness import OrphanNav2Witness,RestartNotSafe


OLD="a7"*16
OTHER="b9"*16


class RestartWitnessTests(unittest.TestCase):
    def setUp(self):
        self.w=OrphanNav2Witness(OLD)
        self.w.see_status(OLD,2)

    def cancel_and_terminal(self):
        self.w.cancel_response(0,(OLD,))
        self.w.see_status(OLD,5)

    def quiet(self,n=9,step=.18):
        for k in range(n):
            self.w.observe(sim_time=10+k*step,x=.0,y=.0,
                           linear_mps=0.,angular_rps=0.)

    def test_acknowledged_cancel_is_not_terminal(self):
        self.w.cancel_response(0,(OLD,))
        self.quiet()
        self.assertFalse(self.w.ready)
        with self.assertRaises(RestartNotSafe):self.w.require_ready()

    def test_terminal_and_bounded_quiet_pose_required(self):
        self.cancel_and_terminal()
        self.assertFalse(self.w.ready)
        self.quiet()
        self.assertTrue(self.w.ready)
        a=self.w.require_ready()
        self.assertEqual(a['known_old_goal_uuid'],OLD)
        self.assertTrue(a['verified_disarmed_stop'])

    def test_missing_observed_active_goal_refuses_recovery(self):
        w=OrphanNav2Witness(OLD)
        with self.assertRaises(RestartNotSafe):
            w.cancel_response(0,(OLD,))

    def test_unrecognized_cancel_response_refuses_recovery(self):
        with self.assertRaises(RestartNotSafe):
            self.w.cancel_response(0,(OTHER,))
        self.w.see_status(OLD,5)
        self.quiet()
        self.assertFalse(self.w.ready)

    def test_bad_service_return_code_refuses(self):
        with self.assertRaises(RestartNotSafe):
            self.w.cancel_response(1,(OLD,))
        self.assertFalse(self.w.ready)

    def test_status_of_other_uuid_does_not_unlock(self):
        self.w.cancel_response(0,(OLD,))
        self.w.see_status(OTHER,5)
        self.quiet()
        self.assertFalse(self.w.ready)

    def test_goal_succeeded_instead_of_cancel_forbids_rearm(self):
        self.w.cancel_response(0,(OLD,))
        self.w.see_status(OLD,4)
        self.quiet()
        self.assertFalse(self.w.ready)
        self.assertTrue(self.w.fault)

    def test_goal_aborted_instead_of_cancel_forbids_rearm(self):
        self.w.cancel_response(0,(OLD,))
        self.w.see_status(OLD,6)
        self.quiet()
        self.assertFalse(self.w.ready)

    def test_motion_resets_quiet_window(self):
        self.cancel_and_terminal()
        self.quiet(n=5)
        self.w.observe(sim_time=11.0,x=.15,y=0.,linear_mps=.25,angular_rps=0.)
        self.assertFalse(self.w.ready)
        for k in range(10):
            self.w.observe(sim_time=11.2+k*.18,x=.15,y=.0,
                           linear_mps=0.,angular_rps=0.)
        self.assertTrue(self.w.ready)

    def test_bad_odom_nan_causes_permanent_fault(self):
        self.cancel_and_terminal()
        self.w.observe(sim_time=float('nan'),x=0.,y=0.,
                       linear_mps=0.,angular_rps=0.)
        self.quiet()
        self.assertFalse(self.w.ready)

    def test_sim_clock_rollback_causes_permanent_fault(self):
        self.cancel_and_terminal()
        self.w.observe(sim_time=10.,x=0.,y=0.,linear_mps=0.,angular_rps=0.)
        self.w.observe(sim_time=9.,x=0.,y=0.,linear_mps=0.,angular_rps=0.)
        self.quiet()
        self.assertFalse(self.w.ready)

    def test_paused_clock_never_satisfies_min_quiet(self):
        self.cancel_and_terminal()
        for i in range(500):
            self.w.observe(sim_time=10.,x=0.,y=0.,linear_mps=0.,angular_rps=0.)
        self.assertFalse(self.w.ready)

    def test_sparse_sim_samples_do_not_prove_continuity(self):
        self.cancel_and_terminal()
        for k in range(20):
            self.w.observe(sim_time=k*1.0,x=0.,y=0.,linear_mps=0.,angular_rps=0.)
        self.assertFalse(self.w.ready)

    def test_position_drift_resets_window(self):
        self.cancel_and_terminal()
        for k in range(9):
            self.w.observe(sim_time=10+k*.18,x=k*.03,y=0.,
                           linear_mps=0.,angular_rps=0.)
        self.assertFalse(self.w.ready)

    def test_invalid_uuid_rejected(self):
        for x in ('','q'*32,'A'*32,'0'*31,None):
            with self.subTest(x=x),self.assertRaises(ValueError):
                OrphanNav2Witness(x)

    def test_terminal_before_service_callback_requires_confirmed_ack(self):
        self.w.see_status(OLD,5)
        self.quiet()
        self.assertFalse(self.w.ready)
        self.w.cancel_response(0,(OLD,))
        self.assertFalse(self.w.ready)  # quiet samples before ACK do not count
        self.quiet()
        self.assertTrue(self.w.ready)

    def test_cancel_rejected_after_early_terminal_never_rearms(self):
        self.w.see_status(OLD,5)
        with self.assertRaises(RestartNotSafe):
            self.w.cancel_response(3,())
        self.quiet()
        self.assertFalse(self.w.ready)

    def test_unknown_status_code_is_fault(self):
        self.w.cancel_response(0,(OLD,))
        self.w.see_status(OLD,77)
        self.quiet()
        self.assertFalse(self.w.ready)

    def test_angular_motion_above_threshold_resets_window(self):
        self.cancel_and_terminal()
        for k in range(9):
            self.w.observe(sim_time=10+k*.18,x=0.,y=0.,
                           linear_mps=0.,angular_rps=.17)
        self.assertFalse(self.w.ready)

    def test_missing_samples_do_not_pass(self):
        self.cancel_and_terminal()
        for k in range(4):
            self.w.observe(sim_time=10+k*.43,x=0.,y=0.,
                           linear_mps=0.,angular_rps=0.)
        self.assertFalse(self.w.ready)

if __name__=="__main__":unittest.main()
