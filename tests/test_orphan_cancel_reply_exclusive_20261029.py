"""Synthetic adversarial CancelGoal acknowledgments; no ROS2 runtime."""
import unittest
from oomwoo_cleaning_jobs.restart_witness import OrphanNav2Witness, RestartNotSafe
OLD="a7"*16
FOREIGN="b9"*16

def active():
    w=OrphanNav2Witness(OLD)
    w.see_status(OLD,2)
    return w

class ExactCancelAckTests(unittest.TestCase):
    def test_foreign_uuid_in_success_ack_is_rejected(self):
        w=active()
        with self.assertRaises(RestartNotSafe):
            w.cancel_response(0,(OLD,FOREIGN))
        self.assertIsNotNone(w.fault)
        w.see_status(OLD,5)
        for k in range(10):
            w.observe(sim_time=20+k*.18,x=0.,y=0.,linear_mps=0.,angular_rps=0.)
        self.assertFalse(w.ready)

    def test_duplicate_uuid_in_ack_is_rejected(self):
        w=active()
        with self.assertRaises(RestartNotSafe):
            w.cancel_response(0,(OLD,OLD))
        self.assertIsNotNone(w.fault)

    def test_boolean_zero_return_code_is_rejected(self):
        w=active()
        with self.assertRaises(RestartNotSafe):
            w.cancel_response(False,(OLD,))
        self.assertIsNotNone(w.fault)

    def test_non_tuple_ack_is_rejected(self):
        w=active()
        with self.assertRaises(RestartNotSafe):
            w.cancel_response(0,[OLD])
        self.assertIsNotNone(w.fault)

    def test_only_exact_goal_ack_allows_terminal_quiet(self):
        w=active()
        w.cancel_response(0,(OLD,))
        w.see_status(OLD,5)
        for k in range(9):
            w.observe(sim_time=10+k*.18,x=0.,y=0.,linear_mps=0.,angular_rps=0.)
        self.assertTrue(w.ready)
        self.assertEqual(w.require_ready()["known_old_goal_uuid"], OLD)

    def test_rejected_ack_does_not_upgrade_pre_ack_terminal(self):
        w=active()
        w.see_status(OLD,5)
        with self.assertRaises(RestartNotSafe):
            w.cancel_response(0,(OLD,FOREIGN))
        self.assertFalse(w.terminal_canceled)
        self.assertFalse(w.ready)

if __name__=="__main__":
    unittest.main()
