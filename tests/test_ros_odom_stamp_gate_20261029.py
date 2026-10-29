"""Pure Python stamp policy regression (not ROS2 / hardware evidence)."""
import unittest
from oomwoo_cleaning_jobs.ros_odom_stamp_gate import OdomStampError,OdomStampGate


def send(gate, now, stamp):
    return gate.accept(ros_now_ns=now,
                       stamp_sec=stamp // 1_000_000_000,
                       stamp_nanosec=stamp % 1_000_000_000)


class TestROSStampedOdometry(unittest.TestCase):
    def setUp(self):
        self.gate = OdomStampGate()
        self.now = 10_000_000_000

    def test_fresh_sample_and_positive_age(self):
        self.assertEqual(send(self.gate,self.now,self.now-100_000_000),
                         100_000_000)

    def test_small_future_lead_is_bounded(self):
        self.assertEqual(send(self.gate,self.now,self.now+30_000_000),
                         -30_000_000)

    def test_boundary_past_age_allowed(self):
        self.assertEqual(send(self.gate,self.now,self.now-350_000_000),
                         350_000_000)

    def test_boundary_future_lead_allowed(self):
        self.assertEqual(send(self.gate,self.now,self.now+50_000_000),
                         -50_000_000)

    def test_stale_sample_denied(self):
        with self.assertRaises(OdomStampError):
            send(self.gate,self.now,self.now-350_000_001)

    def test_future_sample_denied(self):
        with self.assertRaises(OdomStampError):
            send(self.gate,self.now,self.now+50_000_001)

    def test_duplicate_stamp_denied_even_if_receipt_newer(self):
        send(self.gate,self.now,self.now)
        with self.assertRaises(OdomStampError):
            send(self.gate,self.now+100_000_000,self.now)

    def test_out_of_order_denied(self):
        send(self.gate,self.now,self.now-90_000_000)
        with self.assertRaises(OdomStampError):
            send(self.gate,self.now+20_000_000,self.now-95_000_000)

    def test_clock_rewind_denied_despite_newer_stamp(self):
        send(self.gate,self.now,self.now-100_000_000)
        with self.assertRaises(OdomStampError):
            send(self.gate,self.now-1,self.now-50_000_000)

    def test_rejected_sample_does_not_advance_sequence(self):
        send(self.gate,self.now,self.now-100_000_000)
        with self.assertRaises(OdomStampError):
            send(self.gate,self.now+500_000_000,self.now-2_000_000_000)
        self.assertEqual(send(self.gate,self.now+100_000_000,
                              self.now+90_000_000),10_000_000)

    def test_zero_stamp_denied(self):
        with self.assertRaises(OdomStampError):
            send(self.gate,self.now,0)

    def test_zero_clock_denied(self):
        with self.assertRaises(OdomStampError):
            send(self.gate,0,1)

    def test_invalid_types_denied(self):
        for kwargs in ({"ros_now_ns":True,"stamp_sec":9,"stamp_nanosec":1},
                       {"ros_now_ns":10,"stamp_sec":False,"stamp_nanosec":0},
                       {"ros_now_ns":10,"stamp_sec":1,"stamp_nanosec":False},
                       {"ros_now_ns":None,"stamp_sec":1,"stamp_nanosec":0},
                       {"ros_now_ns":10,"stamp_sec":1,"stamp_nanosec":1.5}):
            with self.subTest(kwargs=kwargs):
                with self.assertRaises(OdomStampError):
                    self.gate.accept(**kwargs)

    def test_invalid_nanoseconds_denied(self):
        for n in (-1,1_000_000_000):
            with self.assertRaises(OdomStampError):
                self.gate.accept(ros_now_ns=self.now,
                                 stamp_sec=9,stamp_nanosec=n)

    def test_invalid_configuration_denied(self):
        for kw in ({"max_past_ns":-1},{"max_past_ns":True},
                   {"max_past_ns":1_000_000_001},
                   {"max_future_ns":100_000_001},{"max_future_ns":False}):
            with self.subTest(kw=kw):
                with self.assertRaises(ValueError):OdomStampGate(**kw)

    def test_monotonic_progress_with_sim_clock_fast_forward(self):
        send(self.gate,self.now,self.now)
        self.assertEqual(send(self.gate,self.now+10_000_000_000,
                              self.now+10_000_000_000),0)

if __name__ == "__main__":
    unittest.main()
