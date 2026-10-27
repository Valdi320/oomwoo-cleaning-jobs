"""Bounded AMCL future-tolerance and stale-TF admission tests."""
import math,unittest
from oomwoo_cleaning_jobs.pose_time import require_fresh_amcl_pose,PoseTimeError

class PoseTimeTests(unittest.TestCase):
    def test_known_future_stamped_oomwoo_amcl_transform_accepted(self):
        self.assertAlmostEqual(require_fresh_amcl_pose(25.,30.),-5.)
    def test_fresh_past_is_accepted(self):
        self.assertAlmostEqual(require_fresh_amcl_pose(25.,23.3),1.7)
    def test_too_old_tf_rejected(self):
        with self.assertRaises(PoseTimeError):
            require_fresh_amcl_pose(25.,21.5)
    def test_excessively_future_dated_tf_rejected(self):
        with self.assertRaises(PoseTimeError):
            require_fresh_amcl_pose(25.,32.)
    def test_nan_inf_rejected(self):
        for age in (float("nan"),float("inf"),-float("inf")):
            with self.subTest(age=age),self.assertRaises(PoseTimeError):
                require_fresh_amcl_pose(age,0.)
    def test_invalid_policy_rejected(self):
        with self.assertRaises(PoseTimeError):
            require_fresh_amcl_pose(0.,0.,max_future_s=50.)
