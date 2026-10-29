"""Pure Python frame checks; frame strings are not authenticated."""
import unittest
from oomwoo_cleaning_jobs.ros_odom_frame_gate import (
    OdomFrameError,require_odom_frames,ODOM_PARENT,ODOM_CHILD)


class TestOdomFrameBinding(unittest.TestCase):
    def test_expected_oomwoo_frames_only(self):
        self.assertIsNone(require_odom_frames("odom","base_footprint"))
        self.assertEqual((ODOM_PARENT,ODOM_CHILD),("odom","base_footprint"))

    def test_wrong_pose_frame_rejected(self):
        for value in ("map","world","base_link","/odom","robot/odom",
                      "odom "," Odom",""):
            with self.subTest(frame=value):
                with self.assertRaises(OdomFrameError):
                    require_odom_frames(value,"base_footprint")

    def test_wrong_twist_frame_rejected(self):
        for value in ("map","base_link","base_scan","/base_footprint",
                      "robot/base_footprint","base_footprint ",""):
            with self.subTest(frame=value):
                with self.assertRaises(OdomFrameError):
                    require_odom_frames("odom",value)

    def test_missing_and_wrong_type_rejected(self):
        for value in (None,False,True,12,[],{},b"odom"):
            with self.subTest(value=repr(value)):
                with self.assertRaises(OdomFrameError):
                    require_odom_frames(value,"base_footprint")
                with self.assertRaises(OdomFrameError):
                    require_odom_frames("odom",value)

    def test_no_implicit_alias_or_normalization(self):
        for parent,child in (("odom","base_link"),("odom","base_footprint\n"),
                             ("odom\x00","base_footprint"),("ODOM","base_footprint")):
            with self.subTest(pair=(parent,child)):
                with self.assertRaises(OdomFrameError):
                    require_odom_frames(parent,child)

    def test_recovery_requires_explicit_correct_frame(self):
        with self.assertRaises(OdomFrameError):
            require_odom_frames("map","base_footprint")
        self.assertIsNone(require_odom_frames("odom","base_footprint"))


if __name__=="__main__":
    unittest.main()
