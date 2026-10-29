"""Ensures new DDS frame probe cannot bypass inherited isolation."""
from pathlib import Path
import unittest

ROOT=Path(__file__).resolve().parents[1]


class TestFrameHarness(unittest.TestCase):
    def test_offline_runner_inherits_existing_isolation(self):
        t=(ROOT/"integration"/"run_ros2_odom_frame_gate_20261029.py").read_text()
        self.assertIn("from run_real_full_relay_startup_20261029 import ARGS as BASE_ARGS",t)
        self.assertIn("ARGS = list(BASE_ARGS)",t)
        self.assertIn('result.get("total")==16',t)
        self.assertNotIn("--privileged",t)

    def test_dds_sender_only_odometry_no_control_claim(self):
        t=(ROOT/"integration"/"probe_ros2_odom_frames_20261029.py").read_text()
        self.assertIn('create_publisher(Odometry,"/odom",10)',t)
        self.assertIn("msg.child_frame_id = child_frame_id",t)
        self.assertIn('get_publishers_info_by_topic("/cmd_vel")',t)
        self.assertNotIn("create_publisher(Twist",t)
        for action in ("claim_nav(","claim_dock(","claim_sweep("):
            self.assertNotIn(action,t)
