"""Harness isolation regression; real ROS2 proof must be run separately."""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


class TestOdomDDSRunner(unittest.TestCase):
    def test_runner_must_inherit_offline_lab_sandbox(self):
        src = (ROOT/"integration"/"run_ros2_stamped_odom_20261029.py").read_text()
        self.assertIn("from run_real_full_relay_startup_20261029 import ARGS as BASE_ARGS",src)
        self.assertIn("ARGS = list(BASE_ARGS)",src)
        self.assertIn("ROS2_ODOM_STAMP_PROBE",src)
        self.assertNotIn("--privileged",src)

    def test_probe_publishes_only_odom_without_control_claim(self):
        src = (ROOT/"integration"/"probe_ros2_stamped_odom_20261029.py").read_text()
        self.assertIn('create_publisher(Odometry,"/odom",10)',src)
        self.assertIn('get_publishers_info_by_topic("/cmd_vel")',src)
        self.assertNotIn('create_publisher(Twist',src)
        self.assertNotIn('claim_nav(',src)
        self.assertNotIn('claim_dock(',src)
        self.assertNotIn('claim_sweep(',src)

if __name__=="__main__":
    unittest.main()
