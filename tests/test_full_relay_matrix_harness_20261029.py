"""Safety regression of disposable ROS2 full-relay startup test harness.

These tests are pure Python and verify harness configuration, not ROS2 runtime.
Actual Jazzy validation is an explicit separate offline integration command.
"""
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, ROOT / path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class TestFullRelayStartupHarness(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.runner = load(
            "integration/run_real_full_relay_startup_20261029.py",
            "real_relay_runner")
        cls.matrix = load(
            "integration/full_relay_matrix_container_20261029.py",
            "real_relay_matrix")

    def test_disposable_no_network_no_pull(self):
        argv = self.runner.ARGS
        for flag in ("--rm", "--pull=never", "--network=none",
                     "--cap-drop=ALL", "--read-only",
                     "--security-opt=no-new-privileges"):
            self.assertIn(flag, argv)

    def test_no_hardware_mount_and_readonly_checkout(self):
        argv = self.runner.ARGS
        self.assertNotIn("--device", argv)
        self.assertNotIn("--privileged", argv)
        self.assertIn("-v", argv)
        self.assertTrue(argv[argv.index("-v") + 1].endswith(":/work:ro"))

    def test_rosidl_build_is_local_and_reproducible(self):
        shell = self.runner.SHELL
        self.assertIn("source /opt/ros/jazzy/setup.bash", shell)
        self.assertIn("colcon build", shell)
        self.assertIn("/tmp/valdi_ros_ws", shell)
        self.assertNotIn("apt install", shell)
        self.assertNotIn("curl ", shell)
        self.assertNotIn("git clone", shell)

    def test_five_negative_two_positive_full_relay_scenarios(self):
        cases = dict(self.matrix.CASES)
        self.assertEqual(set(cases),
                         {"nominal", "namespace", "remap_motor",
                          "remap_controller", "remap_other",
                          "simulation_off", "override_motor"})
        for case in ("remap_motor", "remap_controller", "remap_other"):
            self.assertIn("--ros-args", cases[case])
        self.assertIn(":=/cmd_vel", " ".join(cases["remap_motor"]))
        self.assertIn(":=/diff_drive_controller/cmd_vel_unstamped",
                      " ".join(cases["remap_controller"]))

    def test_no_external_motor_topic_writes_in_probe(self):
        text = (ROOT / "integration" /
                "probe_full_relay_startup_20261029.py").read_text()
        self.assertIn("get_publishers_info_by_topic(\"/cmd_vel\")", text)
        self.assertIn("denied_before_timer", text)
        self.assertIn("all(v == (0.0, 0.0) for v in received)", text)


if __name__ == "__main__":
    unittest.main()
