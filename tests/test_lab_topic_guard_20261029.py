"""Pure Python regression for remapped experimental velocity publishers."""
import pathlib
import unittest

from oomwoo_cleaning_jobs.lab_topic_guard import (
    LAB_OUTPUT_TOPIC, LabTopicGuardError, require_lab_output_publisher)


class FakePublisher:
    def __init__(self, topic):
        self.topic_name = topic


class BrokenPublisher:
    @property
    def topic_name(self):
        raise RuntimeError("ROS publisher was destroyed")


class TestResolvedLabPublisher(unittest.TestCase):
    def test_01_exact_absolute_lab_topic_allowed(self):
        self.assertEqual(require_lab_output_publisher(FakePublisher(LAB_OUTPUT_TOPIC)),
                         LAB_OUTPUT_TOPIC)

    def test_02_cmd_vel_remap_rejected(self):
        with self.assertRaises(LabTopicGuardError):
            require_lab_output_publisher(FakePublisher("/cmd_vel"))

    def test_03_diff_drive_motor_alias_rejected(self):
        with self.assertRaises(LabTopicGuardError):
            require_lab_output_publisher(FakePublisher(
                "/diff_drive_controller/cmd_vel_unstamped"))

    def test_04_nearby_lab_prefix_rejected(self):
        with self.assertRaises(LabTopicGuardError):
            require_lab_output_publisher(FakePublisher(
                "/valdi/lab_safe_cmd_vel_output/other"))

    def test_05_other_namespace_rejected(self):
        with self.assertRaises(LabTopicGuardError):
            require_lab_output_publisher(FakePublisher(
                "/robot/valdi/lab_safe_cmd_vel_output"))

    def test_06_relative_name_rejected(self):
        with self.assertRaises(LabTopicGuardError):
            require_lab_output_publisher(FakePublisher("valdi/lab_safe_cmd_vel_output"))

    def test_07_non_string_or_missing_topic_rejected(self):
        for value in (None, b"/valdi/lab_safe_cmd_vel_output", 0, True, []):
            with self.subTest(value=value), self.assertRaises(LabTopicGuardError):
                require_lab_output_publisher(FakePublisher(value))
        with self.assertRaises(LabTopicGuardError):
            require_lab_output_publisher(object())

    def test_08_property_failure_rejected(self):
        with self.assertRaises(LabTopicGuardError):
            require_lab_output_publisher(BrokenPublisher())

    def test_09_whitespace_and_unicode_alias_rejected(self):
        for value in (" "+LAB_OUTPUT_TOPIC, LAB_OUTPUT_TOPIC+" ",
                      "/valdi//lab_safe_cmd_vel_output",
                      "／valdi/lab_safe_cmd_vel_output"):
            with self.subTest(value=value), self.assertRaises(LabTopicGuardError):
                require_lab_output_publisher(FakePublisher(value))

    def test_10_relay_calls_guard_before_creating_its_timer(self):
        relay = (pathlib.Path(__file__).parents[1] / "src" /
                 "oomwoo_cleaning_jobs" / "ros_dock_velocity_relay.py").read_text()
        make = relay.index("self.safe=self.create_publisher")
        check = relay.index("require_lab_output_publisher(self.safe)")
        timer = relay.index("self.create_timer(.05,self._tick)")
        self.assertLess(make, check)
        self.assertLess(check, timer)


if __name__ == "__main__":
    unittest.main()
