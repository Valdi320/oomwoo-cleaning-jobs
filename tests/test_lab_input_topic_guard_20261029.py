import pathlib
import unittest
from oomwoo_cleaning_jobs.lab_topic_guard import (
    LabTopicGuardError,LAB_INPUT_TOPICS,LAB_THREE_WAY_INPUT_TOPICS,require_lab_input_subscriptions)

class Fake:
    def __init__(self,topic): self.topic_name=topic
class Broken:
    @property
    def topic_name(self): raise RuntimeError("broken subscription")
def correct():return {name:Fake(name) for name in LAB_INPUT_TOPICS}

class TestInputIsolation(unittest.TestCase):
    def test_01_nominal(self):
        self.assertIsNone(require_lab_input_subscriptions(correct()))
    def test_02_nav_dock_alias(self):
        x=correct();x["/valdi/nav2_cmd_vel"]=Fake("/valdi/dock_drive_cmd_vel")
        with self.assertRaises(LabTopicGuardError):require_lab_input_subscriptions(x)
    def test_03_dock_nav_alias(self):
        x=correct();x["/valdi/dock_drive_cmd_vel"]=Fake("/valdi/nav2_cmd_vel")
        with self.assertRaises(LabTopicGuardError):require_lab_input_subscriptions(x)
    def test_04_wrong_odom_source(self):
        x=correct();x["/odom"]=Fake("/valdi/nav2_cmd_vel")
        with self.assertRaises(LabTopicGuardError):require_lab_input_subscriptions(x)
    def test_05_wrong_nav_status(self):
        x=correct();x["/navigate_to_pose/_action/status"]=Fake("/fake")
        with self.assertRaises(LabTopicGuardError):require_lab_input_subscriptions(x)
    def test_06_each_witness_rejected_when_remapped(self):
        for name in LAB_INPUT_TOPICS:
            x=correct();x[name]=Fake(name+"_foreign")
            with self.subTest(name=name),self.assertRaises(LabTopicGuardError):
                require_lab_input_subscriptions(x)
    def test_07_missing_extra(self):
        x=correct();x.pop("/battery_state")
        with self.assertRaises(LabTopicGuardError):require_lab_input_subscriptions(x)
        x=correct();x["/extra"]=Fake("/extra")
        with self.assertRaises(LabTopicGuardError):require_lab_input_subscriptions(x)
    def test_08_broken_nonstring_relative(self):
        for bad in (None,42,True,b"/odom","odom","/odom "):
            x=correct();x["/odom"]=Fake(bad)
            with self.subTest(bad=repr(bad)),self.assertRaises(LabTopicGuardError):
                require_lab_input_subscriptions(x)
        x=correct();x["/odom"]=Broken()
        with self.assertRaises(LabTopicGuardError):require_lab_input_subscriptions(x)
    def test_09_distinct_handles(self):
        x=correct();x["/battery_state"]=x["/odom"]
        with self.assertRaises(LabTopicGuardError):require_lab_input_subscriptions(x)
    def test_10_guard_before_timer(self):
        s=(pathlib.Path(__file__).parents[1]/"src"/
           "oomwoo_cleaning_jobs"/"ros_dock_velocity_relay.py").read_text()
        self.assertLess(s.index("require_lab_input_subscriptions(inputs)"),
                        s.index("self.create_service(ControlCleaning,"))
        self.assertLess(s.index("require_lab_input_subscriptions(inputs)"),
                        s.index("self.create_timer(.05,self._tick)"))
    def test_11_three_way_requires_all_fourteen_resolved_handles(self):
        x={name:Fake(name) for name in LAB_THREE_WAY_INPUT_TOPICS}
        self.assertEqual(len(x),14)
        self.assertIsNone(require_lab_input_subscriptions(x,three_way=True))
        for name in LAB_THREE_WAY_INPUT_TOPICS:
            wrong=dict(x);wrong[name]=Fake(name+"_foreign")
            with self.subTest(name=name),self.assertRaises(LabTopicGuardError):
                require_lab_input_subscriptions(wrong,three_way=True)
    def test_12_no_implicit_upgrade_or_missing_health(self):
        x={name:Fake(name) for name in LAB_THREE_WAY_INPUT_TOPICS}
        with self.assertRaises(LabTopicGuardError):require_lab_input_subscriptions(x)
        with self.assertRaises(LabTopicGuardError):
            require_lab_input_subscriptions(correct(),three_way=True)
if __name__=="__main__":unittest.main()
