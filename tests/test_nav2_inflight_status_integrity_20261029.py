"""Pure-Python regression for ambiguous Nav2 in-flight GoalStatusArray.

This file does not import ROS2, connect to a robot or run Gazebo.
"""
import ast
from pathlib import Path
import unittest

from oomwoo_cleaning_jobs.nav_lease_witness import only_own_nav2_goal_active

OWN = "ab" * 16
FOREIGN = "cd" * 16

class InflightStatusIntegrityTests(unittest.TestCase):
    def test_single_own_active_with_terminal_history_allowed(self):
        for state in (1, 2, 3):
            with self.subTest(state=state):
                self.assertTrue(only_own_nav2_goal_active(((FOREIGN, 4), (OWN, state)), OWN))

    def test_foreign_active_alongside_own_is_blocked(self):
        self.assertFalse(only_own_nav2_goal_active(((OWN, 2), (FOREIGN, 1)), OWN))

    def test_duplicate_uuid_conflicting_terminal_and_active_is_blocked(self):
        self.assertFalse(only_own_nav2_goal_active(((OWN, 4), (OWN, 2)), OWN))

    def test_unknown_foreign_status_alongside_own_is_blocked(self):
        self.assertFalse(only_own_nav2_goal_active(((FOREIGN, 0), (OWN, 2)), OWN))

    def test_wrong_unknown_and_absent_own_status_blocked(self):
        for data in ((), ((FOREIGN, 2),), ((OWN, 4),), ((OWN, 7),)):
            with self.subTest(data=data):
                self.assertFalse(only_own_nav2_goal_active(data, OWN))

    def test_malformed_statuses_or_goal_id_blocked(self):
        for data in ([ (OWN,2) ], ((OWN, "2"),), ((OWN, True),),
                     (("zz"*16, 2),), ((OWN, 2, "extra"),)):
            with self.subTest(data=data):
                self.assertFalse(only_own_nav2_goal_active(data, OWN))
        for goal in (None, "bad", "AB"*16, True):
            with self.subTest(goal=goal):
                self.assertFalse(only_own_nav2_goal_active(((OWN, 2),), goal))

    def test_foreign_terminal_history_is_not_confused_with_active(self):
        self.assertTrue(only_own_nav2_goal_active(((FOREIGN, 5), (OWN, 2)), OWN))
        self.assertFalse(only_own_nav2_goal_active(((FOREIGN, 3), (OWN, 5)), OWN))

    def test_experimental_node_uses_strict_helper_in_lifecycle_fallback(self):
        node = Path(__file__).resolve().parents[1] / "src/oomwoo_cleaning_jobs/ros_experimental_node.py"
        tree = ast.parse(node.read_text())
        method = next(n for n in ast.walk(tree)
                      if isinstance(n, ast.FunctionDef) and n.name == "_nav2_execution_healthy")
        calls = [n for n in ast.walk(method) if isinstance(n, ast.Call)]
        self.assertTrue(any(isinstance(n.func, ast.Name)
                            and n.func.id == "only_own_nav2_goal_active" for n in calls))
        self.assertFalse(any(isinstance(n.func, ast.Name) and n.func.id == "any" for n in calls))

if __name__ == "__main__":
    unittest.main()
