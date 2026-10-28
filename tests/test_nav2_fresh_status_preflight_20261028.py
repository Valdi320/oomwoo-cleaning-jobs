"""Synthetic fresh Nav2 GoalStatusArray pre-send admission and wiring tests.

No ROS2, Gazebo, motor topics, battery or MCU contacted.
"""
import ast
from pathlib import Path
import unittest

from oomwoo_cleaning_jobs.nav_lease_witness import fresh_nav2_status_idle
from oomwoo_cleaning_jobs.nav_interlock import Goal
from oomwoo_cleaning_jobs.ros_nav2 import AsyncNav2Transport

UUIDA = "a1"*16
UUIDB = "b2"*16

class FakeFuture:
    def add_done_callback(self, _callback):
        pass

class FakeClient:
    def __init__(self):
        self.sent=[]
    def send_goal_async(self, request):
        self.sent.append(request)
        return FakeFuture()

class FreshStatusContractTests(unittest.TestCase):
    def fresh(self, statuses, received=99.9, now=100.0, max_age=2.5):
        return fresh_nav2_status_idle(
            statuses, received_s=received, now_s=now, max_age_s=max_age)

    def test_missing_observation_is_not_idle(self):
        self.assertFalse(self.fresh((), None))

    def test_fresh_empty_observation_is_idle_in_isolated_lab(self):
        self.assertTrue(self.fresh(()))

    def test_fresh_terminal_observation_is_idle(self):
        self.assertTrue(self.fresh(((UUIDA, 4), (UUIDB, 5))))

    def test_foreign_active_and_canceling_are_not_idle(self):
        for code in (1, 2, 3):
            with self.subTest(code=code):
                self.assertFalse(self.fresh(((UUIDA, code),)))

    def test_unknown_duplicate_or_malformed_snapshot_is_not_idle(self):
        for bad in (((UUIDA, 0),), ((UUIDA, 6),(UUIDA, 6)),
                    ((UUIDA, "4"),), ((UUIDA, 7),), [(UUIDA, 4)],
                    ((None, 4),), ((UUIDA, True),)):
            with self.subTest(bad=bad):
                self.assertFalse(self.fresh(bad))

    def test_stale_or_future_status_refused(self):
        for received in (97.0, 100.01, -1.0):
            with self.subTest(received=received):
                self.assertFalse(self.fresh((), received))

    def test_monotonic_age_bounds(self):
        self.assertTrue(self.fresh((), 97.5))
        self.assertFalse(self.fresh((), 97.4999))
        self.assertTrue(self.fresh((), 100.0))

    def test_untrusted_numeric_time_inputs_refused(self):
        for received, now, age in (
            (float("nan"), 100.0, 2.5), (float("inf"), 100.0, 2.5),
            (99.0, float("nan"), 2.5), (99.0, 100.0, 0),
            (99.0, 100.0, float("inf")), (True, 100.0, 2.5)):
            with self.subTest(values=(received, now, age)):
                self.assertFalse(self.fresh((), received, now, age))

class TransportAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.goal=Goal("boot1-1","unit",2,(0,1,2),"a"*64)

    def make_port(self, pre_send, builder=None):
        self.client=FakeClient()
        return AsyncNav2Transport(
            self.client, builder or (lambda g: ("POSE",g.target_cell)),
            on_terminal=lambda *_: None, on_fault=lambda *_: None,
            pre_send=pre_send)

    def test_foreign_goal_prevents_send_and_pending_mutation(self):
        snapshot=[((UUIDA, 2),)]
        def require_idle():
            if not fresh_nav2_status_idle(
                snapshot[0],received_s=99.9,now_s=100.0):
                raise RuntimeError("foreign Nav2 goal")
        port=self.make_port(require_idle)
        with self.assertRaises(RuntimeError):
            port.send_goal(self.goal)
        self.assertEqual(self.client.sent, [])
        self.assertEqual(port.outstanding_tokens(), ())
        snapshot[0]=()
        port.send_goal(self.goal)
        self.assertEqual(self.client.sent,[("POSE",2)])
        self.assertEqual(port.outstanding_tokens(),(self.goal.token,))

    def test_stale_status_prevents_send(self):
        port=self.make_port(lambda: (
            None if fresh_nav2_status_idle(
                (),received_s=90.0,now_s=100.0) else
            (_ for _ in ()).throw(RuntimeError("stale status"))))
        with self.assertRaises(RuntimeError):
            port.send_goal(self.goal)
        self.assertEqual(self.client.sent, [])

    def test_pre_send_is_checked_before_request_builder(self):
        events=[]
        def builder(_g):
            events.append("build")
            return "POSE"
        def blocked():
            events.append("preflight")
            raise RuntimeError("blocked")
        port=self.make_port(blocked,builder)
        with self.assertRaises(RuntimeError):
            port.send_goal(self.goal)
        self.assertEqual(events,["preflight"])
        self.assertEqual(self.client.sent,[])

    def test_invalid_pre_send_callback_refused(self):
        with self.assertRaises(ValueError):
            self.make_port(123)

    def test_ros2_node_checks_before_spot_and_room_dispatch_and_send(self):
        node_path=Path(__file__).resolve().parents[1]/"src/oomwoo_cleaning_jobs/ros_experimental_node.py"
        tree=ast.parse(node_path.read_text(encoding="utf-8"))
        methods={node.name:node for node in ast.walk(tree)
                 if isinstance(node,(ast.FunctionDef,ast.AsyncFunctionDef))}
        def guarded_before_dispatch(method):
            calls=[n for n in ast.walk(methods[method]) if isinstance(n,ast.Call)]
            gate=[n.lineno for n in calls if isinstance(n.func,ast.Attribute)
                  and n.func.attr=="_require_fresh_nav2_idle"]
            dispatch=[n.lineno for n in calls if isinstance(n.func,ast.Attribute)
                      and n.func.attr=="dispatch"]
            return bool(gate and dispatch and min(gate)<min(dispatch))
        self.assertTrue(guarded_before_dispatch("_dispatch"))
        self.assertTrue(guarded_before_dispatch("_dispatch_room"))
        attach=methods["_attach_job"]
        calls=[n for n in ast.walk(attach) if isinstance(n,ast.Call)]
        self.assertTrue(any(any(k.arg=="pre_send" for k in c.keywords)
                            for c in calls if isinstance(c.func,ast.Attribute)
                            and c.func.attr=="from_ros2"))
        start=methods["_handle"]
        self.assertTrue(sum(isinstance(n,ast.Call)
                  and isinstance(n.func,ast.Attribute)
                  and n.func.attr=="_require_fresh_nav2_idle"
                  for n in ast.walk(start)) >= 2)

if __name__=="__main__":
    unittest.main()
