"""Pure-Python opt-in Nav2 transit-only fail-closed tests."""
import inspect
import unittest
from dataclasses import replace
from oomwoo_cleaning_jobs.nav_interlock import Goal
from oomwoo_cleaning_jobs.ros_nav2 import AsyncNav2Transport

class FakeFuture:
    def add_done_callback(self, callback):
        pass

class FakeClient:
    def __init__(self): self.sent=[]
    def send_goal_async(self, payload):
        self.sent.append(payload)
        return FakeFuture()

class NeverJournal:
    def begin(self, **kwargs):
        raise AssertionError("journal mutated before intent rejection")

class Nav2TransitOnlyTests(unittest.TestCase):
    def setUp(self):
        self.goal=Goal("tok","job",7,(1,7),"a"*64)
        self.client=FakeClient()
        self.calls=[]
    def port(self, strict=True, journal=None):
        return AsyncNav2Transport(
            self.client,lambda g:("POSE",g.target_cell),
            on_terminal=lambda *a:None,on_fault=lambda *a:None,
            pre_send=lambda:self.calls.append("pre_send"),
            transit_only=strict,journal=journal)
    def test_legacy_default_still_dispatches(self):
        self.assertEqual(self.goal.purpose,"coverage")
        self.port(False).send_goal(self.goal)
        self.assertEqual(self.client.sent,[("POSE",7)])
    def test_reject_legacy_before_any_side_effect(self):
        p=self.port(journal=NeverJournal())
        with self.assertRaisesRegex(ValueError,"intent denied"):
            p.send_goal(self.goal)
        self.assertEqual(self.calls,[])
        self.assertEqual(self.client.sent,[])
        self.assertEqual(p.outstanding_tokens(),())
    def test_transit_and_dock_explicitly_allowed(self):
        p=self.port()
        p.send_goal(replace(self.goal,purpose="transit"))
        p.send_goal(replace(self.goal,token="tok2",purpose="dock"))
        self.assertEqual(self.client.sent,[("POSE",7),("POSE",7)])
        self.assertEqual(self.calls,["pre_send","pre_send"])
    def test_reject_unknown_malformed_and_case_variants(self):
        p=self.port()
        for purpose in ("sweep","localize","TRANSIT","transit ","",None,1,True,[],{}):
            with self.subTest(purpose=repr(purpose)):
                with self.assertRaises(ValueError):
                    p.send_goal(replace(self.goal,purpose=purpose))
        self.assertEqual(self.client.sent,[])
        self.assertEqual(self.calls,[])
    def test_reject_non_bool_toggle(self):
        for value in ("yes",1,None):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    AsyncNav2Transport(self.client,lambda g:g,
                        on_terminal=lambda *a:None,
                        on_fault=lambda *a:None,transit_only=value)
    def test_factory_exposes_transit_only(self):
        p=inspect.signature(AsyncNav2Transport.from_ros2).parameters
        self.assertIn("transit_only",p)
        self.assertIs(p["transit_only"].default,False)
    def test_goal_builder_is_not_called_before_gate(self):
        p=AsyncNav2Transport(self.client,
            lambda g:(_ for _ in ()).throw(AssertionError("unsafe builder")),
            on_terminal=lambda *a:None,on_fault=lambda *a:None,
            transit_only=True)
        with self.assertRaises(ValueError):p.send_goal(self.goal)
        self.assertEqual(self.client.sent,[])
    def test_old_positional_and_keyword_goal_signatures_work(self):
        self.assertEqual(Goal("t","j",1,(1,),"x").purpose,"coverage")
        self.assertEqual(Goal(token="t",job_id="j",target_cell=1,
                              candidate_cells=(1,),map_digest="x").purpose,"coverage")
if __name__=="__main__":unittest.main()
