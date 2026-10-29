"""Pure-Python pre-fix counterexamples and ROS2 adapter wiring regression.

No ROS2 runtime, motor, hardware or Gazebo is executed.
"""
import ast
from pathlib import Path
import unittest

from oomwoo_cleaning_jobs.restart_witness import OrphanNav2Witness

OLD = "a7"*16
FOREIGN = "b9"*16

def ready_witness():
    w=OrphanNav2Witness(OLD)
    w.see_status(OLD,2)
    w.cancel_response(0,(OLD,))
    w.see_status(OLD,5)
    for k in range(9):
        w.observe(sim_time=10+k*.18,x=0.,y=0.,linear_mps=0.,angular_rps=0.)
    assert w.ready
    return w

class OrphanStatusSnapshotTests(unittest.TestCase):
    def test_pre_fix_counterexamples_old_logic_passed(self):
        # Prior AsyncOrphanRecovery.step accepted whenever no FOREIGN active
        # UUIDs were present, even if the OLD terminal proof was missing.
        w=ready_witness()
        def historical_accept(statuses):
            foreign=[(u,code) for u,code in statuses
                     if code in (1,2,3) and u!=OLD]
            return w.ready and not foreign
        self.assertTrue(historical_accept(()))
        self.assertTrue(historical_accept(((FOREIGN,0),)))
        self.assertTrue(historical_accept(((OLD,2),(OLD,5))))
        self.assertTrue(historical_accept(((FOREIGN,4),)))

    def test_current_exact_terminal_required_for_rearm(self):
        w=ready_witness()
        self.assertFalse(w.final_snapshot_safe(()))
        self.assertFalse(w.final_snapshot_safe(((FOREIGN,4),)))
        self.assertFalse(w.final_snapshot_safe(((OLD,2),)))
        self.assertTrue(w.final_snapshot_safe(((OLD,5),)))
        self.assertTrue(w.final_snapshot_safe(((FOREIGN,4),(OLD,5))))

    def test_unknown_or_malformed_snapshot_blocks_rearm(self):
        for data in (
            ((OLD,5),(FOREIGN,0)), ((OLD,5),(FOREIGN,7)),
            ((OLD,5),(FOREIGN,True)), ((OLD,5),(FOREIGN,"4")),
            ((OLD,5),("QQ"*16,4)), ((OLD,5),(OLD,5)),
            ((OLD,5),(OLD,2)), [(OLD,5)], None,
            ((OLD,5,123),),
        ):
            with self.subTest(data=data):
                self.assertFalse(ready_witness().final_snapshot_safe(data))

    def test_foreign_active_at_any_time_is_latched_fault(self):
        w=OrphanNav2Witness(OLD)
        w.see_snapshot(((OLD,2),(FOREIGN,1)))
        self.assertIsNotNone(w.fault)
        w.see_snapshot(((OLD,5),))
        self.assertFalse(w.ready)

    def test_unknown_status_latches_fault_before_rearm(self):
        w=OrphanNav2Witness(OLD)
        w.see_snapshot(((FOREIGN,0),))
        self.assertIsNotNone(w.fault)
        self.assertFalse(w.final_snapshot_safe(((OLD,5),)))

    def test_terminal_resurrection_latches_fault(self):
        w=ready_witness()
        w.see_snapshot(((OLD,2),))
        self.assertIsNotNone(w.fault)
        self.assertFalse(w.final_snapshot_safe(((OLD,5),)))

    def test_valid_status_sequence_keeps_quiet_proof(self):
        w=OrphanNav2Witness(OLD)
        w.see_snapshot(((OLD,2),(FOREIGN,4)))
        w.cancel_response(0,(OLD,))
        w.see_snapshot(((OLD,5),(FOREIGN,4)))
        for k in range(9):
            w.observe(sim_time=10+k*.18,x=0.,y=0.,linear_mps=0.,angular_rps=0.)
        self.assertTrue(w.final_snapshot_safe(((OLD,5),(FOREIGN,4))))
        self.assertIsNone(w.fault)

    def test_ros2_adapter_wires_snapshot_validation_at_both_boundaries(self):
        src=Path(__file__).resolve().parents[1]/"src/oomwoo_cleaning_jobs/ros_restart.py"
        tree=ast.parse(src.read_text())
        funcs={n.name:n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)}
        def methods(name):
            return {n.func.attr for n in ast.walk(funcs[name])
                    if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)}
        self.assertIn("see_snapshot",methods("_on_status"))
        self.assertIn("final_snapshot_safe",methods("step"))
