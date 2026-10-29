"""Execute actual relay methods with fake ROS transport; no ROS2/DDS or motors.

Only the clock, publisher and unrelated Nav2 witnesses are stand-ins. The
battery callback, ownership admission, output tick and two-way mux are real
production code. Constructor, middleware and hardware are outside this test.
"""
import ast
import copy
import math
from pathlib import Path
from types import SimpleNamespace as N
import unittest

from oomwoo_cleaning_jobs.dock_handoff import (
    DockHandoffError, ExclusiveVelocityMux, Velocity)


SOURCE = (Path(__file__).resolve().parents[1] / "src" /
          "oomwoo_cleaning_jobs" / "ros_dock_velocity_relay.py")
UUID = "b6" * 16


def relay_methods(clock):
    """Load tested methods verbatim without importing or starting ROS2."""
    tree = ast.parse(SOURCE.read_text())
    source_class = next(node for node in tree.body
                        if isinstance(node, ast.ClassDef)
                        and node.name == "ExperimentalDockVelocityRelay")
    names = {"_battery", "_fresh", "_execute", "_tick", "_reset"}
    methods = [copy.deepcopy(node) for node in source_class.body
               if isinstance(node, ast.FunctionDef) and node.name in names]
    if {node.name for node in methods} != names:
        raise AssertionError("relay method contract changed; review the test")
    selected = ast.ClassDef(name="RelayUnderTest", bases=[], keywords=[],
                            body=methods, decorator_list=[])
    module = ast.fix_missing_locations(ast.Module(body=[selected], type_ignores=[]))
    namespace = {
        "time": N(monotonic=lambda: clock[0]), "math": math,
        "DockHandoffError": DockHandoffError, "Velocity": Velocity,
        "Twist": lambda: N(linear=N(x=0.), angular=N(z=0.)),
        "_ACTIVE": frozenset((1, 2, 3)),
    }
    exec(compile(module, str(SOURCE), "exec"), namespace)
    return namespace["RelayUnderTest"]()


class ROSRelayBatteryBoundsTests(unittest.TestCase):
    def make_relay(self):
        clock = [1.9]
        relay = relay_methods(clock)
        relay.mux = ExclusiveVelocityMux()
        for index in range(8):
            relay.mux.observe_odom(now=1. + .12*index,
                                   linear_mps=0., angular_rps=0.)
        relay._goal_witness = N(reset=lambda: None, fault=None, uuid=UUID,
            current_active_matches=lambda: True,
            can_release_for_dock=lambda: False)
        relay._nav_idle = lambda: True
        relay._nav_confirmed_terminal = lambda: True
        relay._odom_last = 1.84
        relay._dock_state = "IDLE"
        relay._dock_last = 1.84
        relay._last_status = 1.9
        relay._nav_status = ((UUID, 2),)
        relay._nav_lease_started = None
        relay._released_nav = False
        relay._battery_last = None
        relay._battery_fraction = None
        relay._latest_output = Velocity()
        outputs = []
        relay.safe = N(publish=outputs.append)
        return relay, clock, outputs

    def active_relay(self, owner):
        relay, clock, outputs = self.make_relay()
        relay._battery(N(percentage=.6))
        if owner == relay.mux.NAV:
            relay._execute({"operation": "claim_nav"})
        else:
            relay.mux.claim_dock(now=clock[0], nav2_relinquished=True,
                                 dock_drive_input_isolated=True)
        clock[0] = 2.
        relay.mux.accept_twist(owner, Velocity(.1, 0.), now=clock[0])
        relay._tick()
        self.assertEqual(outputs[-1].linear.x, .1)
        return relay, clock, outputs

    def test_percent_style_100_does_not_arm_navigation(self):
        relay, _, _ = self.make_relay()
        relay._battery(N(percentage=100.))
        with self.assertRaises(DockHandoffError):
            relay._execute({"operation": "claim_nav"})
        self.assertEqual(relay.mux.owner, relay.mux.STOP)

    def test_above_one_cannot_arm_even_with_other_proofs_healthy(self):
        relay, _, _ = self.make_relay()
        relay._battery(N(percentage=1.00001))
        with self.assertRaises(DockHandoffError):
            relay._execute({"operation": "claim_nav"})

    def test_invalid_high_battery_revokes_both_active_owners(self):
        for owner in (ExclusiveVelocityMux.NAV, ExclusiveVelocityMux.DOCK):
            for value in (1.00001, 100.):
                with self.subTest(owner=owner, battery=value):
                    relay, clock, outputs = self.active_relay(owner)
                    clock[0] = 2.01
                    relay._battery(N(percentage=value))
                    clock[0] = 2.02
                    relay._tick()
                    self.assertEqual(relay.mux.owner, relay.mux.STOP)
                    self.assertEqual(outputs[-1].linear.x, 0.)
                    self.assertEqual(outputs[-1].angular.z, 0.)

    def test_valid_admission_range_keeps_existing_boundaries(self):
        for value in (.20, .60, 1.):
            with self.subTest(battery=value):
                relay, _, _ = self.make_relay()
                relay._battery(N(percentage=value))
                relay._execute({"operation": "claim_nav"})
                self.assertEqual(relay.mux.owner, relay.mux.NAV)

    def test_low_and_nonfinite_values_remain_denied(self):
        for value in (-1., 0., .199, float("nan"), float("inf"), -float("inf")):
            with self.subTest(battery=value):
                relay, _, _ = self.make_relay()
                relay._battery(N(percentage=value))
                with self.assertRaises(DockHandoffError):
                    relay._execute({"operation": "claim_nav"})

    def test_valid_active_range_keeps_existing_stop_threshold(self):
        for owner in (ExclusiveVelocityMux.NAV, ExclusiveVelocityMux.DOCK):
            for value in (.15, .60, 1.):
                with self.subTest(owner=owner, battery=value):
                    relay, clock, outputs = self.active_relay(owner)
                    clock[0] = 2.01
                    relay._battery(N(percentage=value))
                    clock[0] = 2.02
                    relay._tick()
                    self.assertEqual(relay.mux.owner, owner)
                    self.assertEqual(outputs[-1].linear.x, .1)


if __name__ == "__main__":
    unittest.main()
