"""Synthetic, deterministic three-way cmd_vel owner and watchdog tests."""
from dataclasses import replace
import unittest

from oomwoo_cleaning_jobs.dock_handoff import Velocity
from oomwoo_cleaning_jobs.reactive_velocity import (
    MotionHealth, ReactiveMuxError, ThreeWayVelocityMux)


class ThreeWayVelocityMuxTests(unittest.TestCase):
    def setUp(self):
        self.m = ThreeWayVelocityMux()

    def proof(self, **overrides):
        result = MotionHealth(
            pose_fresh=True, resources_ok=True, map_current=True,
            safety_loop_ok=True, isolated_sources=True,
            nav2_idle=True, nav2_active_goal=None,
            dock_inactive=True, sweep_inactive=True)
        return replace(result, **overrides)

    def quiet(self, t0=1.0):
        for i in range(8):
            self.m.observe_odom(
                now=round(t0 + i * .12, 5), linear_mps=0., angular_rps=0.)

    def arm_sweep(self):
        self.quiet()
        self.m.observe_health(now=1.85, proof=self.proof(sweep_inactive=False))
        return self.m.claim_sweep(
            now=1.9, nav_released=True, prior_terminal_uuid=None)

    def live_sweep(self):
        token = self.arm_sweep()
        self.m.observe_odom(now=1.92, linear_mps=0., angular_rps=0.)
        self.m.observe_health(now=1.93, proof=self.proof(sweep_inactive=False))
        self.m.accept_twist(source=self.m.SWEEP, token=token, now=1.94,
                            velocity=Velocity(.12, .10))
        return token

    def arm_nav(self):
        self.quiet()
        self.m.observe_health(now=1.85, proof=self.proof())
        return self.m.claim_transit(now=1.9, nav_goal_uuid="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")

    def nav_running(self):
        token = self.arm_nav()
        self.m.observe_health(now=1.92, proof=self.proof(
            nav2_idle=False, nav2_active_goal="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"))
        self.m.observe_odom(now=1.93, linear_mps=0., angular_rps=0.)
        self.m.accept_twist(source=self.m.NAV, token=token, now=1.94,
                            velocity=Velocity(.1, 0.))
        return token

    def test_01_cold_start_is_stop(self):
        self.assertEqual(self.m.output(now=0), Velocity())
        self.assertEqual(self.m.owner, self.m.STOP)

    def test_02_parameters_must_be_bounded_numeric(self):
        for kwargs in ({"max_age_s":float("nan")},
                       {"quiet_required_s":True},
                       {"max_linear_mps":1.0},
                       {"max_angular_rps":float("inf")}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                ThreeWayVelocityMux(**kwargs)

    def test_03_requires_quiet_odom_and_fresh_health(self):
        self.m.observe_health(now=.1, proof=self.proof())
        with self.assertRaises(ReactiveMuxError):
            self.m.claim_sweep(now=.11, nav_released=True,
                               prior_terminal_uuid=None)

    def test_04_sweep_rejects_active_nav_goal(self):
        self.quiet()
        self.m.observe_health(now=1.85, proof=self.proof(
            nav2_idle=False, nav2_active_goal="foreign"))
        with self.assertRaises(ReactiveMuxError):
            self.m.claim_sweep(now=1.9, nav_released=True,
                               prior_terminal_uuid=None)

    def test_05_sweep_cannot_arm_without_current_isolation(self):
        self.quiet()
        self.m.observe_health(now=1.85, proof=self.proof(
            isolated_sources=False))
        with self.assertRaises(ReactiveMuxError):
            self.m.claim_sweep(now=1.9, nav_released=True,
                               prior_terminal_uuid=None)

    def test_06_sweep_good_lease_emits_bounded_command(self):
        token = self.live_sweep()
        self.assertEqual(self.m.owner, self.m.SWEEP)
        self.assertEqual(len(token), 32)
        self.assertEqual(self.m.output(now=1.95), Velocity(.12, .10))

    def test_07_foreign_nonzero_nav_command_forces_stop(self):
        self.live_sweep()
        with self.assertRaises(ReactiveMuxError):
            self.m.accept_twist(source=self.m.NAV, token="foreign", now=1.95,
                                velocity=Velocity(.1, 0.))
        self.assertEqual(self.m.owner, self.m.STOP)
        self.assertEqual(self.m.output(now=1.96), Velocity())

    def test_08_foreign_zero_does_not_take_output(self):
        self.live_sweep()
        with self.assertRaises(ReactiveMuxError):
            self.m.accept_twist(source=self.m.DOCK, token="wrong", now=1.95,
                                velocity=Velocity())
        self.assertEqual(self.m.output(now=1.96), Velocity(.12, .10))

    def test_09_replayed_old_token_fails_closed(self):
        self.live_sweep()
        with self.assertRaises(ReactiveMuxError):
            self.m.accept_twist(source=self.m.SWEEP, token="old", now=1.95,
                                velocity=Velocity(.1, 0.))
        self.assertEqual(self.m.owner, self.m.STOP)

    def test_10_expired_velocity_lease_is_stop(self):
        self.live_sweep()
        self.m.observe_health(now=2.2, proof=self.proof(sweep_inactive=False))
        self.m.observe_odom(now=2.21, linear_mps=.1, angular_rps=0.)
        self.assertEqual(self.m.output(now=2.25), Velocity())
        self.assertEqual(self.m.owner, self.m.STOP)

    def test_11_expired_health_is_stop(self):
        self.live_sweep()
        self.m.observe_odom(now=2.24, linear_mps=.1, angular_rps=0.)
        self.assertEqual(self.m.output(now=2.26), Velocity())
        self.assertEqual(self.m.owner, self.m.STOP)

    def test_12_expired_odom_is_stop(self):
        self.live_sweep()
        self.m.observe_health(now=2.25, proof=self.proof(sweep_inactive=False))
        self.assertEqual(self.m.output(now=2.26), Velocity())
        self.assertEqual(self.m.owner, self.m.STOP)

    def test_13_map_change_during_sweep_forces_stop(self):
        self.live_sweep()
        with self.assertRaises(ReactiveMuxError):
            self.m.observe_health(now=1.95, proof=self.proof(
                sweep_inactive=False, map_current=False))
        self.assertEqual(self.m.output(now=1.96), Velocity())

    def test_14_low_resources_during_sweep_forces_stop(self):
        self.live_sweep()
        with self.assertRaises(ReactiveMuxError):
            self.m.observe_health(now=1.95, proof=self.proof(
                sweep_inactive=False, resources_ok=False))
        self.assertEqual(self.m.owner, self.m.STOP)

    def test_15_badly_typed_boolean_health_rejected(self):
        self.live_sweep()
        with self.assertRaises(ReactiveMuxError):
            self.m.observe_health(now=1.95, proof=self.proof(
                safety_loop_ok=1))
        self.assertEqual(self.m.owner, self.m.STOP)

    def test_16_nonfinite_twist_forces_stop(self):
        token = self.live_sweep()
        with self.assertRaises(ReactiveMuxError):
            self.m.accept_twist(source=self.m.SWEEP, token=token, now=1.95,
                                velocity=Velocity(float("nan"), 0.))
        self.assertEqual(self.m.owner, self.m.STOP)

    def test_17_excessive_twist_forces_stop(self):
        token = self.live_sweep()
        with self.assertRaises(ReactiveMuxError):
            self.m.accept_twist(source=self.m.SWEEP, token=token, now=1.95,
                                velocity=Velocity(.3, 0.))
        self.assertEqual(self.m.owner, self.m.STOP)

    def test_18_backwards_clock_forces_stop(self):
        self.live_sweep()
        with self.assertRaises(ReactiveMuxError):
            self.m.output(now=1.)
        self.assertEqual(self.m.owner, self.m.STOP)

    def test_19_nav_uuid_mismatch_refuses_handoff(self):
        self.nav_running()
        self.m.observe_health(now=1.95, proof=self.proof())
        with self.assertRaises(ReactiveMuxError):
            self.m.release_transit(now=1.96, terminal_uuid="wrong",
                                   terminal_verified=True, nav_input_isolated=True)
        self.assertEqual(self.m.owner, self.m.STOP)

    def test_20_transit_to_sweep_requires_fresh_quiet_and_exact_terminal(self):
        self.nav_running()
        self.m.observe_health(now=1.95, proof=self.proof())
        self.assertEqual(self.m.output(now=1.96), Velocity())
        self.m.release_transit(now=1.97, terminal_uuid="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                               terminal_verified=True, nav_input_isolated=True)
        with self.assertRaises(ReactiveMuxError):
            self.m.claim_sweep(now=1.98, nav_released=True,
                               prior_terminal_uuid="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        self.quiet(3.)
        self.m.observe_health(now=3.85, proof=self.proof(sweep_inactive=False))
        with self.assertRaises(ReactiveMuxError):
            self.m.claim_sweep(now=3.86, nav_released=True,
                               prior_terminal_uuid="stale")
        token = self.m.claim_sweep(now=3.9, nav_released=True,
                                   prior_terminal_uuid="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        self.assertEqual(self.m.owner, self.m.SWEEP)
        self.assertEqual(len(token), 32)

    def test_21_consumed_terminal_cannot_be_replayed(self):
        self.nav_running()
        self.m.observe_health(now=1.95, proof=self.proof())
        self.m.release_transit(now=1.97, terminal_uuid="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                               terminal_verified=True, nav_input_isolated=True)
        self.quiet(3.)
        self.m.observe_health(now=3.85, proof=self.proof(sweep_inactive=False))
        token = self.m.claim_sweep(now=3.9, nav_released=True,
                                   prior_terminal_uuid="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        self.m.release_non_nav(now=3.91, token=token,
                               source_stopped=True, source_isolated=True)
        self.quiet(5.)
        self.m.observe_health(now=5.85, proof=self.proof())
        with self.assertRaises(ReactiveMuxError):
            self.m.claim_dock(now=5.9, nav_released=True,
                              prior_terminal_uuid="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")

    def test_22_unverified_sweep_release_forces_stop(self):
        token = self.live_sweep()
        with self.assertRaises(ReactiveMuxError):
            self.m.release_non_nav(now=1.95, token=token,
                                   source_stopped=False, source_isolated=True)
        self.assertEqual(self.m.owner, self.m.STOP)

    def test_23_dock_and_sweep_are_mutually_exclusive(self):
        self.quiet()
        self.m.observe_health(now=1.85, proof=self.proof(
            dock_inactive=False, sweep_inactive=True))
        token = self.m.claim_dock(now=1.9, nav_released=True,
                                  prior_terminal_uuid=None)
        self.assertEqual(self.m.owner, self.m.DOCK)
        with self.assertRaises(ReactiveMuxError):
            self.m.claim_sweep(now=1.91, nav_released=True,
                               prior_terminal_uuid=None)
        self.m.observe_odom(now=1.92, linear_mps=0., angular_rps=0.)
        self.m.observe_health(now=1.93, proof=self.proof(dock_inactive=False))
        self.m.accept_twist(source=self.m.DOCK, token=token,
                            now=1.94, velocity=Velocity(-.1, 0.))
        self.assertEqual(self.m.output(now=1.95), Velocity(-.1, 0.))

    def test_24_process_restart_has_no_live_lease(self):
        self.live_sweep()
        new = ThreeWayVelocityMux()
        self.assertEqual(new.output(now=1.95), Velocity())
        self.assertIsNone(new.token)

    def test_25_nonzero_command_without_owner_fails(self):
        with self.assertRaises(ReactiveMuxError):
            self.m.accept_twist(source=self.m.SWEEP, token="fake",
                                now=0, velocity=Velocity(.1, 0))
        self.assertEqual(self.m.output(now=.1), Velocity())

    def test_26_invalid_odometry_forces_stop(self):
        self.live_sweep()
        with self.assertRaises(ReactiveMuxError):
            self.m.observe_odom(now=1.95, linear_mps=float("inf"), angular_rps=0)
        self.assertEqual(self.m.owner, self.m.STOP)

    def test_27_foreign_nav2_goal_during_transit_forces_stop(self):
        self.nav_running()
        with self.assertRaises(ReactiveMuxError):
            self.m.observe_health(now=1.95, proof=self.proof(
                nav2_idle=False, nav2_active_goal="foreign"))
        self.assertEqual(self.m.owner, self.m.STOP)

    def test_28_nav_output_needs_matching_active_uuid(self):
        token = self.arm_nav()
        self.m.observe_odom(now=1.91, linear_mps=0, angular_rps=0)
        self.m.accept_twist(source=self.m.NAV, token=token,
                            now=1.92, velocity=Velocity(.10, 0))
        self.assertEqual(self.m.output(now=1.93), Velocity())
        self.m.observe_health(now=1.94, proof=self.proof(
            nav2_idle=False, nav2_active_goal="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"))
        self.assertEqual(self.m.output(now=1.95), Velocity(.10, 0))

    def test_29_idle_after_active_nav_cannot_replay_motion(self):
        self.nav_running()
        self.assertEqual(self.m.output(now=1.95), Velocity(.1, 0))
        self.m.observe_health(now=1.96, proof=self.proof())
        self.m.accept_twist(source=self.m.NAV, token=self.m.token, now=1.97,
                            velocity=Velocity(.2, 0))
        self.assertEqual(self.m.output(now=1.98), Velocity())

    def test_31_reject_malformed_nav_uuid(self):
        self.quiet()
        self.m.observe_health(now=1.85, proof=self.proof())
        for value in ("abc", "AB" * 16, "", None, True):
            with self.subTest(value=value), self.assertRaises(ReactiveMuxError):
                self.m.claim_transit(now=1.9, nav_goal_uuid=value)
        self.assertEqual(self.m.owner, self.m.STOP)

    def test_32_terminal_without_observed_active_uuid_rejected(self):
        self.arm_nav()
        with self.assertRaises(ReactiveMuxError):
            self.m.release_transit(now=1.91,
                terminal_uuid="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
                terminal_verified=True, nav_input_isolated=True)
        self.assertEqual(self.m.owner, self.m.STOP)

    def test_33_unhashable_source_is_fail_closed(self):
        self.live_sweep()
        with self.assertRaises(ReactiveMuxError):
            self.m.accept_twist(source=[], token="a", now=1.95,
                                velocity=Velocity(.1, 0))
        self.assertEqual(self.m.output(now=1.96), Velocity())

    def test_30_missing_nav_idle_reconciliation_blocks_cold_sweep(self):
        self.quiet()
        self.m.observe_health(now=1.85, proof=self.proof(sweep_inactive=False))
        with self.assertRaises(ReactiveMuxError):
            self.m.claim_sweep(now=1.9, nav_released=False,
                               prior_terminal_uuid=None)


if __name__ == "__main__":
    unittest.main()
