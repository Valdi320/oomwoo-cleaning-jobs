"""Synthetic anti-replay safety regression; no actual ROS2 or robot output."""
from dataclasses import replace
import unittest

from oomwoo_cleaning_jobs.dock_handoff import Velocity
from oomwoo_cleaning_jobs.reactive_velocity import MotionHealth, ReactiveMuxError
from oomwoo_cleaning_jobs.sequenced_motion_ingress import (
    EvidenceEnvelope, SequencedMotionMux)


def good_health(**kwargs):
    return replace(MotionHealth(
        pose_fresh=True, resources_ok=True, map_current=True,
        safety_loop_ok=True, isolated_sources=True, nav2_idle=True,
        nav2_active_goal=None, dock_inactive=True, sweep_inactive=True),
        **kwargs)


class TestSequencedEvidenceIngress(unittest.TestCase):
    def setUp(self):
        self.m = SequencedMotionMux(
            odom_source="odom:robot1", health_source="supervisor:robot1")
        self.odseq = 0
        self.hs = 0

    def odom(self, now, *, sample=None, sequence=None, source="odom:robot1"):
        self.odseq += 1
        self.m.observe_odom(
            now=now,
            envelope=EvidenceEnvelope(source, self.odseq if sequence is None else sequence,
                                      now - .01 if sample is None else sample),
            linear_mps=0., angular_rps=0.)

    def health(self, now, *, sample=None, sequence=None,
               source="supervisor:robot1", proof=None):
        self.hs += 1
        self.m.observe_health(
            now=now,
            envelope=EvidenceEnvelope(source, self.hs if sequence is None else sequence,
                                      now - .01 if sample is None else sample),
            proof=good_health(sweep_inactive=False) if proof is None else proof)

    def running(self):
        for i in range(8):
            self.odom(round(1. + i * .12, 5))
        self.health(1.85)
        token = self.m.claim_sweep(
            now=1.90, nav_released=True, prior_terminal_uuid=None)
        self.odom(1.92)
        self.health(1.93)
        self.m.accept_twist(source="reactive_sweep", token=token,
                            now=1.94, velocity=Velocity(.10, .02))
        self.assertEqual(self.m.output(now=1.95), Velocity(.10, .02))
        return token

    def assert_forced_stop(self, action):
        with self.assertRaises(ReactiveMuxError):
            action()
        self.assertEqual(self.m.owner, "stopped")
        self.assertEqual(self.m.output(now=2.05), Velocity())

    def test_cold_start_stays_stopped(self):
        self.assertEqual(self.m.output(now=0), Velocity())

    def test_good_sequenced_stream_allows_motion(self):
        self.running()

    def test_replayed_health_sequence_cannot_renew_health(self):
        self.running()
        self.assert_forced_stop(lambda: self.health(1.97, sequence=self.hs - 1))

    def test_replayed_odom_sequence_revokes_live_lease(self):
        self.running()
        self.assert_forced_stop(lambda: self.odom(1.97, sequence=self.odseq - 1))

    def test_reordered_health_measurement_time_revokes_lease(self):
        self.running()
        self.assert_forced_stop(lambda: self.health(1.97, sample=1.80))

    def test_stale_health_measurement_cannot_be_retimestamped(self):
        self.running()
        self.assert_forced_stop(lambda: self.health(1.97, sample=1.50))

    def test_future_odom_measurement_revokes_lease(self):
        self.running()
        self.assert_forced_stop(lambda: self.odom(1.97, sample=2.0))

    def test_foreign_source_odom_revokes_lease(self):
        self.running()
        self.assert_forced_stop(lambda: self.odom(1.97, source="odom:unknown"))

    def test_foreign_supervisor_health_revokes_lease(self):
        self.running()
        self.assert_forced_stop(
            lambda: self.health(1.97, source="supervisor:unknown"))

    def test_bool_sequence_is_not_a_valid_uint(self):
        self.running()
        self.assert_forced_stop(lambda: self.odom(1.97, sequence=True))

    def test_nonfinite_measurement_timestamp_revokes_lease(self):
        self.running()
        self.assert_forced_stop(lambda: self.health(1.97, sample=float("nan")))

    def test_stop_does_not_reset_replay_guard(self):
        self.running()
        s = self.hs
        self.m.stop("commanded stop")
        with self.assertRaises(ReactiveMuxError):
            self.health(1.97, sequence=s)

    def test_new_health_can_confirm_foreign_nav_goal_and_stop(self):
        self.running()
        self.assert_forced_stop(lambda: self.health(
            1.97, proof=good_health(
                sweep_inactive=False, nav2_idle=False,
                nav2_active_goal="f" * 32)))

    def test_stale_event_with_forward_sequence_is_rejected(self):
        self.running()
        self.assert_forced_stop(lambda: self.odom(1.97, sample=1.01, sequence=999))

    def test_mismatched_backend_reconciliation_cannot_arm(self):
        for i in range(8):
            self.odom(round(1. + i * .12, 5))
        self.health(1.85)
        with self.assertRaises(ReactiveMuxError):
            self.m.claim_sweep(now=1.9, nav_released=False,
                               prior_terminal_uuid=None)
        self.assertEqual(self.m.owner, "stopped")

    def test_parameters_reject_evidence_window_wider_than_watchdog(self):
        for kwargs in ({"max_evidence_age_s": .31},
                       {"max_evidence_age_s": 0},
                       {"odom_source": ""}):
            args = {"odom_source": "o", "health_source": "h"}
            args.update(kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                SequencedMotionMux(**args)

    def test_rejected_transit_claim_preserves_sequenced_original_owner(self):
        for index in range(8):
            self.odom(round(1. + index * .12, 5))
        self.health(1.85, proof=good_health())
        token = self.m.claim_transit(now=1.90, nav_goal_uuid="a" * 32)
        with self.assertRaises(ReactiveMuxError):
            self.m.claim_transit(now=1.91, nav_goal_uuid="b" * 32)
        self.odom(1.92)
        self.health(1.93, proof=good_health(
            nav2_idle=False, nav2_active_goal="a" * 32))
        self.m.accept_twist(source="navigation", token=token, now=1.94,
                            velocity=Velocity(.10, 0.))
        self.assertEqual(self.m.owner, "navigation")
        self.assertEqual(self.m.token, token)
        self.assertEqual(self.m.output(now=1.95), Velocity(.10, 0.))

    def test_fresh_forward_sequences_cannot_revive_expired_motion_lease(self):
        token = self.running()
        self.odom(2.30)
        self.health(2.31)
        with self.assertRaises(ReactiveMuxError):
            self.m.accept_twist(source="reactive_sweep", token=token, now=2.32,
                                velocity=Velocity(.10, .02))
        self.assertEqual(self.m.output(now=2.33), Velocity())
        self.assertEqual(self.m.owner, "stopped")
        self.assertIsNone(self.m.token)


if __name__ == "__main__":
    unittest.main()
