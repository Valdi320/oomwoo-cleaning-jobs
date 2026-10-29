"""Synthetic fail-closed reactive /cmd_vel keepout projection; no ROS2 or hardware."""
from dataclasses import replace
import unittest

from oomwoo_cleaning_jobs.dock_handoff import Velocity
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.reactive_velocity import MotionHealth
from oomwoo_cleaning_jobs.sequenced_motion_ingress import EvidenceEnvelope, SequencedMotionMux
from oomwoo_cleaning_jobs.reactive_keepout_envelope import (
    KeepoutEvidence, PoseEvidence, ReactiveKeepoutEnvelope)


def raster(*blocked, cost=100, frame="map", yaw=0.):
    cells = [0] * (60 * 60)
    origin = -3.
    for x, y in blocked:
        col = int((x - origin) / .1)
        row = int((y - origin) / .1)
        cells[row * 60 + col] = cost
    return Raster(60, 60, .1, origin, origin, yaw, tuple(cells), frame)


def health(**kwargs):
    return replace(MotionHealth(
        pose_fresh=True, resources_ok=True, map_current=True,
        safety_loop_ok=True, isolated_sources=True, nav2_idle=True,
        nav2_active_goal=None, dock_inactive=True, sweep_inactive=False), **kwargs)


class ReactiveKeepoutEnvelopeTest(unittest.TestCase):
    def setUp(self):
        self.m = SequencedMotionMux(odom_source="odom:sim", health_source="health:sim")
        self.guard = ReactiveKeepoutEnvelope(self.m)
        self.empty = raster()
        self.mask = raster()
        self.live = raster()

    def active(self, velocity=Velocity(.20, 0.)):
        for i in range(8):
            t = round(1. + .12 * i, 5)
            self.m.observe_odom(
                now=t, envelope=EvidenceEnvelope("odom:sim", i, t-.01),
                linear_mps=0., angular_rps=0.)
        self.m.observe_health(
            now=1.85, envelope=EvidenceEnvelope("health:sim", 0, 1.84),
            proof=health())
        token = self.m.claim_sweep(
            now=1.90, nav_released=True, prior_terminal_uuid=None)
        self.m.observe_odom(
            now=1.92, envelope=EvidenceEnvelope("odom:sim", 8, 1.91),
            linear_mps=0., angular_rps=0.)
        self.m.observe_health(
            now=1.93, envelope=EvidenceEnvelope("health:sim", 1, 1.92),
            proof=health())
        self.m.accept_twist(
            source="reactive_sweep", token=token, now=1.94, velocity=velocity)

    def evidence(self, *, static=None, live=None, mask=None,
                 static_at=1.90, live_at=1.90, mask_at=1.90):
        return KeepoutEvidence(
            self.empty if static is None else static,
            self.live if live is None else live,
            self.mask if mask is None else mask,
            static_at, live_at, mask_at)

    def decision(self, maps=None, pose=None, now=1.95):
        return self.guard.output(
            now=now,
            pose=PoseEvidence(0., 0., 0., 1.92) if pose is None else pose,
            maps=self.evidence() if maps is None else maps)

    def stopped(self, reason):
        self.assertEqual(self.m.owner, "stopped")
        self.assertEqual(self.m.output(now=3.0), Velocity())
        self.assertIn(reason, self.m.stop_reason)

    def test_01_cold_start_output_zero(self):
        self.assertEqual(self.decision(), Velocity())

    def test_02_empty_maps_allow_bounded_sweep(self):
        self.active()
        self.assertEqual(self.decision(), Velocity(.20, 0.))
        self.assertEqual(self.m.owner, "reactive_sweep")

    def test_03_real_static_wall_in_prediction_path_revokes_lease(self):
        self.active()
        self.assertEqual(self.decision(self.evidence(static=raster((.40, 0.)))), Velocity())
        self.stopped("static")

    def test_04_virtual_wall_alone_blocks_direct_reactive_motion(self):
        self.active()
        self.assertEqual(self.decision(self.evidence(mask=raster((.40, 0.)))), Velocity())
        self.stopped("keepout")

    def test_05_live_obstacle_with_free_static_and_mask_blocks(self):
        self.active()
        self.assertEqual(self.decision(self.evidence(live=raster((.40, 0.)))), Velocity())
        self.stopped("live")

    def test_06_distant_wall_does_not_disable_all_sweeps(self):
        self.active()
        self.assertEqual(self.decision(self.evidence(mask=raster((1.20, 0.)))), Velocity(.20, 0.))

    def test_07_reversed_command_checks_rear_area(self):
        self.active(Velocity(-.20, 0.))
        self.assertEqual(self.decision(self.evidence(mask=raster((-.42, 0.)))), Velocity())
        self.stopped("keepout")

    def test_08_angular_sweep_evaluates_arc_not_only_forward_ray(self):
        self.active(Velocity(.20, .75))
        self.assertEqual(self.decision(self.evidence(mask=raster((.27, .26)))), Velocity())
        self.stopped("keepout")

    def test_09_stale_pose_revokes_current_token(self):
        self.active()
        self.assertEqual(self.decision(pose=PoseEvidence(0., 0., 0., 1.0)), Velocity())
        self.stopped("pose")

    def test_10_stale_mask_revokes_current_token(self):
        self.active()
        self.assertEqual(self.decision(self.evidence(mask_at=1.0)), Velocity())
        self.stopped("map")

    def test_11_non_raster_live_costmap_revokes_current_token(self):
        self.active()
        self.assertEqual(self.decision(self.evidence(live=object())), Velocity())
        self.stopped("map")

    def test_12_invalid_mask_encoding_revokes_current_token(self):
        self.active()
        self.assertEqual(self.decision(self.evidence(mask=raster((.40, 0.), cost=50))), Velocity())
        self.stopped("map")

    def test_13_mismatched_mask_geometry_revokes_current_token(self):
        self.active()
        shifted = Raster(60, 60, .1, -2.9, -3., 0., self.mask.data)
        self.assertEqual(self.decision(self.evidence(mask=shifted)), Velocity())
        self.stopped("map")

    def test_14_fresh_map_cannot_replace_stale_underlying_odom(self):
        self.active()
        self.assertEqual(self.decision(now=2.4), Velocity())
        self.stopped("lease")

    def test_15_nonfinite_pose_revokes_lease(self):
        self.active()
        self.assertEqual(self.decision(pose=PoseEvidence(float("nan"), 0., 0., 1.92)), Velocity())
        self.stopped("pose")

    def test_16_turn_in_place_checks_full_circular_footprint(self):
        self.active(Velocity(0., .75))
        self.assertEqual(self.decision(self.evidence(mask=raster((.23, 0.)))), Velocity())
        self.stopped("keepout")

    def test_17_invalid_configuration_rejected(self):
        for kwargs in ({"prediction_horizon_s":0},
                       {"footprint_radius_m":0},
                       {"stop_margin_m":-1},
                       {"max_map_age_s":float("inf")},
                       {"max_pose_age_s":True},
                       {"live_max_cost":100}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                ReactiveKeepoutEnvelope(self.m, **kwargs)

    def test_18_foreign_nav_owner_never_relayed_by_sweep_guard(self):
        # Nav2 has its own relay; this sweep-only guard must not become it.
        uuid = "a" * 32
        for i in range(8):
            t = round(1. + .12 * i, 5)
            self.m.observe_odom(
                now=t, envelope=EvidenceEnvelope("odom:sim", i, t-.01),
                linear_mps=0., angular_rps=0.)
        self.m.observe_health(
            now=1.85, envelope=EvidenceEnvelope("health:sim", 0, 1.84),
            proof=health(sweep_inactive=True))
        token = self.m.claim_transit(now=1.90, nav_goal_uuid=uuid)
        self.m.observe_odom(
            now=1.92, envelope=EvidenceEnvelope("odom:sim", 8, 1.91),
            linear_mps=0., angular_rps=0.)
        self.m.observe_health(
            now=1.93, envelope=EvidenceEnvelope("health:sim", 1, 1.92),
            proof=health(sweep_inactive=True, nav2_idle=False, nav2_active_goal=uuid))
        self.m.accept_twist(source="navigation", token=token, now=1.94,
                            velocity=Velocity(.10, 0.))
        self.assertEqual(self.decision(), Velocity())
        self.stopped("sweep-only")

    def test_19_output_zero_does_not_rearm_or_move_after_stop(self):
        self.active()
        self.decision(self.evidence(mask=raster((.40, 0.))))
        self.assertEqual(self.decision(self.evidence()), Velocity())

    def test_20_valid_to_invalid_mask_swap_cannot_reuse_cache(self):
        self.active()
        self.assertEqual(self.decision(), Velocity(.20, 0.))
        invalid = raster((.9, .9), cost=50)
        self.assertEqual(self.decision(self.evidence(mask=invalid)), Velocity())
        self.stopped("map")

    def test_21_new_keepout_snapshot_after_clear_output_blocks(self):
        self.active()
        self.assertEqual(self.decision(), Velocity(.20, 0.))
        forbidden = raster((.40, 0.))
        self.assertEqual(self.decision(self.evidence(mask=forbidden)), Velocity())
        self.stopped("keepout")

    def test_22_live_map_crop_outside_prediction_is_stop(self):
        self.active()
        # A rolling local map that does not fully include the projected
        # circular footprint cannot be interpreted as clear.
        cropped = Raster(20, 20, .05, -.1, -.1, 0., (0,) * 400)
        self.assertEqual(self.decision(self.evidence(live=cropped)), Velocity())
        self.stopped("live")


if __name__ == "__main__":
    unittest.main()
