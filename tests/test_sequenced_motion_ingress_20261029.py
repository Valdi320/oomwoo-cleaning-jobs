"""Synthetic anti-replay safety regression; no actual ROS2 or robot output."""
import ast
import copy
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import unittest

from oomwoo_cleaning_jobs.dock_handoff import Velocity
from oomwoo_cleaning_jobs.reactive_velocity import (
    MotionHealth, ReactiveMuxError, ThreeWayVelocityMux)
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



class TestRelayDDSMetadataIngress(unittest.TestCase):
    """Actual relay methods/core, synthetic DDS metadata, graph and clocks."""

    def relay(self):
        source=Path(__file__).parents[1]/"src/oomwoo_cleaning_jobs/ros_dock_velocity_relay.py"
        tree=ast.parse(source.read_text())
        klass=next(n for n in tree.body if isinstance(n,ast.ClassDef)
                   and n.name=="ExperimentalDockVelocityRelay")
        names={"_on_motion_cmd","_reset","_motion_graph"}
        methods=[copy.deepcopy(n) for n in klass.body if isinstance(n,ast.FunctionDef)
                 and n.name in names]
        self.assertEqual(len(methods),len(names))
        module=ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(
            name="Relay",bases=[],keywords=[],body=methods,decorator_list=[])],type_ignores=[]))
        mono=[1.94];wall=[10_000_000_000]
        scope={"Velocity":Velocity,"ReactiveMuxError":ReactiveMuxError,
               "time":SimpleNamespace(monotonic=lambda:mono[0],time_ns=lambda:wall[0])}
        exec(compile(module,str(source),"exec"),scope)
        relay=scope["Relay"]();relay._three_way=True;relay.mux=ThreeWayVelocityMux()
        for i in range(8):relay.mux.observe_odom(now=1.+i*.12,linear_mps=0.,angular_rps=0.)
        relay.mux.observe_health(now=1.85,proof=good_health())
        relay._motion_token=relay.mux.claim_sweep(now=1.9,nav_released=True,prior_terminal_uuid=None)
        relay._claim_wall_ns=9_900_000_000
        relay._commands={};relay._command_sequences={}
        relay._source_gids={role:bytes([i])*24 for i,role in enumerate(
            (relay.mux.NAV,relay.mux.SWEEP,relay.mux.DOCK),1)}
        relay._observed_gids={}
        graph={topic:[SimpleNamespace(endpoint_gid=gid)] for topic,gid in zip(
            ("/valdi/nav2_cmd_vel","/valdi/sweep_cmd_vel","/valdi/dock_drive_cmd_vel"),
            relay._source_gids.values())}
        graph["/valdi/lab_safe_cmd_vel_output"]=[SimpleNamespace(node_name="relay",node_namespace="/")]
        relay.get_publishers_info_by_topic=lambda topic:graph[topic]
        relay.get_name=lambda:"relay";relay.get_namespace=lambda:"/"
        relay.count_publishers=lambda topic:0
        return relay,mono,wall,graph

    def send(self,relay,*,sequence=10,stamp=9_990_000_000,info=None):
        if info is None:info={"publication_sequence_number":sequence,"source_timestamp":stamp}
        msg=SimpleNamespace(linear=SimpleNamespace(x=.18),angular=SimpleNamespace(z=0.))
        relay._on_motion_cmd(relay.mux.SWEEP,msg,info)

    def test_real_method_accepts_supported_sequence_and_epoch(self):
        relay,_,_,_=self.relay();self.send(relay)
        self.assertEqual(relay.mux.output(now=1.95),Velocity(.18,0.))

    def test_preclaim_packet_does_not_acquire_new_token(self):
        relay,_,_,_=self.relay();self.send(relay,stamp=9_899_999_999)
        self.assertEqual(relay.mux.owner,relay.mux.STOP)
        self.assertIn("predates",relay.mux.stop_reason)

    def test_sequence_replay_and_reordered_timestamp_revoke(self):
        for seq,stamp in [(10,9_991_000_000),(11,9_980_000_000)]:
            relay,mono,_,_=self.relay();self.send(relay);mono[0]=1.96
            self.send(relay,sequence=seq,stamp=stamp)
            self.assertEqual(relay.mux.owner,relay.mux.STOP)
            self.assertIn("replayed/reordered",relay.mux.stop_reason)

    def test_missing_unsupported_or_stale_metadata_cannot_move(self):
        cases=[{}, {"source_timestamp":9_990_000_000},
               {"source_timestamp":9_990_000_000,"publication_sequence_number":-1},
               {"source_timestamp":9_990_000_000,"publication_sequence_number":True},
               {"source_timestamp":9_990_000_000,"publication_sequence_number":2**63},
               {"source_timestamp":9_600_000_000,"publication_sequence_number":10},
               {"source_timestamp":10_001_000_000,"publication_sequence_number":10}]
        for info in cases:
            with self.subTest(info=info):
                relay,_,_,_=self.relay();self.send(relay,info=info)
                self.assertEqual(relay.mux.owner,relay.mux.STOP)

    def test_replaced_endpoint_or_second_writer_cannot_inherit_lease(self):
        for duplicate in [False,True]:
            relay,_,_,graph=self.relay()
            changed=SimpleNamespace(endpoint_gid=bytes([8])*24)
            graph["/valdi/sweep_cmd_vel"]=(graph["/valdi/sweep_cmd_vel"]+[changed]
                if duplicate else [changed])
            self.send(relay)
            self.assertEqual(relay.mux.owner,relay.mux.STOP)
            self.assertIn("graph changed",relay.mux.stop_reason)

    def test_history_survives_stop_and_fresh_claim(self):
        for sequence,owner in [(10,"stopped"),(11,"reactive_sweep")]:
            with self.subTest(sequence=sequence):
                relay,mono,wall,_=self.relay();self.send(relay);relay._reset("operator stop")
                for i in range(8):relay.mux.observe_odom(now=2.+i*.12,linear_mps=0.,angular_rps=0.)
                relay.mux.observe_health(now=2.85,proof=good_health())
                relay._motion_token=relay.mux.claim_sweep(now=2.9,nav_released=True,prior_terminal_uuid=None)
                relay._claim_wall_ns=10_900_000_000;mono[0]=2.94;wall[0]=11_000_000_000
                self.send(relay,sequence=sequence,stamp=10_990_000_000)
                self.assertEqual(relay.mux.owner,owner)


if __name__ == "__main__":
    unittest.main()
