"""Synthetic anti-replay safety regression; no actual ROS2 or robot output."""
import ast
import copy
from dataclasses import replace
import math
from pathlib import Path
from types import SimpleNamespace
from tempfile import TemporaryDirectory
import unittest

from oomwoo_cleaning_jobs.dock_handoff import Velocity
from oomwoo_cleaning_jobs.goal_journal import GoalJournal, GoalJournalError
from oomwoo_cleaning_jobs.nav_lease_witness import ACTIVE, NavLeaseWitness, nav_status_idle
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
        names={"_on_motion_cmd","_reset","_motion_graph","_motion_now",
               "_poll_nav_backend","_execute_motion","_fresh","_feedback_live","_tick_motion",
               "_motion_health"}
        methods=[copy.deepcopy(n) for n in klass.body if isinstance(n,ast.FunctionDef)
                 and n.name in names]
        self.assertEqual(len(methods),len(names))
        module=ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(
            name="Relay",bases=[],keywords=[],body=methods,decorator_list=[])],type_ignores=[]))
        mono=[1.94];wall=[10_000_000_000]
        scope={"Velocity":Velocity,"ReactiveMuxError":ReactiveMuxError,
               "time":SimpleNamespace(monotonic=lambda:mono[0],time_ns=lambda:wall[0]),
               "GetState":SimpleNamespace(Request=object),"GoalJournalError":GoalJournalError,
               "MotionHealth":MotionHealth,"math":math,
               "_ACTIVE":ACTIVE,
               "rclpy":SimpleNamespace(time=SimpleNamespace(Time=lambda:None)),
               "TransformException":LookupError,"AdmissionError":ValueError,
               "nav_status_idle":nav_status_idle,
               "Twist":lambda:SimpleNamespace(linear=SimpleNamespace(x=0.),
                                               angular=SimpleNamespace(z=0.))}
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

    def managed(self):
        relay,mono,wall,graph=self.relay()
        relay._managed_nav2=True
        nav=graph["/valdi/nav2_cmd_vel"][0]
        nav.node_name="controller_server";nav.node_namespace="/"
        nav.topic_type="geometry_msgs/msg/Twist"
        relay.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=1_940_000_000))
        relay._nav_backend_query=None;relay._nav_backend_poll=mono[0]
        relay._nav_backend_active=True;relay._nav_backend_last=mono[0]
        relay._nav_lifecycle=SimpleNamespace(service_is_ready=lambda:False)
        relay._nav_inventory_last=0.
        nodes=[("coverage_planner","/"),("bt_navigator","/")]
        relay.get_node_names_and_namespaces=lambda:nodes
        relay.get_client_names_and_types_by_node=lambda *args:[
            ("/navigate_to_pose/_action/send_goal",["nav2_msgs/action/NavigateToPose_SendGoal"])]
        graph["/coverage_planner/motion_state"]=[SimpleNamespace(
            node_name="coverage_planner",node_namespace="/")]
        receivers={"/goal_pose":[],"/valdi/disabled_nav2_goal_pose":[SimpleNamespace(
            node_name="bt_navigator",node_namespace="/",topic_type="geometry_msgs/msg/PoseStamped")]}
        relay.get_subscriptions_info_by_topic=lambda topic:receivers[topic]
        return relay,mono,wall,graph,nodes,receivers

    def test_nav_backend_self_client_requires_quarantined_alternative_ingress(self):
        for case in ("isolated","public_receiver","quarantine_publisher","extra_client"):
            with self.subTest(case=case):
                relay,_,_,_,nodes,receivers=self.managed()
                if case=="public_receiver":receivers["/goal_pose"]=[object()]
                if case=="quarantine_publisher":
                    relay.count_publishers=lambda topic:int(topic=="/valdi/disabled_nav2_goal_pose")
                if case=="extra_client":nodes.append(("rviz2","/"))
                relay._poll_nav_backend()
                self.assertEqual(relay._nav_single_dispatcher,case=="isolated")

    def test_managed_claim_uses_same_simulated_clock_as_odometry(self):
        relay,mono,_,_,_,_=self.managed()
        relay.mux=ThreeWayVelocityMux()
        relay._prior_terminal=None
        for i in range(8):relay.mux.observe_odom(now=1.+i*.12,linear_mps=0.,angular_rps=0.)
        relay._motion_health=lambda:good_health()
        relay._goal_witness=SimpleNamespace(journal=SimpleNamespace(read=lambda:None))
        relay._map_hash="m"
        mono[0]=50_000.
        result=relay._execute_motion({"operation":"claim_sweep"})
        self.assertTrue(result["ok"])
        self.assertEqual(relay.mux.owner,relay.mux.SWEEP)
        self.assertAlmostEqual(relay.mux._health_time,1.94)

    def test_managed_active_feedback_requires_current_uuid_pose_and_journal(self):
        relay,mono,_,_,_,_=self.managed()
        relay._motion_nav_uuid="a"*32
        relay._goal_witness=SimpleNamespace(current_active_matches=lambda:True)
        relay._nav_feedback=(relay._motion_nav_uuid,mono[0],1_800_000_000)
        self.assertTrue(relay._feedback_live(relay._motion_nav_uuid))
        self.assertFalse(relay._feedback_live("b"*32))
        relay._nav_feedback=(relay._motion_nav_uuid,mono[0],1_000_000_000)
        self.assertFalse(relay._feedback_live(relay._motion_nav_uuid))
        relay._nav_feedback=(relay._motion_nav_uuid,mono[0],1_800_000_000)
        relay._goal_witness.current_active_matches=lambda:False
        self.assertFalse(relay._feedback_live(relay._motion_nav_uuid))
        relay._goal_witness.current_active_matches=lambda:True
        mono[0]+=1.01
        self.assertFalse(relay._feedback_live(relay._motion_nav_uuid))

    def test_lost_startup_lifecycle_query_cannot_suppress_fresh_queries(self):
        relay,mono,_,_,_,_=self.managed()
        canceled=[];requests=[]
        old=SimpleNamespace(done=lambda:False,cancel=lambda:canceled.append(True))
        replacement=SimpleNamespace(done=lambda:False)
        relay._nav_backend_query=old
        relay._nav_backend_poll=0.
        relay._nav_lifecycle=SimpleNamespace(service_is_ready=lambda:True,
            call_async=lambda request:requests.append(request) or replacement)
        relay._poll_nav_backend()
        self.assertEqual(canceled,[True])
        self.assertEqual(len(requests),1)
        self.assertIs(relay._nav_backend_query,replacement)
        self.assertFalse(relay._nav_backend_active)

    def test_managed_nav_accepts_supported_publication_sequence_then_rejects_second_writer(self):
        relay,_,_,graph,_,_=self.managed()
        zero=SimpleNamespace(linear=SimpleNamespace(x=0.),angular=SimpleNamespace(z=0.))
        relay._on_motion_cmd(relay.mux.NAV,zero,{
            "publication_sequence_number":1,"reception_sequence_number":0,
            "source_timestamp":9_990_000_000})
        self.assertEqual(relay.mux.owner,relay.mux.SWEEP)
        graph["/valdi/nav2_cmd_vel"].append(SimpleNamespace(endpoint_gid=bytes([10])*24))
        relay._on_motion_cmd(relay.mux.NAV,zero,{
            "publication_sequence_number":2,"reception_sequence_number":0,
            "source_timestamp":9_995_000_000})
        self.assertEqual(relay.mux.owner,relay.mux.STOP)
        self.assertIn("graph changed",relay.mux.stop_reason)

    def terminal_tick(self):
        relay,mono,_,_,_,_=self.managed()
        relay.mux=ThreeWayVelocityMux()
        for i in range(8):relay.mux.observe_odom(now=1.+i*.12,linear_mps=0.,angular_rps=0.)
        relay.mux.observe_health(now=1.85,proof=good_health())
        relay._motion_token=relay.mux.claim_transit(now=1.9,nav_goal_uuid="a"*32)
        relay.mux.observe_health(now=1.92,proof=good_health(
            nav2_idle=False,nav2_active_goal="a"*32))
        relay._poll_nav_backend=lambda:None
        relay._motion_health=lambda:good_health()
        relay._nav_lease_started=0.
        relay._last_status=mono[0]
        relay._nav_status=(("a"*32,4),)
        relay._nav_confirmed_terminal=lambda:False
        relay.safe=SimpleNamespace(publish=lambda message:None)
        return relay,mono

    def test_terminal_callback_before_result_fsync_keeps_zero_and_does_not_rearm_watchdog(self):
        relay,mono=self.terminal_tick()
        with TemporaryDirectory() as scratch:
            journal=GoalJournal(Path(scratch)/"goal.json")
            token="b"*32+"-1"
            journal.begin(uuid_hex="a"*32,token=token,job_id="coverage_sweep",
                          map_digest="c"*64,target_cell=0)
            journal.accepted(token=token,uuid_hex="a"*32)
            witness=NavLeaseWitness(journal)
            witness.observe((("a"*32,2),))
            relay._goal_witness=witness
            mono[0]=2.1
            relay._tick_motion()
            self.assertEqual(relay.mux.owner,relay.mux.NAV)
            self.assertEqual(relay._latest_output,Velocity())
            self.assertFalse(witness.can_release_for_dock())
            journal.terminal(token=token,uuid_hex="a"*32,outcome="succeeded")
            mono[0]=2.2
            relay._tick_motion()
            self.assertTrue(witness.can_release_for_dock())
            self.assertEqual(relay._last_status,1.94)

    def test_old_terminal_event_is_never_reconciled_as_fresh(self):
        relay,mono=self.terminal_tick()
        relay._last_status=0.
        relay._goal_witness=SimpleNamespace(saw_active=True,saw_terminal=False,fault=None,
            observe=lambda statuses:self.fail("stale terminal was replayed"),
            current_active_matches=lambda:False)
        mono[0]=2.1
        relay._tick_motion()
        self.assertEqual(relay._last_status,0.)

    def test_managed_contact_cancellation_mutes_nav_without_handing_out_a_sweep_lease(self):
        relay,_=self.terminal_tick()
        relay._motion_health=lambda:good_health(nav2_idle=False,nav2_active_goal="a"*32)
        relay._goal_witness=SimpleNamespace(saw_active=True,saw_terminal=False,
            current_active_matches=lambda:True)
        relay._nav_status=(("a"*32,3),)
        relay._sweep_state="TRANSIT_CANCEL"
        relay.mux.accept_twist(source=relay.mux.NAV,token=relay._motion_token,
                               now=1.93,velocity=Velocity(.1,0.))
        self.assertEqual(relay.mux.output(now=1.94),Velocity(.1,0.))
        relay._tick_motion()
        self.assertEqual(relay._latest_output,Velocity())
        self.assertEqual(relay.mux.owner,relay.mux.NAV)

    def test_transit_cancel_phase_is_inactive_only_in_the_managed_protocol(self):
        for managed in (False,True):
            with self.subTest(managed=managed):
                relay,mono,_,_,_,_=self.managed()
                relay._managed_nav2=managed
                relay.tf_buffer=SimpleNamespace(lookup_transform=lambda *args:SimpleNamespace(
                    header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=900_000_000)),
                    transform=SimpleNamespace(translation=SimpleNamespace(x=0.,y=0.))))
                relay._map_hash="known";relay._map_fault=False
                relay._map_raster=SimpleNamespace(cell_at=lambda x,y:0,data=[0])
                relay._nav_idle=lambda:False
                relay._nav_status=(("a"*32,2),)
                relay._last_status=mono[0]
                relay._battery_fraction=.92;relay._battery_last=mono[0]
                relay._clock_fault=False;relay._clock_advanced=mono[0]
                relay._unstable=False;relay._unstable_last=mono[0]
                relay._sweep_state="TRANSIT_CANCEL";relay._sweep_last=mono[0]
                relay._dock_state="IDLE";relay._dock_last=mono[0]
                relay._nav_single_dispatcher=True
                proof=relay._motion_health()
                self.assertTrue(proof.pose_fresh and proof.resources_ok and proof.isolated_sources)
                self.assertEqual(proof.safety_loop_ok,managed)
                self.assertEqual(proof.sweep_inactive,managed)
                self.assertEqual(proof.nav2_active_goal,"a"*32)


if __name__ == "__main__":
    unittest.main()
