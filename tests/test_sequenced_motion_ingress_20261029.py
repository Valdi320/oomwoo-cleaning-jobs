"""Synthetic anti-replay safety regression; no actual ROS2 or robot output."""
import ast
import copy
import json
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
from oomwoo_cleaning_jobs.map_admission import Raster, AdmissionError
from oomwoo_cleaning_jobs.nav_keepout_admission import VerifiedGlobalKeepout
from oomwoo_cleaning_jobs.reactive_keepout_envelope import (
    PoseEvidence, projected_motion_blocker)
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
               "_managed_idle",
               "_motion_health", "_keepout_dds_stamp", "_on_motion_keepout_mask",
               "_on_motion_keepout_info", "_on_motion_costmap", "_keepout_ready",
               "_keepout_output", "_motion_transform", "_motion_zero", "_on_nav_status"}
        methods=[copy.deepcopy(n) for n in klass.body if isinstance(n,ast.FunctionDef)
                 and n.name in names]
        self.assertEqual(len(methods),len(names))
        module=ast.fix_missing_locations(ast.Module(body=[ast.ClassDef(
            name="Relay",bases=[],keywords=[],body=methods,decorator_list=[])],type_ignores=[]))
        mono=[1.94];wall=[10_000_000_000]
        scope={"Velocity":Velocity,"ReactiveMuxError":ReactiveMuxError,
               "deepcopy":copy.deepcopy,
               "time":SimpleNamespace(monotonic=lambda:mono[0],time_ns=lambda:wall[0]),
               "GetState":SimpleNamespace(Request=object),"GoalJournalError":GoalJournalError,
               "MotionHealth":MotionHealth,"math":math,"json":json,
               "_ACTIVE":ACTIVE,
               "rclpy":SimpleNamespace(time=SimpleNamespace(Time=lambda:None)),
               "TransformException":LookupError,"AdmissionError":AdmissionError,
               "Raster":Raster,"VerifiedGlobalKeepout":VerifiedGlobalKeepout,
               "PoseEvidence":PoseEvidence,"projected_motion_blocker":projected_motion_blocker,
               "nav_status_idle":nav_status_idle,
               "Twist":lambda:SimpleNamespace(linear=SimpleNamespace(x=0.),
                                               angular=SimpleNamespace(z=0.))}
        exec(compile(module,str(source),"exec"),scope)
        relay=scope["Relay"]();relay._three_way=True;relay.mux=ThreeWayVelocityMux()
        relay._diagnostic_messages=[]
        relay.get_logger=lambda:SimpleNamespace(warn=relay._diagnostic_messages.append)
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

    def test_first_lease_revocation_survives_later_stopped_commands(self):
        relay,_,_,_=self.relay()
        relay._reset("original failure",proof=good_health(map_current=False))
        relay._reset("later stopped command")
        self.assertEqual(relay._last_lease_revocation["reason"],"original failure")
        self.assertFalse(relay._last_lease_revocation["health"]["map_current"])
        self.assertEqual(len(relay._diagnostic_messages),1)
        self.assertEqual(relay.mux.owner,relay.mux.STOP)

    def test_preclaim_packet_does_not_acquire_new_token(self):
        relay,_,_,_=self.relay();self.send(relay,stamp=9_899_999_999)
        self.assertEqual(relay.mux.owner,relay.mux.STOP)
        self.assertIn("predates",relay.mux.stop_reason)

    def test_queued_preclaim_zero_neither_revokes_nor_proves_the_new_lease(self):
        relay,mono,_,_=self.relay()
        token=relay._motion_token
        zero=SimpleNamespace(linear=SimpleNamespace(x=0.),angular=SimpleNamespace(z=0.))
        relay._on_motion_cmd(relay.mux.SWEEP,zero,{
            "source_timestamp":9_899_999_999,"publication_sequence_number":10})
        self.assertEqual(relay.mux.owner,relay.mux.SWEEP)
        self.assertEqual(relay._motion_token,token)
        self.assertEqual(relay.mux.output(now=1.95),Velocity())
        self.assertFalse(relay._motion_zero(relay.mux.SWEEP))
        mono[0]=1.955
        self.send(relay,sequence=11)
        self.assertEqual(relay.mux.output(now=1.96),Velocity(.18,0.))

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

    def initial_escape(self, directory):
        relay,mono,_,_,_,_=self.managed()
        relay._goal_witness=NavLeaseWitness(GoalJournal(Path(directory)/"goal.json"))
        relay._nav_single_dispatcher=True
        relay._sweep_state="ESCAPE";relay._sweep_last=mono[0]
        relay._nav_status=relay._last_status=relay._nav_feedback=None
        relay._nav_idle=relay._managed_idle
        relay.tf_buffer=SimpleNamespace(lookup_transform=lambda *args:SimpleNamespace(
            header=SimpleNamespace(stamp=SimpleNamespace(sec=1,nanosec=900_000_000)),
            transform=SimpleNamespace(translation=SimpleNamespace(x=0.,y=0.))))
        relay._map_hash="known";relay._map_fault=False
        relay._map_raster=SimpleNamespace(cell_at=lambda x,y:0,data=[0])
        relay._battery_fraction=.92;relay._battery_last=mono[0]
        relay._clock_fault=False;relay._clock_advanced=mono[0]
        relay._unstable=False;relay._unstable_last=mono[0]
        relay._dock_state="IDLE";relay._dock_last=mono[0]
        return relay,mono

    def test_initial_bumper_escape_preserves_sweep_lease_without_any_nav_goal(self):
        with TemporaryDirectory() as directory:
            relay,mono=self.initial_escape(directory)
            proof=relay._motion_health()
            self.assertTrue(proof.nav2_idle)
            self.assertTrue(proof.safety_loop_ok)
            self.assertFalse(proof.sweep_inactive)
            relay.mux.observe_health(now=mono[0],proof=proof)
            command=Velocity(-.08,-.6)
            relay.mux.accept_twist(source=relay.mux.SWEEP,token=relay._motion_token,
                                   now=mono[0],velocity=command)
            self.assertEqual(relay.mux.output(now=mono[0]),command)
            self.assertEqual(relay.mux.owner,relay.mux.SWEEP)

    def test_initial_escape_still_stops_for_contradictory_or_missing_nav_proofs(self):
        cases=("backend_inactive","backend_stale","dispatcher","phase_stale",
               "active_uuid","pending_journal","TRANSIT","TRANSIT_CANCEL","FAULT")
        for case in cases:
            with self.subTest(case=case), TemporaryDirectory() as directory:
                relay,mono=self.initial_escape(directory)
                if case=="backend_inactive":relay._nav_backend_active=False
                elif case=="backend_stale":relay._nav_backend_last=mono[0]-2.6
                elif case=="dispatcher":relay._nav_single_dispatcher=False
                elif case=="phase_stale":relay._sweep_last=mono[0]-1.01
                elif case=="active_uuid":relay._nav_status=(("a"*32,2),)
                elif case=="pending_journal":
                    relay._goal_witness.journal.begin(uuid_hex="a"*32,token="c"*32+"-1",
                        job_id="sweep",map_digest="b"*64,target_cell=0)
                else:relay._sweep_state=case
                proof=relay._motion_health()
                self.assertFalse(proof.nav2_idle)
                with self.assertRaises(ReactiveMuxError):
                    relay.mux.observe_health(now=mono[0],proof=proof)
                self.assertEqual(relay.mux.owner,relay.mux.STOP)
                self.assertEqual(relay.mux.output(now=mono[0]),Velocity())

    def test_other_initial_non_nav_phases_keep_their_existing_idle_proof(self):
        for phase in ("IDLE","SWEEP","PREPARE_NAV"):
            with self.subTest(phase=phase), TemporaryDirectory() as directory:
                relay,_=self.initial_escape(directory)
                relay._sweep_state=phase
                self.assertTrue(relay._managed_idle())

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

    def test_fast_active_then_cancel_between_ticks_retains_exact_release_proof(self):
        for common_valid in (True, False):
            with self.subTest(common_valid=common_valid), TemporaryDirectory() as scratch:
                relay,_,_,_,_,_=self.managed()
                relay.mux=ThreeWayVelocityMux()
                for i in range(8):
                    relay.mux.observe_odom(now=1.+i*.12,linear_mps=0.,angular_rps=0.)
                relay.mux.observe_health(now=1.85,proof=good_health())
                relay._motion_token=relay.mux.claim_transit(now=1.9,nav_goal_uuid="a"*32)
                relay._prior_terminal=None
                journal=GoalJournal(Path(scratch)/"goal.json")
                token="b"*32+"-1"
                journal.begin(uuid_hex="a"*32,token=token,job_id="coverage_sweep",
                              map_digest="c"*64,target_cell=0)
                journal.accepted(token=token,uuid_hex="a"*32)
                relay._goal_witness=NavLeaseWitness(journal)
                relay._nav_status=()
                relay._motion_health=lambda:good_health(
                    pose_fresh=common_valid,
                    nav2_idle=nav_status_idle(relay._nav_status),
                    nav2_active_goal=next((uuid for uuid,code in relay._nav_status
                                          if code in ACTIVE),None))
                def event(code):
                    return SimpleNamespace(status_list=[SimpleNamespace(
                        goal_info=SimpleNamespace(goal_id=SimpleNamespace(
                            uuid=bytes.fromhex("a"*32))),status=code)])
                # Both DDS events arrive within one50ms timer period. There
                # is no intermediate output tick or fabricated ACTIVE sample.
                relay._on_nav_status(event(2))
                journal.terminal(token=token,uuid_hex="a"*32,outcome="canceled")
                relay._on_nav_status(event(5))
                relay._commands[relay.mux.NAV]=(1.94,Velocity())
                relay._latest_output=Velocity()
                relay._nav_confirmed_terminal=relay._goal_witness.can_release_for_dock
                if common_valid:
                    self.assertTrue(relay._goal_witness.can_release_for_dock())
                    result=relay._execute_motion({"operation":"release_nav"})
                    self.assertTrue(result["ok"])
                    self.assertEqual(relay._prior_terminal,"a"*32)
                else:
                    with self.assertRaises(ReactiveMuxError):
                        relay._execute_motion({"operation":"release_nav"})
                    self.assertIsNone(relay._prior_terminal)
                self.assertEqual(relay.mux.owner,relay.mux.STOP)
                self.assertEqual(relay.mux.output(now=1.95),Velocity())

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

    def keepout(self, *, near=True):
        relay,mono,wall,graph=self.relay()
        relay._keepout_required=True
        relay._keepout_fault=False;relay._keepout_info_valid=False
        relay._keepout_mask=None;relay._keepout_costmap=None
        relay._keepout_mask_epoch=relay._keepout_info_epoch=0
        relay._keepout_costmap_dds=relay._keepout_costmap_sequence=0
        relay._keepout_costmap_stamp=0;relay._keepout_sources=None
        relay._keepout_proof=None
        relay._map_raster=Raster(60,60,.1,-3.,-3.,0.,(0,)*3600)
        relay.get_clock=lambda:SimpleNamespace(now=lambda:SimpleNamespace(nanoseconds=wall[0]))
        pose_stamp=[wall[0]-10_000_000]
        def transform(*args):
            return SimpleNamespace(
                header=SimpleNamespace(stamp=SimpleNamespace(
                    sec=pose_stamp[0]//1_000_000_000,nanosec=pose_stamp[0]%1_000_000_000)),
                transform=SimpleNamespace(translation=SimpleNamespace(x=0.,y=0.),
                    rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.)))
        relay.tf_buffer=SimpleNamespace(lookup_transform=transform)
        for i,(topic,kind) in enumerate([
                ("/oomwoo/keepout_mask","nav_msgs/msg/OccupancyGrid"),
                ("/oomwoo/keepout_filter_info","nav2_msgs/msg/CostmapFilterInfo"),
                ("/global_costmap/costmap","nav_msgs/msg/OccupancyGrid")],20):
            graph[topic]=[SimpleNamespace(endpoint_gid=bytes([i])*24,topic_type=kind)]
        def grid(cells,stamp=None):
            stamp=wall[0]-10_000_000 if stamp is None else stamp
            return SimpleNamespace(header=SimpleNamespace(frame_id="map",
                stamp=SimpleNamespace(sec=stamp//1_000_000_000,nanosec=stamp%1_000_000_000)),
                info=SimpleNamespace(width=60,height=60,resolution=.1,
                    origin=SimpleNamespace(position=SimpleNamespace(x=-3.,y=-3.),
                        orientation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.))),data=cells)
        cells=[0]*3600;cells[30*60+(33 if near else 45)]=100
        mask=grid(cells)
        relay._on_motion_keepout_mask(mask,{"source_timestamp":wall[0]-100_000_000})
        relay._on_motion_keepout_info(SimpleNamespace(type=0,base=0.,multiplier=1.,
            filter_mask_topic="/oomwoo/keepout_mask"),{"source_timestamp":wall[0]-90_000_000})
        relay._on_motion_costmap(grid(cells),{"source_timestamp":wall[0]-10_000_000,
                                            "publication_sequence_number":1})
        return relay,mono,wall,graph,grid,cells,pose_stamp

    def pose_health(self, relay, mono):
        relay._map_hash="known";relay._map_fault=False
        relay._nav_idle=lambda:True
        relay._nav_status=();relay._last_status=mono[0]
        relay._battery_fraction=.92;relay._battery_last=mono[0]
        relay._clock_fault=False;relay._clock_advanced=mono[0]
        relay._unstable=False;relay._unstable_last=mono[0]
        relay._sweep_state="SWEEP";relay._sweep_last=mono[0]
        relay._dock_state="IDLE";relay._dock_last=mono[0]

    def test_future_tf_uses_prior_original_pose_until_clock_catches_up(self):
        relay,mono,wall,_,_,_,pose_stamp=self.keepout(near=False)
        self.pose_health(relay,mono)
        previous=pose_stamp[0]
        self.assertTrue(relay._motion_health().pose_fresh)
        self.assertEqual(relay._keepout_output(Velocity(.18,0.)),Velocity(.18,0.))
        pose_stamp[0]=wall[0]+5_000_000
        self.assertTrue(relay._motion_health().pose_fresh)
        self.assertEqual(relay._motion_health_detail["pose_stamp_ns"],previous)
        self.assertEqual(relay._keepout_output(Velocity(.18,0.)),Velocity(.18,0.))
        wall[0]=pose_stamp[0]
        self.assertTrue(relay._motion_health().pose_fresh)
        self.assertEqual(relay._motion_health_detail["pose_stamp_ns"],pose_stamp[0])

    def test_successive_future_tf_samples_are_admitted_only_after_their_time(self):
        relay,mono,wall,_,_,_,pose_stamp=self.keepout(near=False)
        self.pose_health(relay,mono)
        self.assertTrue(relay._motion_health().pose_fresh)
        previous_future=None
        for _ in range(8):
            wall[0]+=50_000_000
            pose_stamp[0]=wall[0]+5_000_000
            self.assertTrue(relay._motion_health().pose_fresh)
            used=relay._motion_health_detail["pose_stamp_ns"]
            self.assertLessEqual(used,wall[0])
            if previous_future is not None:self.assertEqual(used,previous_future)
            self.assertEqual(relay._keepout_output(Velocity(.18,0.)),Velocity(.18,0.))
            previous_future=pose_stamp[0]

    def test_future_tf_without_accepted_history_cannot_move(self):
        relay,mono,wall,_,_,_,pose_stamp=self.keepout(near=False)
        self.pose_health(relay,mono);pose_stamp[0]=wall[0]+5_000_000
        self.assertFalse(relay._motion_health().pose_fresh)
        self.assertEqual(relay._keepout_output(Velocity(.18,0.)),Velocity())
        self.assertEqual(relay.mux.owner,relay.mux.STOP)

    def test_future_tf_cannot_refresh_the_200ms_output_source_budget(self):
        relay,mono,wall,_,_,_,pose_stamp=self.keepout(near=False)
        self.pose_health(relay,mono);previous=pose_stamp[0]
        self.assertTrue(relay._motion_health().pose_fresh)
        wall[0]=previous+200_000_000;pose_stamp[0]=wall[0]+5_000_000
        self.assertEqual(relay._keepout_output(Velocity(.18,0.)),Velocity(.18,0.))
        wall[0]+=1;pose_stamp[0]=wall[0]+5_000_000
        self.assertEqual(relay._keepout_output(Velocity(.18,0.)),Velocity())
        self.assertEqual(relay.mux.owner,relay.mux.STOP)

    def test_future_tf_cannot_refresh_the_500ms_health_source_budget(self):
        relay,mono,wall,_,_,_,pose_stamp=self.keepout(near=False)
        self.pose_health(relay,mono);previous=pose_stamp[0]
        self.assertTrue(relay._motion_health().pose_fresh)
        wall[0]=previous+500_000_000;pose_stamp[0]=wall[0]+5_000_000
        self.assertTrue(relay._motion_health().pose_fresh)
        self.assertEqual(relay._motion_health_detail["pose_stamp_ns"],previous)
        wall[0]+=1;pose_stamp[0]=wall[0]+5_000_000
        self.assertFalse(relay._motion_health().pose_fresh)

    def test_mutating_a_future_tf_cannot_change_the_saved_original_pose(self):
        relay,mono,wall,_,_,_,_=self.keepout(near=False)
        self.pose_health(relay,mono)
        transform=relay.tf_buffer.lookup_transform()
        relay.tf_buffer.lookup_transform=lambda *args:transform
        previous=transform.header.stamp.sec*1_000_000_000+transform.header.stamp.nanosec
        self.assertTrue(relay._motion_health().pose_fresh)
        transform.header.stamp.sec=wall[0]//1_000_000_000
        transform.header.stamp.nanosec=5_000_000
        transform.transform.translation.x=1.4
        self.assertTrue(relay._motion_health().pose_fresh)
        self.assertEqual(relay._motion_health_detail["pose_stamp_ns"],previous)
        self.assertEqual(relay._keepout_output(Velocity(.18,0.)),Velocity(.18,0.))
        self.assertEqual(relay._motion_health_detail["pose_latest_age_ns"],-5_000_000)

    def test_reset_discards_a_previously_admitted_pose(self):
        relay,mono,wall,_,_,_,pose_stamp=self.keepout(near=False)
        self.pose_health(relay,mono)
        self.assertTrue(relay._motion_health().pose_fresh)
        pose_stamp[0]=wall[0]+5_000_000
        self.assertTrue(relay._motion_health().pose_fresh)
        relay._reset("explicit stop")
        wall[0]+=10_000_000
        pose_stamp[0]=wall[0]+5_000_000
        self.assertFalse(relay._motion_health().pose_fresh)
        self.assertEqual(relay._keepout_output(Velocity(.18,0.)),Velocity())

    def test_missing_tf_lookup_does_not_reuse_cached_history(self):
        relay,mono,_,_,_,_,_=self.keepout(near=False)
        self.pose_health(relay,mono)
        self.assertTrue(relay._motion_health().pose_fresh)
        def unavailable(*args):raise LookupError("unavailable")
        relay.tf_buffer.lookup_transform=unavailable
        self.assertFalse(relay._motion_health().pose_fresh)
        self.assertEqual(relay._keepout_output(Velocity(.18,0.)),Velocity())
        self.assertEqual(relay.mux.owner,relay.mux.STOP)

    def test_actual_final_tick_stops_virtual_crossing_with_other_proofs_healthy(self):
        for near,expected in [(False,.18),(True,0.)]:
            with self.subTest(near=near):
                relay,_,_,_,_,_,_=self.keepout(near=near)
                self.assertTrue(relay._keepout_ready())
                self.send(relay)
                relay._motion_health=lambda:good_health(sweep_inactive=False)
                sent=[];relay.safe=SimpleNamespace(publish=sent.append)
                relay._tick_motion()
                self.assertEqual(len(sent),1)
                self.assertEqual(sent[0].linear.x,expected)
                if near:
                    self.assertEqual(relay.mux.owner,relay.mux.STOP)
                    self.assertIn("intersects keepout",relay.mux.stop_reason)

    def test_missing_postfilter_enforcement_cannot_relay_sweep(self):
        relay,_,wall,_,grid,_,_=self.keepout()
        relay._on_motion_costmap(grid([0]*3600),{
            "source_timestamp":wall[0],"publication_sequence_number":2})
        self.assertFalse(relay._keepout_ready())
        self.assertEqual(relay._keepout_output(Velocity(.18,0.)),Velocity())
        self.assertEqual(relay.mux.owner,relay.mux.STOP)

    def test_fresh_dds_receipt_does_not_refresh_old_costmap_ros_stamp(self):
        relay,_,wall,_,grid,cells,_=self.keepout(near=False)
        relay._on_motion_costmap(grid(cells,wall[0]-1_100_000_000),{
            "source_timestamp":wall[0],"publication_sequence_number":2})
        self.assertFalse(relay._keepout_ready())
        self.assertEqual(relay._keepout_output(Velocity(.18,0.)),Velocity())

    def test_stale_pose_blocks_even_with_fresh_matching_filtered_map(self):
        relay,_,wall,_,_,_,pose_stamp=self.keepout(near=False)
        self.assertTrue(relay._keepout_ready());pose_stamp[0]=wall[0]-210_000_000
        self.assertEqual(relay._keepout_output(Velocity(.18,0.)),Velocity())
        self.assertIn("pose stamp",relay.mux.stop_reason)

    def test_static_mask_change_is_sticky_and_old_bytes_cannot_rearm(self):
        relay,_,wall,_,grid,cells,_=self.keepout()
        original=list(cells);cells=list(cells);cells[1]=100
        relay._on_motion_keepout_mask(grid(cells),{"source_timestamp":wall[0]})
        relay._on_motion_keepout_mask(grid(original),{"source_timestamp":wall[0]+1})
        self.assertTrue(relay._keepout_fault)
        self.assertEqual(relay._keepout_output(Velocity(.18,0.)),Velocity())

    def test_shifted_mask_is_not_proven_by_a_matching_pixel_index(self):
        relay,_,wall,_,grid,cells,_=self.keepout()
        relay._keepout_mask=None;relay._keepout_proof=None
        shifted=grid(cells);shifted.info.origin.position.x+=.1
        relay._on_motion_keepout_mask(shifted,{"source_timestamp":wall[0]-100_000_000})
        self.assertFalse(relay._keepout_ready())

    def test_mask_publisher_replacement_latches_reconciliation(self):
        relay,_,_,graph,_,_,_=self.keepout(near=False)
        self.assertTrue(relay._keepout_ready())
        endpoint=graph["/oomwoo/keepout_mask"][0];original=endpoint.endpoint_gid
        endpoint.endpoint_gid=b"X"*24
        self.assertFalse(relay._keepout_ready());endpoint.endpoint_gid=original
        self.assertFalse(relay._keepout_ready())

    def test_stale_dds_costmap_cannot_authorize_fresh_ros_header(self):
        relay,_,wall,_,_,_,_=self.keepout(near=False)
        relay._keepout_costmap_dds=wall[0]-1_100_000_000
        self.assertFalse(relay._keepout_ready())


if __name__ == "__main__":
    unittest.main()
