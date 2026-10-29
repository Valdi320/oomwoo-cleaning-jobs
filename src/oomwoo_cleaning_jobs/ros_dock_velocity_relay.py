"""Experimental exclusive Nav2/dock-drive velocity relay for ISOLATED ROS2 labs.

Never launch this unchanged against a real robot: it publishes ONLY the
default /valdi/lab_safe_cmd_vel_output topic, NOT /cmd_vel, and will not
arm unless simulation_lab=true. To move a physical robot, canonical OOMWOO
bringup, trusted ROS2 endpoints and an independent MCU watchdog must be
reviewed and tested. ROS2 status messages are not authenticated.

The default two-source mode retains ExclusiveVelocityMux. Opt-in three_way
uses the existing ThreeWayVelocityMux in this same node, with one lab output
and one steady-clock timer. It derives health from TF/map/clock/phase/resource
observations and checks unique graph endpoints plus native DDS sequence/time.
Sources are experimental simulation observations, not physical attestations.
"""
from __future__ import annotations
import json
import hashlib
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import (
    QoSProfile,QoSDurabilityPolicy,QoSReliabilityPolicy)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from rosgraph_msgs.msg import Clock as ClockMessage
from tf2_ros import Buffer, TransformListener, TransformException
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool,String
from action_msgs.msg import GoalStatusArray
from oomwoo_cleaning_jobs_lab_interfaces.srv import (
    ControlCleaning,GetCleaningStatus)

from .dock_handoff import ExclusiveVelocityMux,Velocity,DockHandoffError
from .reactive_velocity import ThreeWayVelocityMux, MotionHealth, ReactiveMuxError
from .map_admission import Raster, AdmissionError
from .lab_topic_guard import (
    LabTopicGuardError,require_lab_output_publisher,
    require_lab_input_subscriptions)
from .ros_odom_stamp_gate import OdomStampGate
from .ros_odom_frame_gate import OdomFrameError, require_odom_frames
from .request_dedupe import RequestDedupe,CommandReplayError
from .goal_journal import GoalJournal,GoalJournalError
from .nav_lease_witness import NavLeaseWitness,ACTIVE,nav_status_idle


_ACTIVE=frozenset((1,2,3))
_TERMINAL=frozenset((4,5,6))


class ExperimentalDockVelocityRelay(Node):
    def __init__(self,*,parameter_overrides=None):
        super().__init__("valdi_experimental_cmdvel_relay",
                         parameter_overrides=parameter_overrides)
        self.declare_parameter("simulation_lab",False)
        self.declare_parameter("output_topic","/valdi/lab_safe_cmd_vel_output")
        self.declare_parameter("goal_journal_path","")
        self.declare_parameter("three_way",False)
        self._three_way=self.get_parameter("three_way").value is True
        if self.get_parameter("simulation_lab").value is not True:
            raise ValueError("command source relay requires isolated simulation_lab")
        output=str(self.get_parameter("output_topic").value)
        if output in ("/cmd_vel","cmd_vel","/diff_drive_controller/cmd_vel_unstamped"):
            raise ValueError("lab relay must not publish directly to real motors")
        if output!="/valdi/lab_safe_cmd_vel_output":
            raise ValueError("unsupported lab output topic")
        self.mux=(ThreeWayVelocityMux() if self._three_way else
                  ExclusiveVelocityMux(max_age_s=.30,quiet_required_s=.75))
        self._odom_stamp_gate=OdomStampGate()
        self._requests=RequestDedupe(128)
        journal_path=str(self.get_parameter("goal_journal_path").value)
        self._goal_witness=(NavLeaseWitness(GoalJournal(journal_path))
                            if journal_path else None)
        self._nav_lease_started=None
        self._last_status=None
        self._nav_status=None
        self._nav_zero=True
        self._dock_state=None
        self._docked=False
        self._dock_last=None
        self._odom_last=None
        self._battery_last=None
        self._battery_fraction=None
        self._released_nav=False
        self._latest_output=Velocity()
        if self._three_way:
            self._motion_token=None
            self._claim_wall_ns=0
            self._source_gids={}
            self._observed_gids={}
            self._commands={}
            self._command_sequences={}
            self._sweep_state=None
            self._sweep_last=None
            self._docked_last=None
            self._map_hash=None
            self._map_fault=False
            self._clock_ns=None
            self._clock_advanced=None
            self._clock_fault=False
            self._unstable=None
            self._unstable_last=None
            self._prior_terminal=None
            self._dock_started=False
            self.tf_buffer=Buffer()
        latched=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.safe=self.create_publisher(Twist,output,10)
        # ROS2 CLI/launch remapping can redirect the *nominally safe* topic.
        # Reject the REAL resolved publisher name before any output timer.
        try:
            require_lab_output_publisher(self.safe)
        except LabTopicGuardError:
            self.destroy_publisher(self.safe)
            raise
        # Actual ROS2 input resolution must retain source isolation.
        # Verify all subscribed input topics before services/output timer.
        inputs={}
        try:
            inputs["/valdi/nav2_cmd_vel"]=self.create_subscription(
                Twist,"/valdi/nav2_cmd_vel",self._nav_twist,10)
            inputs["/valdi/dock_drive_cmd_vel"]=self.create_subscription(
                Twist,"/valdi/dock_drive_cmd_vel",self._dock_twist,10)
            inputs["/odom"]=self.create_subscription(
                Odometry,"/odom",self._on_odom,10)
            inputs["/battery_state"]=self.create_subscription(
                BatteryState,"/battery_state",self._battery,10)
            inputs["/navigate_to_pose/_action/status"]=self.create_subscription(
                GoalStatusArray,"/navigate_to_pose/_action/status",
                self._on_nav_status,latched)
            inputs["/dock_drive/state"]=self.create_subscription(
                String,"/dock_drive/state",self._on_dock_state,10)
            inputs["/dock_drive/docked"]=self.create_subscription(
                Bool,"/dock_drive/docked",self._on_docked,latched)
            if self._three_way:
                inputs["/valdi/sweep_cmd_vel"]=self.create_subscription(
                    Twist,"/valdi/sweep_cmd_vel",self._sweep_twist,10)
                inputs["/coverage_planner/motion_state"]=self.create_subscription(
                    String,"/coverage_planner/motion_state",self._on_sweep_state,10)
                inputs["/map"]=self.create_subscription(
                    OccupancyGrid,"/map",self._on_motion_map,latched)
                inputs["/clock"]=self.create_subscription(
                    ClockMessage,"/clock",self._on_motion_clock,10)
                inputs["/coverage_meter/sim_unstable"]=self.create_subscription(
                    Bool,"/coverage_meter/sim_unstable",self._on_motion_unstable,10)
                self.tf_listener=TransformListener(self.tf_buffer,self)
                inputs["/tf"]=self.tf_listener.tf_sub
                inputs["/tf_static"]=self.tf_listener.tf_static_sub
                require_lab_input_subscriptions(inputs,three_way=True)
            else:
                require_lab_input_subscriptions(inputs)
        except Exception:
            for sub in inputs.values():
                try:self.destroy_subscription(sub)
                except Exception:pass
            self.destroy_publisher(self.safe)
            raise
        self.create_service(ControlCleaning,
              "/valdi/relay/control",self._control)
        self.create_service(GetCleaningStatus,
              "/valdi/relay/status",self._status)
        if self._three_way:
            # Reuse this relay's only timer, on steady time so a paused /clock
            # cannot suspend its existing output/lease checks.
            self.create_timer(.05,self._tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        else:
            self.create_timer(.05,self._tick)

    def _on_nav_status(self,msg):
        self._last_status=time.monotonic()
        self._nav_status=tuple(
            (bytes(s.goal_info.goal_id.uuid).hex(),int(s.status))
            for s in msg.status_list)
        if getattr(self,"_three_way",False):
            if self._goal_witness is not None and (
                    self.mux.owner==self.mux.NAV or self._prior_terminal is not None):
                self._goal_witness.observe(self._nav_status)
                if self._goal_witness.fault is not None:
                    self._reset(self._goal_witness.fault)
            return
        if (self._goal_witness is not None and
            (self.mux.owner in (self.mux.NAV, self.mux.DOCK)
             or self._released_nav)):
            self._goal_witness.observe(self._nav_status)
            if self._goal_witness.fault is not None:
                self._reset(self._goal_witness.fault)
                self._released_nav=False

    def _on_dock_state(self,msg):
        self._dock_state=str(msg.data)
        self._dock_last=time.monotonic()
        if getattr(self,"_three_way",False):
            if self.mux.owner==self.mux.DOCK and self._dock_state in (
                    "SEARCH","GOTO","TURN","BACK","REGROUP"):
                self._dock_started=True
            return
        if (self.mux.owner==self.mux.DOCK and
            (self._dock_state!="DONE" or self._docked)):
            # DockDrive publishes state and docked on separate ROS topics.
            # An arriving DONE state before its latched docked=True must
            # NOT fault an otherwise legitimate confirmation pair.
            try:self.mux.observe_dock(now=time.monotonic(),
                 drive_state=self._dock_state,docked=self._docked)
            except DockHandoffError as exc:self._reset(str(exc))

    def _on_docked(self,msg):
        self._docked=bool(msg.data)
        if getattr(self,"_three_way",False):
            self._docked_last=time.monotonic()
            return
        if (self.mux.owner==self.mux.DOCK and self._dock_state is not None
            and (self._dock_state!="DONE" or self._docked
                 or self.mux.docking_proven)):
            try:self.mux.observe_dock(now=time.monotonic(),
                 drive_state=self._dock_state,docked=self._docked)
            except DockHandoffError as exc:self._reset(str(exc))

    def _on_odom(self,msg):
        # Receipt time alone is NOT a freshness proof: a delayed/replayed
        # Odometry packet can arrive now with a stale original header stamp.
        # Compare header to the node's own ROS clock (supports sim /clock),
        # never to monotonic wall time. Missing /clock, rewind or replay STOP.
        observed_at=time.monotonic()
        # Fresh timestamps in an unrelated reference frame are NOT
        # evidence of a stationary robot. Check OOMWOO's TF contract first.
        try:
            require_odom_frames(msg.header.frame_id,msg.child_frame_id)
        except (OdomFrameError,TypeError,AttributeError):
            self._odom_last=None
            self._reset("ROS odometry frame mismatch or missing")
            return
        try:
            stamp=msg.header.stamp
            self._odom_stamp_gate.accept(
                ros_now_ns=self.get_clock().now().nanoseconds,
                stamp_sec=stamp.sec,stamp_nanosec=stamp.nanosec)
        except Exception:
            self._odom_last=None
            self._reset("ROS odometry header stale/replayed/invalid")
            return
        try:
            v=msg.twist.twist
            speed=math.hypot(v.linear.x,v.linear.y)
            self.mux.observe_odom(now=observed_at,
                  linear_mps=speed,angular_rps=v.angular.z)
        except (DockHandoffError,ReactiveMuxError,ValueError,TypeError,OverflowError):
            self._odom_last=None
            self._reset("invalid observed odometry")
            return
        self._odom_last=observed_at

    def _battery(self,msg):
        self._battery_fraction=float(msg.percentage)
        self._battery_last=time.monotonic()

    def _fresh(self,then,age):
        return then is not None and 0<=time.monotonic()-then<age

    def _nav_idle(self):
        return (self._fresh(self._last_status,3.)
            and nav_status_idle(self._nav_status))

    def _nav_confirmed_terminal(self):
        # Revocable: never trust the terminal witness only from NAV phase.
        return (self._nav_idle()
            and self._goal_witness is not None
            and self._nav_status is not None
            and self._goal_witness.still_authorizes_dock(self._nav_status))

    def _on_cmd(self,source,msg):
        stamp=time.monotonic()
        v=Velocity(float(msg.linear.x),float(msg.angular.z))
        if source==self.mux.NAV:
            self._nav_zero=(abs(v.linear)<.001 and abs(v.angular)<.001)
        try:self.mux.accept_twist(source,v,now=stamp)
        except DockHandoffError:self._reset("invalid velocity data")

    def _nav_twist(self,msg,info):
        if self._three_way:
            self._on_motion_cmd(self.mux.NAV,msg,info)
        else:
            self._on_cmd(self.mux.NAV,msg)

    def _dock_twist(self,msg,info):
        if self._three_way:
            self._on_motion_cmd(self.mux.DOCK,msg,info)
        else:
            self._on_cmd(self.mux.DOCK,msg)
    def _sweep_twist(self,msg,info):self._on_motion_cmd(self.mux.SWEEP,msg,info)

    def _execute(self,body):
        if getattr(self,"_three_way",False):
            return self._execute_motion(body)
        name=body.get("operation")
        now=time.monotonic()
        if name=="stop":
            self._reset("ROS2 explicit STOP")
            self._released_nav=False
            return {"ok":True,"state":self.mux.owner}
        if name=="claim_nav":
            if self._goal_witness is None:
                raise DockHandoffError("exact Nav2 goal UUID journal required before lab handoff")
            self.mux.claim_nav(now=now,nav2_idle=self._nav_idle(),
                 pose_fresh=self._fresh(self._odom_last,.35),
                 dock_process_inactive=(self._dock_state=="IDLE"
                      and self._fresh(self._dock_last,3.)),
                 resources_verified=(self._fresh(self._battery_last,10.)
                      and self._battery_fraction is not None
                      and math.isfinite(self._battery_fraction)
                      and .20<=self._battery_fraction<=1.))
            self._goal_witness.reset()
            self._nav_lease_started=now
            self._released_nav=False
        elif name=="release_for_dock":
            self.mux.release_for_dock(now=now,
                  nav2_terminal=self._nav_confirmed_terminal(),
                  action_uuid_reconciled=self._nav_confirmed_terminal(),
                  nav2_input_stopped=self._nav_zero)
            self._released_nav=True
        elif name=="claim_dock":
            self.mux.claim_dock(now=now,
                    nav2_relinquished=(self._released_nav
                        and self._nav_confirmed_terminal()),
                    dock_drive_input_isolated=True)
            self._released_nav=False  # one-shot, never replay a stale release
        elif name=="finish_dock":
            # This lab-only node cannot independently prove an actual
            # physical dock controller has exited. Check last DONE/
            # DOCKED and allow only when there is no named dock_drive
            # node in this isolated graph. Never infer charge/empty/wash.
            nodes=self.get_node_names()
            self.mux.finish_dock(now=now,dock_drive_terminated=(
                self._dock_state=="DONE" and self._docked
                and self._fresh(self._dock_last,3.)
                and "dock_drive" not in nodes))
            self._released_nav=False
        else:
            raise DockHandoffError("unsupported exclusive relay operation")
        return {"ok":True,"state":self.mux.owner}

    def _control(self,request,response):
        if not request.isolated_disposable_sim:
            response.ok=False
            response.error="isolated disposable simulator must be explicitly selected"
            response.state=self.mux.owner
            return response
        packet={"operation":str(request.operation)}
        try:
            result=self._requests.handle(request.request_id,packet,self._execute)
            response.ok=bool(result["ok"])
            response.error=str(result.get("error",""))
        except (DockHandoffError,ReactiveMuxError,GoalJournalError,CommandReplayError,ValueError) as exc:
            response.ok=False
            response.error=str(exc)
        response.state=self.mux.owner
        return response

    def _status(self,request,response):
        response.ok=True
        response.status_json=json.dumps({
            "mode":"ISOLATED ROS2 SIMULATION ONLY",
            "owner":self.mux.owner,"reason":self.mux.stop_reason,
            "policy":"three_way" if self._three_way else "two_way",
            "motion_health":(self._motion_health().__dict__ if self._three_way else None),
            "nav_status_fresh":self._fresh(self._last_status,3.),
            "nav_idle":self._nav_idle(),
            "odom_fresh":self._fresh(self._odom_last,.35),
            "dock_state":self._dock_state,"docked":self._docked,
            "battery_fresh":self._fresh(self._battery_last,10.),
            "last_output":[self._latest_output.linear,self._latest_output.angular],
            "physical_dock_service_verified":False,
            "motor_hardware_connected":False,
            "nav_uuid_evidence":(
                {"uuid":self._goal_witness.uuid,
                 "saw_current_goal_active":self._goal_witness.saw_active,
                 "terminal_journal_matched":self._goal_witness.can_release_for_dock(),
                 "fault":self._goal_witness.fault}
                if self._goal_witness else None)},sort_keys=True)
        return response

    def _tick(self):
        if getattr(self,"_three_way",False):
            self._tick_motion()
            return
        # Active ownership is a continuing lease, NOT a one-off service
        # promise: loss of odometry, battery or the Nav2 goal-status topic
        # makes the lab output STOP even if the old Twist remains fresh.
        owner=self.mux.owner
        if owner!=self.mux.STOP:
            if not self._fresh(self._odom_last,.35):
                self._reset("odometry lost during velocity lease")
            elif not self._fresh(self._battery_last,10.):
                self._reset("battery telemetry lost during velocity lease")
            elif (self._battery_fraction is None or
                  not math.isfinite(self._battery_fraction) or
                  not .15<=self._battery_fraction<=1.):
                self._reset("unsafe resource telemetry during velocity lease")
            elif not self._fresh(self._last_status,3.):
                self._reset("Nav2 status expired during velocity lease")
            elif owner==self.mux.DOCK and not self._nav_idle():
                self._reset("new Nav2 action started during dock velocity ownership")
            elif owner==self.mux.DOCK and not self._nav_confirmed_terminal():
                self._reset("Nav2 terminal proof expired during dock velocity ownership")
                self._released_nav=False
            elif owner==self.mux.NAV and (
                    self._goal_witness is None or
                    self._goal_witness.fault is not None):
                self._reset("missing/current Nav2 goal UUID evidence")
            elif owner==self.mux.NAV and (
                    self._dock_state!="IDLE" or not self._fresh(self._dock_last,3.)):
                self._reset("dock controller activated during Nav2 velocity ownership")
        try:velocity=self.mux.output(now=time.monotonic())
        except DockHandoffError:
            self._reset("relay clock/motion fault")
            velocity=Velocity()
        # Even though this controller owns the velocity mux, no nonzero
        # NAV2 command may pass unless the exact PRE-ALLOCATED UUID was
        # observed active during this lease and matches the durable
        # GoalJournal. Old latched terminal status is never sufficient.
        if self.mux.owner==self.mux.NAV:
            witnessed=(self._goal_witness is not None
                       and self._goal_witness.current_active_matches()
                       and self._nav_status is not None
                       and any(uuid==self._goal_witness.uuid and status in _ACTIVE
                               for uuid,status in self._nav_status)
                       and self._fresh(self._last_status,1.0))
            if not witnessed:
                velocity=Velocity()
                if (self._nav_lease_started is not None
                    and time.monotonic()-self._nav_lease_started>2.0
                    and not (self._goal_witness is not None
                             and self._goal_witness.can_release_for_dock())):
                    self._reset("Nav2 goal UUID never became active")
        # Only one safe OUTPUT publisher. Competing upstream sources
        # publish on isolated input topics that never reach the motors
        # directly in this lab. Zero on STOP or stale owner command.
        self._latest_output=velocity
        m=Twist()
        m.linear.x=velocity.linear
        m.angular.z=velocity.angular
        self.safe.publish(m)

    def _reset(self,reason):
        if getattr(self,"_three_way",False):
            self.mux.stop(reason)
            self._motion_token=None
            self._prior_terminal=None
        else:
            self.mux.reset(reason)
        self._released_nav=False

    def _on_sweep_state(self,msg):
        self._sweep_state=str(msg.data)
        self._sweep_last=time.monotonic()

    def _on_motion_map(self,msg):
        try:
            raster=Raster.from_ros(msg)
            payload=json.dumps([raster.width,raster.height,raster.resolution,
                raster.origin_x,raster.origin_y,raster.yaw,raster.data],separators=(",",":"))
            digest=hashlib.sha256(payload.encode()).hexdigest()
            if self._map_hash is not None and digest!=self._map_hash:
                raise AdmissionError("static map changed; restart/reconcile lab")
            self._map_hash=digest
            self._map_raster=raster
        except (AdmissionError,ValueError,TypeError):
            self._map_fault=True
            self._reset("invalid/changed motion map")

    def _on_motion_clock(self,msg):
        stamp=msg.clock.sec*1_000_000_000+msg.clock.nanosec
        if stamp<=0 or (self._clock_ns is not None and stamp<self._clock_ns):
            self._clock_fault=True
            self._reset("simulator clock rewound/invalid; restart/reconcile lab")
        elif self._clock_ns is None or stamp>self._clock_ns:
            self._clock_advanced=time.monotonic()
        self._clock_ns=stamp

    def _on_motion_unstable(self,msg):
        self._unstable=bool(msg.data)
        self._unstable_last=time.monotonic()

    def _motion_graph(self):
        topics={self.mux.NAV:"/valdi/nav2_cmd_vel",
                self.mux.SWEEP:"/valdi/sweep_cmd_vel",
                self.mux.DOCK:"/valdi/dock_drive_cmd_vel"}
        observed={}
        for role,topic in topics.items():
            publishers=self.get_publishers_info_by_topic(topic)
            if len(publishers)!=1:
                return False
            gid=bytes(publishers[0].endpoint_gid)
            if not gid or not any(gid):
                return False
            observed[role]=gid
        output=self.get_publishers_info_by_topic("/valdi/lab_safe_cmd_vel_output")
        if (len(set(observed.values()))!=3 or len(output)!=1
            or output[0].node_name!=self.get_name()
            or output[0].node_namespace!=self.get_namespace()
            or self.count_publishers("/cmd_vel")!=0):
            return False
        self._observed_gids=observed
        return self.mux.owner==self.mux.STOP or observed==self._source_gids

    def _motion_health(self):
        pose=False
        try:
            t=self.tf_buffer.lookup_transform("map","base_link",rclpy.time.Time())
            stamp=t.header.stamp.sec*1_000_000_000+t.header.stamp.nanosec
            age=self.get_clock().now().nanoseconds-stamp
            if stamp>0 and 0<=age<=500_000_000 and self._map_hash is not None:
                cell=self._map_raster.cell_at(t.transform.translation.x,
                                               t.transform.translation.y)
                pose=self._map_raster.data[cell]==0
        except (TransformException,AdmissionError,ValueError,AttributeError):
            pass
        idle=self._nav_idle()
        # A cached ACTIVE UUID must not be refreshed by the relay timer.
        # Preserve the existing one-second nonzero-NAV status budget.
        active=(None if not self._fresh(self._last_status,1.) or self._nav_status is None else
                next((uuid for uuid,code in self._nav_status if code in _ACTIVE),None))
        # These fields are derived observations in this disposable simulation.
        # Meter health is not an electrical/MCU safety attestation.
        return MotionHealth(
            pose_fresh=bool(pose),
            resources_ok=(self._fresh(self._battery_last,10.)
                and self._battery_fraction is not None
                and math.isfinite(self._battery_fraction)
                and (.20 if self.mux.owner==self.mux.STOP else .15)
                    <=self._battery_fraction<=1.),
            map_current=self._map_hash is not None and not self._map_fault,
            safety_loop_ok=(not self._clock_fault and self._fresh(self._clock_advanced,.35)
                and self._unstable is False and self._fresh(self._unstable_last,2.5)
                and self._sweep_state in ("IDLE","TRANSIT","SWEEP","ESCAPE")
                and self._fresh(self._sweep_last,1.)
                and self._dock_state in ("IDLE","SEARCH","GOTO","TURN","BACK","REGROUP","DONE")
                and self._fresh(self._dock_last,3.)),
            isolated_sources=self._motion_graph(),
            nav2_idle=idle,nav2_active_goal=active,
            dock_inactive=self._dock_state=="IDLE" and self._fresh(self._dock_last,3.),
            sweep_inactive=self._sweep_state in ("IDLE","TRANSIT")
                and self._fresh(self._sweep_last,1.))

    def _on_motion_cmd(self,source,msg,info):
        now=time.monotonic()
        velocity=Velocity(float(msg.linear.x),float(msg.angular.z))
        # Raw Twist has no lease token. Jazzy's MessageInfo supplies sequence
        # and DDS wall timestamp, but no per-packet publisher GID. Associate
        # with the unique current graph endpoint; do not call it authentication.
        try:
            if not self._motion_graph():
                raise ReactiveMuxError("velocity writer graph changed/nonexclusive")
            if not isinstance(info,dict):
                raise ReactiveMuxError("DDS command metadata unavailable")
            stamp=info["source_timestamp"]
            sequence=info["publication_sequence_number"]
            if (type(stamp) is not int or type(sequence) is not int
                or not 0<sequence<2**63):
                raise ReactiveMuxError("invalid/unsupported DDS source sequence")
            gid=self._observed_gids[source]
            previous=self._command_sequences.get(source)
            if previous is not None and previous[0]==gid and (
                    sequence<=previous[1] or stamp<=previous[2]):
                raise ReactiveMuxError("DDS command replayed/reordered")
            wall=time.time_ns()
            if stamp<=0 or not 0<=wall-stamp<=300_000_000:
                raise ReactiveMuxError("stale/future DDS command timestamp")
            if self.mux.owner==source and stamp<self._claim_wall_ns:
                raise ReactiveMuxError("DDS command predates current velocity lease")
            self._command_sequences[source]=(gid,sequence,stamp)
            self._commands[source]=(now,velocity)
            if source==self.mux.NAV:
                self._nav_zero=velocity==Velocity()
            if self.mux.owner!=source and velocity==Velocity():
                return
            self.mux.accept_twist(source=source,token=self._motion_token,now=now,
                                  velocity=velocity)
        except (AttributeError,KeyError,TypeError,ValueError,ReactiveMuxError) as exc:
            # Nonzero foreign commands revoke ownership. Graph endpoint and
            # sequence observations survive STOP; they are not DDS credentials.
            self._reset(str(exc))

    def _motion_zero(self,source):
        entry=self._commands.get(source)
        return entry is not None and self._fresh(entry[0],.30) and entry[1]==Velocity()

    def _execute_motion(self,body):
        name=body.get("operation")
        now=time.monotonic()
        if name=="stop":
            self._reset("ROS2 explicit STOP")
            return {"ok":True,"state":self.mux.owner}
        health=self._motion_health()
        self.mux.observe_health(now=now,proof=health)
        if name=="claim_nav":
            if self._goal_witness is None:
                raise ReactiveMuxError("durable preallocated Nav2 UUID required")
            record=self._goal_witness.journal.read()
            if (record is None or record["state"]=="terminal"
                or record["map_digest"]!=self._map_hash):
                raise ReactiveMuxError("pending Nav2 UUID/map journal mismatch")
            token=self.mux.claim_transit(now=now,nav_goal_uuid=record["uuid"])
            self._goal_witness.reset()
            self._nav_lease_started=now
        elif name in ("release_nav","release_for_dock"):
            if not self._nav_confirmed_terminal() or not self._motion_zero(self.mux.NAV):
                raise ReactiveMuxError("fresh exact Nav2 terminal/zero not observed")
            uuid=self._goal_witness.uuid
            self.mux.release_transit(now=now,terminal_uuid=uuid,
                terminal_verified=self._nav_confirmed_terminal(),
                nav_input_isolated=health.isolated_sources)
            self._prior_terminal=uuid
            self._motion_token=None
            return {"ok":True,"state":self.mux.owner}
        elif name in ("claim_sweep","claim_dock"):
            if self._goal_witness is None:
                raise ReactiveMuxError("cold-start journal reconciliation required")
            record=self._goal_witness.journal.read()
            reconciled=(health.nav2_idle and (record is None or self._nav_confirmed_terminal()))
            claim=self.mux.claim_sweep if name=="claim_sweep" else self.mux.claim_dock
            token=claim(now=now,nav_released=reconciled,
                        prior_terminal_uuid=self._prior_terminal)
            self._prior_terminal=None
            self._dock_started=False
        elif name=="release_motion":
            owner=self.mux.owner
            inactive=(health.sweep_inactive if owner==self.mux.SWEEP else
                (self._dock_state=="IDLE" and self._fresh(self._dock_last,3.)) or
                (self._dock_started and self._dock_state=="DONE" and self._docked
                 and self._fresh(self._dock_last,3.) and self._fresh(self._docked_last,3.)))
            self.mux.release_non_nav(now=now,token=self._motion_token,
                source_stopped=inactive and self._motion_zero(owner),
                source_isolated=health.isolated_sources)
            self._motion_token=None
            return {"ok":True,"state":self.mux.owner}
        else:
            raise ReactiveMuxError("unsupported three-way lab operation")
        # Only a successful new claim rotates the receiver-side command epoch.
        self._motion_token=token
        self._claim_wall_ns=time.time_ns()
        self._source_gids=dict(self._observed_gids)
        return {"ok":True,"state":self.mux.owner}

    def _tick_motion(self):
        try:
            self.mux.observe_health(now=time.monotonic(),proof=self._motion_health())
            velocity=self.mux.output(now=time.monotonic())
            if self.mux.owner==self.mux.NAV:
                witnessed=(self._goal_witness is not None
                    and self._goal_witness.current_active_matches())
                if not witnessed:
                    velocity=Velocity()
                    if (self._nav_lease_started is not None
                        and time.monotonic()-self._nav_lease_started>2.
                        and not self._nav_confirmed_terminal()):
                        self._reset("Nav2 UUID never became active/verified")
        except (ReactiveMuxError,GoalJournalError) as exc:
            self._reset(str(exc))
            velocity=Velocity()
        self._latest_output=velocity
        message=Twist()
        message.linear.x=velocity.linear
        message.angular.z=velocity.angular
        self.safe.publish(message)


def main():
    rclpy.init()
    node=ExperimentalDockVelocityRelay()
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=="__main__":main()
