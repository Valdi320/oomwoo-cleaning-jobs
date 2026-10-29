"""Experimental exclusive Nav2/dock-drive velocity relay for ISOLATED ROS2 labs.

Never launch this unchanged against a real robot: it publishes ONLY the
default /valdi/lab_safe_cmd_vel_output topic, NOT /cmd_vel, and will not
arm unless simulation_lab=true. To move a physical robot, canonical OOMWOO
bringup, trusted ROS2 endpoints and an independent MCU watchdog must be
reviewed and tested. ROS2 status messages are not authenticated.

The existing pure Python ExclusiveVelocityMux provides STOP-default,
lease expiration and non-overlap enforcement. This adapter supplies
actual ROS2 Twist, Odometry and dock status topics plus typed services.
"""
from __future__ import annotations
import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,QoSDurabilityPolicy,QoSReliabilityPolicy)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool,String
from action_msgs.msg import GoalStatusArray
from oomwoo_cleaning_jobs_lab_interfaces.srv import (
    ControlCleaning,GetCleaningStatus)

from .dock_handoff import ExclusiveVelocityMux,Velocity,DockHandoffError
from .lab_topic_guard import LabTopicGuardError, require_lab_output_publisher
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
        if self.get_parameter("simulation_lab").value is not True:
            raise ValueError("command source relay requires isolated simulation_lab")
        output=str(self.get_parameter("output_topic").value)
        if output in ("/cmd_vel","cmd_vel","/diff_drive_controller/cmd_vel_unstamped"):
            raise ValueError("lab relay must not publish directly to real motors")
        if output!="/valdi/lab_safe_cmd_vel_output":
            raise ValueError("unsupported lab output topic")
        self.mux=ExclusiveVelocityMux(max_age_s=.30,quiet_required_s=.75)
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
        self.create_subscription(Twist,"/valdi/nav2_cmd_vel",self._nav_twist,10)
        self.create_subscription(Twist,"/valdi/dock_drive_cmd_vel",self._dock_twist,10)
        self.create_subscription(Odometry,"/odom",self._on_odom,10)
        self.create_subscription(BatteryState,"/battery_state",self._battery,10)
        self.create_subscription(GoalStatusArray,
              "/navigate_to_pose/_action/status",self._on_nav_status,latched)
        self.create_subscription(String,
              "/dock_drive/state",self._on_dock_state,10)
        self.create_subscription(Bool,
              "/dock_drive/docked",self._on_docked,latched)
        self.create_service(ControlCleaning,
              "/valdi/relay/control",self._control)
        self.create_service(GetCleaningStatus,
              "/valdi/relay/status",self._status)
        self.create_timer(.05,self._tick)

    def _on_nav_status(self,msg):
        self._last_status=time.monotonic()
        self._nav_status=tuple(
            (bytes(s.goal_info.goal_id.uuid).hex(),int(s.status))
            for s in msg.status_list)
        if (self._goal_witness is not None and
            (self.mux.owner in (self.mux.NAV, self.mux.DOCK)
             or self._released_nav)):
            self._goal_witness.observe(self._nav_status)
            if self._goal_witness.fault is not None:
                self.mux.reset(self._goal_witness.fault)
                self._released_nav=False

    def _on_dock_state(self,msg):
        self._dock_state=str(msg.data)
        self._dock_last=time.monotonic()
        if (self.mux.owner==self.mux.DOCK and
            (self._dock_state!="DONE" or self._docked)):
            # DockDrive publishes state and docked on separate ROS topics.
            # An arriving DONE state before its latched docked=True must
            # NOT fault an otherwise legitimate confirmation pair.
            try:self.mux.observe_dock(now=time.monotonic(),
                 drive_state=self._dock_state,docked=self._docked)
            except DockHandoffError as exc:self.mux.reset(str(exc))

    def _on_docked(self,msg):
        self._docked=bool(msg.data)
        if (self.mux.owner==self.mux.DOCK and self._dock_state is not None
            and (self._dock_state!="DONE" or self._docked
                 or self.mux.docking_proven)):
            try:self.mux.observe_dock(now=time.monotonic(),
                 drive_state=self._dock_state,docked=self._docked)
            except DockHandoffError as exc:self.mux.reset(str(exc))

    def _on_odom(self,msg):
        self._odom_last=time.monotonic()
        v=msg.twist.twist
        speed=math.hypot(v.linear.x,v.linear.y)
        try:self.mux.observe_odom(now=self._odom_last,
              linear_mps=speed,angular_rps=v.angular.z)
        except DockHandoffError:self.mux.reset("invalid observed odometry")

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
        except DockHandoffError:self.mux.reset("invalid velocity data")

    def _nav_twist(self,msg):self._on_cmd(self.mux.NAV,msg)
    def _dock_twist(self,msg):self._on_cmd(self.mux.DOCK,msg)

    def _execute(self,body):
        name=body.get("operation")
        now=time.monotonic()
        if name=="stop":
            self.mux.reset("ROS2 explicit STOP")
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
                      and self._battery_fraction>=.20))
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
        except (DockHandoffError,CommandReplayError,ValueError) as exc:
            response.ok=False
            response.error=str(exc)
        response.state=self.mux.owner
        return response

    def _status(self,request,response):
        response.ok=True
        response.status_json=json.dumps({
            "mode":"ISOLATED ROS2 SIMULATION ONLY",
            "owner":self.mux.owner,"reason":self.mux.stop_reason,
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
        # Active ownership is a continuing lease, NOT a one-off service
        # promise: loss of odometry, battery or the Nav2 goal-status topic
        # makes the lab output STOP even if the old Twist remains fresh.
        owner=self.mux.owner
        if owner!=self.mux.STOP:
            if not self._fresh(self._odom_last,.35):
                self.mux.reset("odometry lost during velocity lease")
            elif not self._fresh(self._battery_last,10.):
                self.mux.reset("battery telemetry lost during velocity lease")
            elif (self._battery_fraction is None or
                  not math.isfinite(self._battery_fraction) or
                  self._battery_fraction<.15):
                self.mux.reset("unsafe resource telemetry during velocity lease")
            elif not self._fresh(self._last_status,3.):
                self.mux.reset("Nav2 status expired during velocity lease")
            elif owner==self.mux.DOCK and not self._nav_idle():
                self.mux.reset("new Nav2 action started during dock velocity ownership")
            elif owner==self.mux.DOCK and not self._nav_confirmed_terminal():
                self.mux.reset("Nav2 terminal proof expired during dock velocity ownership")
                self._released_nav=False
            elif owner==self.mux.NAV and (
                    self._goal_witness is None or
                    self._goal_witness.fault is not None):
                self.mux.reset("missing/current Nav2 goal UUID evidence")
            elif owner==self.mux.NAV and (
                    self._dock_state!="IDLE" or not self._fresh(self._dock_last,3.)):
                self.mux.reset("dock controller activated during Nav2 velocity ownership")
        try:velocity=self.mux.output(now=time.monotonic())
        except DockHandoffError:
            self.mux.reset("relay clock/motion fault")
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
                    self.mux.reset("Nav2 goal UUID never became active")
        # Only one safe OUTPUT publisher. Competing upstream sources
        # publish on isolated input topics that never reach the motors
        # directly in this lab. Zero on STOP or stale owner command.
        self._latest_output=velocity
        m=Twist()
        m.linear.x=velocity.linear
        m.angular.z=velocity.angular
        self.safe.publish(m)


def main():
    rclpy.init()
    node=ExperimentalDockVelocityRelay()
    try:rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=="__main__":main()
