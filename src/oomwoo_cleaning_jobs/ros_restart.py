"""Asynchronous *simulation-only* recovery of an orphaned Nav2 action UUID.

Works inside a running ROS2 single-threaded executor without blocking
callbacks. A persistent controller supplies a previously fsynced goal UUID
from GoalJournal; ROS2 status/CancelGoal/odom provide independent evidence.

Cannot certify stopped physical motors; may only rearm when every required
ROS2 observation is present and fresh, and no other active goal was seen.
"""
from __future__ import annotations
import time

from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSDurabilityPolicy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

from .restart_witness import OrphanNav2Witness,RestartNotSafe


class AsyncOrphanRecovery:
    def __init__(self,node,known_goal_uuid:str,*,max_wall_s:float=25.):
        if not 5.<max_wall_s<=90.:
            raise ValueError("invalid bounded recovery deadline")
        self.node=node
        self.witness=OrphanNav2Witness(known_goal_uuid)
        self._started=time.monotonic()
        self._deadline=self._started+max_wall_s
        self._clock=None
        self._last_clock_received=None
        self._last_odom_received=None
        self._last_status_received=None
        self._status=None
        self._pending_cancel=None
        self.phase="observing_old_uuid"
        self.error=None
        self._proof=None
        qos=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._status_sub=node.create_subscription(GoalStatusArray,
                "/navigate_to_pose/_action/status",self._on_status,qos)
        self._odom_sub=node.create_subscription(Odometry,"/odom",
                self._on_odom,10)
        self._clock_sub=node.create_subscription(Clock,"/clock",
                self._on_clock,10)
        self._cancel=node.create_client(CancelGoal,
                                        "/navigate_to_pose/_action/cancel_goal")

    def _on_clock(self,msg):
        self._clock=msg.clock.sec+msg.clock.nanosec*1e-9
        self._last_clock_received=time.monotonic()

    def _on_status(self,msg):
        self._status=msg
        self._last_status_received=time.monotonic()
        for x in msg.status_list:
            self.witness.see_status(bytes(x.goal_info.goal_id.uuid).hex(),x.status)

    def _on_odom(self,msg):
        self._last_odom_received=time.monotonic()
        if self._clock is None or self._last_clock_received is None:
            return
        if time.monotonic()-self._last_clock_received>.6:return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if abs(self._clock-stamp)>.65:return
        self.witness.observe(sim_time=stamp,
             x=msg.pose.pose.position.x,y=msg.pose.pose.position.y,
             linear_mps=msg.twist.twist.linear.x,
             angular_rps=msg.twist.twist.angular.z)

    def step(self)->bool:
        """Nonblocking; True once exact old goal terminal + quiet verified."""
        if self.phase=="verified":return True
        if self.phase=="failed":return False
        if time.monotonic()>self._deadline:
            self.error="orphan cancel/quiet deadline elapsed"
            self.phase="failed"
            return False
        if self.witness.fault:
            self.error=self.witness.fault
            self.phase="failed"
            return False

        if self.phase=="observing_old_uuid":
            if not self.witness.active_seen or not self._cancel.service_is_ready():
                return False
            msg=CancelGoal.Request()
            msg.goal_info.goal_id.uuid=list(bytes.fromhex(self.witness.expected))
            try:
                self._pending_cancel=self._cancel.call_async(msg)
            except Exception:
                self.error="exact-UUID cancel service unavailable"
                self.phase="failed"
                return False
            self.phase="waiting_cancel_ack"
        if self.phase=="waiting_cancel_ack":
            if self._pending_cancel is None or not self._pending_cancel.done():
                return False
            try:
                reply=self._pending_cancel.result()
                self.witness.cancel_response(
                    int(reply.return_code),
                    tuple(bytes(g.goal_id.uuid).hex()
                          for g in reply.goals_canceling))
            except Exception as exc:
                self.error="orphan cancellation was not explicitly acknowledged: "+str(exc)
                self.phase="failed"
                return False
            self.phase="waiting_terminal_and_quiet"
        if self.phase=="waiting_terminal_and_quiet":
            if not self.witness.ready:return False
            now=time.monotonic()
            if (self._last_clock_received is None
                or now-self._last_clock_received>1.0
                or self._last_odom_received is None
                or now-self._last_odom_received>.65
                or self._last_status_received is None
                or now-self._last_status_received>3.0):
                return False
            if self._status is None:return False
            others=[g for g in self._status.status_list
                    if g.status in (1,2,3) and
                    bytes(g.goal_info.goal_id.uuid).hex()!=self.witness.expected]
            if others:
                self.error="other active Nav2 action discovered"
                self.phase="failed"
                return False
            self._proof=self.witness.require_ready()
            self.phase="verified"
            return True
        return False

    def result(self):
        if self.phase!="verified":raise RestartNotSafe(self.error or "not verified")
        return self._proof

    def destroy(self):
        for sub in (self._clock_sub,self._odom_sub,self._status_sub):
            self.node.destroy_subscription(sub)
        self.node.destroy_client(self._cancel)
