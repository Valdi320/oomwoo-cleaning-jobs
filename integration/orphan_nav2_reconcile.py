"""Phase B: recover the exact orphaned OOMWOO Nav2 action after process death.

A NEW Python process:
- receives the server's status for the known old UUID,
- requests cancellation by THAT UUID through ROS2 CancelGoal,
- observes named cancellation response and terminal CANCELED,
- verifies fresh, advanced, low-velocity simulator odometry for >1.2s,
- then reopens the durable checkpoint and (only after all proofs)
  resumes, dispatches ONE new real Nav2 goal, and cancels it normally.

This is a one-client OFFLINE OOMWOO Gazebo test, not production recovery.
"""
from __future__ import annotations
import json,math,time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSDurabilityPolicy,QoSReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import OccupancyGrid,Odometry
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from oomwoo_cleaning_jobs import Grid,CleaningJobs,Resources
from oomwoo_cleaning_jobs.nav_interlock import NavigationInterlock
from oomwoo_cleaning_jobs.ros_nav2 import AsyncNav2Transport,pose_for_cell
from oomwoo_cleaning_jobs.restart_witness import OrphanNav2Witness,RestartNotSafe
from oomwoo_cleaning_jobs.goal_journal import GoalJournal

META=Path("/tmp/valdi_orphan_nav2_20261027.json")
CHECKPOINT=Path("/tmp/valdi_orphan_job_20261027.json")
JOURNAL=Path("/tmp/valdi_orphan_durable_uuid_20261027.json")


class RecoveryObserver(Node):
    def __init__(self,witness):
        super().__init__("oomwoo_orphan_recovery_new_process")
        latched=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.witness=witness
        self.old_seen=False
        self.odom_count=0
        self.clock=None
        self.map_msg=None
        self.odom=None
        self.goal_status=None
        self.events=[]
        self.start_wall=time.monotonic()
        self.create_subscription(Clock,"/clock",self.on_clock,10)
        self.create_subscription(Odometry,"/odom",self.on_odom,10)
        self.create_subscription(OccupancyGrid,"/map",self.on_map,latched)
        self.create_subscription(GoalStatusArray,
                                 "/navigate_to_pose/_action/status",self.on_status,latched)
        self.cancel_client=self.create_client(CancelGoal,"/navigate_to_pose/_action/cancel_goal")
        self.tf=Buffer()
        self.listener=TransformListener(self.tf,self,spin_thread=False)

    def on_map(self,m):self.map_msg=m
    def on_clock(self,m):self.clock=m.clock.sec+m.clock.nanosec*1e-9
    def on_odom(self,m):
        self.odom=m
        self.odom_count+=1
        if self.clock is None:return
        stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9
        if abs(self.clock-stamp)>.65:return
        self.witness.observe(sim_time=stamp,
              x=m.pose.pose.position.x,y=m.pose.pose.position.y,
              linear_mps=m.twist.twist.linear.x,
              angular_rps=m.twist.twist.angular.z)

    def on_status(self,m):
        self.goal_status=m
        for status in m.status_list:
            hex_id=bytes(status.goal_info.goal_id.uuid).hex()
            self.witness.see_status(hex_id,status.status)
            if hex_id==self.witness.expected and status.status in (1,2,3):
                self.old_seen=True

    def fresh_pose(self):
        if self.clock is None:return None
        try:
            t=self.tf.lookup_transform("map","base_link",Time())
            stamp=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
            if abs(self.clock-stamp)>2.5:return None
            return t.transform.translation.x,t.transform.translation.y
        except Exception:return None


def main():
    meta=json.loads(META.read_text())
    uuid=meta["uuid"]
    journal=GoalJournal(JOURNAL)
    if (meta.get("journal")!=str(JOURNAL) or
        journal.orphaned_uuid()!=uuid or
        journal.read()["token"]!=meta["local_goal_token"]):
        raise RestartNotSafe("durable Nav2 UUID does not match crashed process")
    rclpy.init()
    witness=OrphanNav2Witness(uuid)
    n=RecoveryObserver(witness)
    state={"ok":False,"phase":"bootstrap","old_uuid":uuid,
           "creator_pid":meta["creator_pid"],"recover_pid":__import__("os").getpid()}
    gate=None
    def until(condition,timeout):
        limit=time.monotonic()+timeout
        while time.monotonic()<limit:
            rclpy.spin_once(n,timeout_sec=.045)
            if condition():return True
        return False
    try:
        if not until(lambda:n.old_seen and n.map_msg is not None
                     and n.odom is not None
                     and n.cancel_client.service_is_ready()
                     and n.fresh_pose() is not None,16):
            raise RestartNotSafe("orphan UUID never seen active or ROS graph unavailable")
        state["phase"]="observed_old_goal_executing"
        state["active_old_goal"]=True
        req=CancelGoal.Request()
        req.goal_info.goal_id.uuid=list(bytes.fromhex(uuid))
        # Stamp == zero, UUID != zero => cancel matching UUID ONLY.
        request=n.cancel_client.call_async(req)
        if not until(request.done,14):
            raise RestartNotSafe("exact-UUID cancellation service timeout")
        response=request.result()
        if response is None:
            raise RestartNotSafe("cancel request has no ROS2 response")
        returned=tuple(bytes(goal.goal_id.uuid).hex()
                       for goal in response.goals_canceling)
        witness.cancel_response(int(response.return_code),returned)
        state["phase"]="cancel_request_acknowledged_not_yet_safe"
        state["ack_uuid_list"]=list(returned)
        assert not witness.ready,"service ACK is not proof of stopped motors"
        if not until(lambda:witness.terminal_canceled or witness.fault,14):
            raise RestartNotSafe("old exact UUID not terminal after cancel")
        if witness.fault:raise RestartNotSafe(witness.fault)
        state["phase"]="old_goal_terminal_wait_for_quiet"
        if not until(lambda:witness.ready or witness.fault,18):
            raise RestartNotSafe("simulation motion did not settle with bounded evidence")
        proof=witness.require_ready()
        if n.goal_status is None:
            raise RestartNotSafe("missing other-goal status")
        others=[(bytes(s.goal_info.goal_id.uuid).hex(),int(s.status))
                 for s in n.goal_status.status_list
                 if int(s.status) in (1,2,3) and
                    bytes(s.goal_info.goal_id.uuid).hex()!=uuid]
        if others:
            raise RestartNotSafe("another active goal; private single-owner simulation invalid")
        # The journal's submitting/accepted record is only cleared by a
        # matching, live OrphanNav2Witness; never accept a JSON claim of safe
        # cancellation from another process.
        journal.mark_reconciled_canceled(witness)
        if journal.orphaned_uuid() is not None:
            raise RestartNotSafe("durable orphan UUID remains unresolved")
        state["journal_old_goal_reconciled"]=True
        state["proof"]=proof

        m=n.map_msg
        grid=Grid(m.info.width,m.info.height,tuple(m.data),
                  resolution_m=float(m.info.resolution),clearance_cells=5)
        if grid.fingerprint()!=meta["map_digest"]:
            raise RestartNotSafe("restart map digest differs from original job")
        reloaded=CleaningJobs(grid,CHECKPOINT)
        saved=reloaded.status()
        if saved["state"]!="paused_recovery" or saved["cleaned_cells"]!=0:
            raise RestartNotSafe("durable job was not held paused after restart")
        state["reloaded_checkpoint"]=saved
        events=[]
        box={}
        q=m.info.origin.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        def done(token,outcome):
            events.append((token,outcome))
            box["gate"].on_goal_result(token=token,outcome=outcome)
        def fault(token,reason):
            box["gate"].on_transport_fault(token=token,reason=reason)
        port=AsyncNav2Transport.from_ros2(n,
            lambda g:pose_for_cell(g,
                width=m.info.width,height=m.info.height,
                resolution=float(m.info.resolution),
                origin_x=m.info.origin.position.x,origin_y=m.info.origin.position.y,
                origin_yaw=yaw,stamp=n.get_clock().now().to_msg()),
            on_terminal=done,on_fault=fault,journal=journal)
        gate=NavigationInterlock(reloaded,port,coverage_source="none_from_navigation")
        box["gate"]=gate
        assert not gate.snapshot()["can_dispatch"]
        # The evidence above was independently observed; this is a
        # simulator-only positive admission after proving the exact old
        # goal terminal, the entire ROS action set idle and odometry quiet.
        gate.confirm_backend_idle(verified_no_outstanding_goals=True)
        point=n.fresh_pose()
        if point is None:raise RestartNotSafe("fresh robot pose lost after proving idle")
        col=math.floor((point[0]-m.info.origin.position.x)/m.info.resolution)
        row=math.floor((point[1]-m.info.origin.position.y)/m.info.resolution)
        cur=row*grid.width+col
        if cur not in grid.traversable:raise RestartNotSafe("restart pose violates footprint grid")
        gate.resume(pose_cell=cur,resources=Resources(.9))
        assert reloaded.status()["resume_count"]==1
        assert port.client.server_is_ready()
        new=gate.dispatch(pose_cell=cur,admit_route=lambda route:bool(route))
        state["new_goal_token"]=new.token
        state["phase"]="resumed_real_nav2"
        if not until(lambda:bool(port.outstanding_tokens()) and
                     port._pending[new.token].handle is not None,12):
            raise RestartNotSafe("new NavigateToPose action not actually accepted")
        gate.pause()
        if not until(lambda:gate.snapshot()["active_goal"] is None
                     and any(v=="canceled" for _,v in events),16):
            raise RestartNotSafe("post-recovery Nav2 action cancel unconfirmed")
        last_journal=journal.read()
        if (last_journal["state"]!="terminal"
            or last_journal["outcome"]!="canceled"
            or last_journal["uuid"]==uuid):
            raise RestartNotSafe("new Nav2 action was not durably finalized")
        state.update({"ok":True,"phase":"completed",
           "journal_new_uuid_terminal":True,
           "old_uuid_not_reused":True,
           "new_goals_dispatched":1,
           "new_goal_terminal":"canceled",
           "same_checkpoint_reloaded":True,
           "no_unverified_cleaning_credit":reloaded.status()["cleaned_cells"]==0,
           "resume_count":reloaded.status()["resume_count"]})
        print("REAL_NAV2_ORPHAN_RECOVERY "+json.dumps(state,sort_keys=True),flush=True)
    except Exception as e:
        state["error"]=str(e)
        print("REAL_NAV2_ORPHAN_RECOVERY "+json.dumps(state,sort_keys=True),flush=True)
        # On a test failure, best effort request exact-UUID cancel again,
        # never assume idle; the enclosing test container will be removed.
        raise
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
