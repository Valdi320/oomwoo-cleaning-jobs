"""Headless Gazebo + REAL OOMWOO Nav2 controlled cancellation regression.

Dry preflight selects a safe reachable goal on the official test_room map.
Cancel mode dispatches through NavigationInterlock + real rclpy ActionClient,
waits for simulator motion, requests cancellation, awaits terminal result
and checks movement settles. THIS IS NOT PHYSICAL-ROBOT SAFETY VALIDATION.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import tempfile
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener
from rclpy.time import Time

from oomwoo_cleaning_jobs import CleaningJobs, Grid, Resources
from oomwoo_cleaning_jobs.nav_interlock import NavigationInterlock
from oomwoo_cleaning_jobs.ros_nav2 import AsyncNav2Transport, pose_for_cell


def world_to_cell(info, x, y):
    q=info.origin.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    dx=x-info.origin.position.x
    dy=y-info.origin.position.y
    c,s=math.cos(yaw),math.sin(yaw)
    local_x=c*dx+s*dy
    local_y=-s*dx+c*dy
    col,row=math.floor(local_x/info.resolution),math.floor(local_y/info.resolution)
    if not (0<=col<info.width and 0<=row<info.height):
        raise ValueError("pose outside of map")
    return row*info.width+col


def cell_to_world(info, cell):
    q=info.origin.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    col,row=cell%info.width,cell//info.width
    a=(col+.5)*info.resolution
    b=(row+.5)*info.resolution
    return (info.origin.position.x+math.cos(yaw)*a-math.sin(yaw)*b,
            info.origin.position.y+math.sin(yaw)*a+math.cos(yaw)*b)


class ProbeNode(Node):
    def __init__(self):
        super().__init__("valdi_oomwoo_real_nav2_cancel_probe")
        self.map_msg=None
        self.amcl=None
        self.odom=None
        self.clock=None
        self.goal_status=None
        self.tf_buffer=Buffer()
        self.tf_listener=TransformListener(self.tf_buffer,self,spin_thread=False)
        self.odom_history=deque(maxlen=2000)
        self.cmd_history=deque(maxlen=2000)
        reliable_latched=QoSProfile(depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,"/map",self.cb_map,reliable_latched)
        self.create_subscription(PoseWithCovarianceStamped,"/amcl_pose",self.cb_amcl,reliable_latched)
        self.create_subscription(Odometry,"/odom",self.cb_odom,10)
        self.create_subscription(Clock,"/clock",self.cb_clock,10)
        self.create_subscription(Twist,"/cmd_vel",self.cb_cmd,10)
        self.create_subscription(GoalStatusArray,
                                 "/navigate_to_pose/_action/status",
                                 self.cb_status,reliable_latched)
    def cb_map(self,msg):self.map_msg=msg
    def cb_amcl(self,msg):self.amcl=msg
    def cb_clock(self,msg):self.clock=msg.clock.sec+msg.clock.nanosec*1e-9
    def cb_odom(self,msg):
        self.odom=msg
        self.odom_history.append((time.monotonic(), msg.pose.pose.position.x,
                                  msg.pose.pose.position.y, msg.twist.twist.linear.x))
    def cb_cmd(self,msg):
        self.cmd_history.append((time.monotonic(),msg.linear.x,msg.angular.z))
    def cb_status(self,msg):
        self.goal_status=msg


def choose_target(grid, info, start, x, y):
    reachable=grid.reachable(start)
    candidates=[]
    for i in reachable:
        gx,gy=cell_to_world(info,i)
        distance=math.hypot(gx-x,gy-y)
        if not 1.3<=distance<=3.2:continue
        # Prefer a long, mostly straight path with a large cancel window.
        if (gx-x)<0.8:continue
        candidates.append((abs(gy-y)*2+abs(distance-2.35),i,distance,gx,gy))
    if not candidates:
        for i in reachable:
            gx,gy=cell_to_world(info,i)
            d=math.hypot(gx-x,gy-y)
            if 1.4<=d<=3.2:
                candidates.append((abs(d-2.1)+abs(gy-y),i,d,gx,gy))
    if not candidates:raise RuntimeError("no sufficiently distant safe cells")
    candidates.sort()
    return candidates[0][1],candidates[0][2:]


def run(mode):
    rclpy.init()
    node=ProbeNode()
    executor=SingleThreadedExecutor()
    executor.add_node(node)
    events=[]
    errors=[]
    accepted=[]
    report={"ok":False,"mode":mode,"step":"preflight","nav2_target":None}

    def until(predicate,timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            executor.spin_once(timeout_sec=.08)
            if predicate():return True
        return False

    try:
        def fresh_map_pose():
            try:
                t=node.tf_buffer.lookup_transform("map","base_link",Time())
                stamp=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
                if node.clock is None or abs(node.clock-stamp)>2.5:
                    return None
                return t
            except Exception:
                return None
        ready=until(lambda: node.map_msg is not None and node.odom is not None
                    and node.clock is not None and fresh_map_pose() is not None,30)
        if not ready:
            report["error"]="not all map/amcl/odom/clock streams received"
            report["streams"]={k:getattr(node,k) is not None for k in ("map_msg","amcl","odom","clock")}
            report["fresh_tf"]=fresh_map_pose() is not None
            return report
        map_msg=node.map_msg
        info=map_msg.info
        # 0.25m Chebyshev static clearance; Nav2 owns real dynamic footprint.
        grid=Grid(int(info.width),int(info.height),tuple(map_msg.data),
                  resolution_m=float(info.resolution),clearance_cells=5)
        t=fresh_map_pose()
        px=t.transform.translation.x
        py=t.transform.translation.y
        start=world_to_cell(info,px,py)
        report.update({"start_cell":start, "start_map_pose":[px,py],
                       "map_size":[info.width,info.height], "map_origin":[info.origin.position.x,info.origin.position.y],
                       "map_resolution":info.resolution,"start_traversable":start in grid.traversable,
                       "sim_time_s":node.clock})
        if start not in grid.traversable:
            report["error"]="start pose fails conservative map clearance"
            return report
        target,(distance,gx,gy)=choose_target(grid,info,start,px,py)
        report["nav2_target"]={"cell":target,"x":gx,"y":gy,"distance_m":distance}
        report["reachable_cells"]=len(grid.reachable(start))
        if mode=="preflight":
            report.update({"ok":True,"step":"target_selected"})
            return report

        with tempfile.TemporaryDirectory(prefix="oomwoo-nav2-gazebo-") as temp:
            job=CleaningJobs(grid,Path(temp)/"job.json")
            job.start("sim_nav2_cancel",mode="spot",spot=target,start_cell=start,
                      radius_m=0.0,resources=Resources(.9))
            box={}
            def on_terminal(token,outcome):
                events.append((token,outcome))
                box["gate"].on_goal_result(token=token,outcome=outcome)
            def on_fault(token,reason):
                errors.append((token,reason))
                box["gate"].on_transport_fault(token=token,reason=reason)

            q=info.origin.orientation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            def build_pose(g):
                return pose_for_cell(g,width=int(info.width),height=int(info.height),
                   resolution=float(info.resolution),
                   origin_x=float(info.origin.position.x),origin_y=float(info.origin.position.y),
                   origin_yaw=yaw,stamp=node.get_clock().now().to_msg())

            port=AsyncNav2Transport.from_ros2(node,build_pose,
                on_terminal=on_terminal,on_fault=on_fault,
                on_accepted=lambda token: accepted.append(token))
            gate=NavigationInterlock(job,port,coverage_source="never_fake_coverage",
                                     cancel_timeout_s=12.)
            box["gate"]=gate

            if not until(lambda:port.client.server_is_ready(),12):
                report["error"]="Nav2 action server not ready"
                return report
            actives={GoalStatus.STATUS_ACCEPTED,GoalStatus.STATUS_EXECUTING,
                     GoalStatus.STATUS_CANCELING}
            statuses=([entry.status for entry in node.goal_status.status_list]
                      if node.goal_status is not None else [])
            if any(x in actives for x in statuses):
                report["error"]="Nav2 had an existing active goal: refused to test"
                report["statuses"]=statuses
                return report
            # Dedicated fresh, isolated Gazebo/Nav2 process with no cleaning
            # executor and no prior goal in this regression. Empty status alone
            # is NOT proof of backend idle on a real robot or after restart.
            report["backend_idle_basis"]="isolated fresh test Nav2; no prior test goals"
            report["status_message_seen"]=node.goal_status is not None
            gate.confirm_backend_idle(verified_no_outstanding_goals=True)
            report["step"]="dispatch"
            goal=gate.dispatch(pose_cell=start,admit_route=lambda r:bool(r) and r[0]==start)
            report["goal_token"]=goal.token
            report["pre_cancel_could_dispatch"]=gate.can_dispatch()
            early_mode=(mode == "early")
            if early_mode:
                # Called before the executor processes send_goal_async acceptance.
                gate.pause()
                report["early_cancel_queued_before_acceptance"]=True

            if not until(lambda:len(accepted)>0 or len(events)>0 or len(errors)>0,15):
                report["error"]="Nav2 never accepted/rejected goal"
                return report
            report["accepted"]=bool(accepted)
            if events and not early_mode:
                report.update({"error":"goal finished before cancellation",
                               "terminal":events[-1][1]})
                return report
            # Allow real simulated movement: once there is odom delta >5cm
            # OR non-zero commanded motion, whichever occurs first. Avoid
            # waiting until the goal finishes in fast simulation.
            p0=node.odom.pose.pose.position
            movement=0.0
            commanded=False
            deadline=time.monotonic()+(0 if early_mode else 18)
            while time.monotonic()<deadline and not events:
                executor.spin_once(timeout_sec=.05)
                if node.odom:
                    p=node.odom.pose.pose.position
                    movement=max(movement,math.hypot(p.x-p0.x,p.y-p0.y))
                if node.cmd_history and abs(node.cmd_history[-1][1])>0.05:
                    commanded=True
                if movement>=.07:
                    break
            report["pre_cancel_movement_m"]=round(movement,4)
            report["pre_cancel_commanded"]=commanded
            if events:
                report.update({"error":"Nav2 goal terminated before test cancel",
                               "terminal":events[-1][1]})
                return report
            report["step"]="cancel_request"
            if mode == "battery":
                gate.observe_resources(Resources(.05))
            elif not early_mode:
                gate.pause()
            report["paused_state"]=job.status()["state"]
            report["cancel_pending"]=gate.snapshot()["awaiting_terminal_result"]
            if gate.can_dispatch():
                report["error"]="interlock armed on pause"
                return report
            if not until(lambda:events or errors or gate.check_timeout(),22):
                report["error"]="Nav2 terminal cancellation timed out"
                return report
            report["terminal_events"]=[v for _,v in events]
            report["faults"]=errors
            report["terminal_state"]=gate.snapshot()
            if not events or events[-1]!=(goal.token,"canceled"):
                report["error"]="no confirmed canceled terminal result"
                return report
            if port.outstanding_tokens():
                report["error"]="transport retained canceled action"
                return report
            expected_state=("paused_service" if mode=="battery" else "paused_user")
            if job.status()["state"] != expected_state:
                report["error"]="job did not remain paused after cancellation"
                return report
            # Wait until odometry reports rest; then assess physical displacement
            # over additional simulated time (not solely ROS action terminal).
            report["step"]="physical_stop_validation"
            rest=False
            t0=time.monotonic()
            while time.monotonic()-t0<12:
                executor.spin_once(timeout_sec=.05)
                if node.odom and abs(node.odom.twist.twist.linear.x)<.03:
                    rest=True
                    break
            report["rest_velocity_seen"]=rest
            if not rest:
                report["error"]="robot did not reach low wheel speed"
                return report
            start_p=node.odom.pose.pose.position
            start_sim=node.clock
            positions=[]
            deadline=time.monotonic()+14
            while time.monotonic()<deadline:
                executor.spin_once(timeout_sec=.05)
                if node.odom:
                    positions.append((node.odom.pose.pose.position.x,
                                      node.odom.pose.pose.position.y))
                if node.clock is not None and node.clock-start_sim>=1.2:break
            displacement=max((math.hypot(x-start_p.x,y-start_p.y)
                            for x,y in positions),default=0)
            report["post_cancel_max_drift_m"]=round(displacement,5)
            report["post_cancel_sim_window_s"]=round(node.clock-start_sim,3)
            report["remaining_cleaned_cells"]=job.status()["cleaned_cells"]
            report["can_dispatch_without_resume"]=gate.can_dispatch()
            report["new_goal_transmitted"]=len(accepted)>1
            report["goal_sent_count"]=len(accepted)
            report["ok"]=(rest and (movement>.02 or early_mode) and displacement<.10
                and node.clock-start_sim>=1.0 and not errors
                and len(events)==1 and events[0][1]=="canceled"
                and job.status()["cleaned_cells"]==0
                and not gate.can_dispatch() and len(accepted)==1)
            if report["ok"]:report["step"]="passed"
            else:report["error"]="physical stop or motion checks did not meet thresholds"
            return report
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--mode",choices=("preflight","cancel","battery","early"),default="preflight")
    args=parser.parse_args()
    try:
        report=run(args.mode)
        print("GAZEBO_NAV2_PROBE "+json.dumps(report,sort_keys=True,default=str),flush=True)
        raise SystemExit(0 if report["ok"] else 1)
    except Exception as e:
        print("GAZEBO_NAV2_PROBE "+json.dumps({"ok":False,"type":type(e).__name__,
                                               "error":str(e)}),flush=True)
        raise
