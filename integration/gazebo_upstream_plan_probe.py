"""Execute two filtered-upstream OOMWOO sweep waypoints with ONE Nav2 owner.

The ORIGINAL OOMWOO boustrophedon planner geometry was exported without
spinning its executor, so it never sends a Nav2 action. This independent
controller adapts each waypoint to a conservative Grid, admits its route
against a live Nav2 costmap, sends via the real NavigateToPose ActionClient,
and only credits NEW external ground-truth meter evidence (not success).

This is a bounded partial full-map sweep, not finished-room evidence.
"""
from __future__ import annotations

import json,math,tempfile,time,os
from collections import deque
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time
from rclpy.qos import QoSProfile,QoSDurabilityPolicy,QoSReliabilityPolicy
from tf2_ros import Buffer,TransformListener
from nav_msgs.msg import OccupancyGrid,Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool
from action_msgs.msg import GoalStatusArray

from oomwoo_cleaning_jobs import Grid,CleaningJobs,Resources
from oomwoo_cleaning_jobs.map_admission import Raster,CostmapChecker
from oomwoo_cleaning_jobs.upstream_sweep_adapter import adapt_upstream_plan
from oomwoo_cleaning_jobs.nav_interlock import NavigationInterlock
from oomwoo_cleaning_jobs.ros_nav2 import AsyncNav2Transport,pose_for_cell
from oomwoo_cleaning_jobs.goal_journal import GoalJournal
from oomwoo_cleaning_jobs.dock_cycle import DockCycle
from oomwoo_cleaning_jobs.sim_coverage import SimCoverageBridge,CoverageError
from oomwoo_cleaning_jobs.measured_sweep_selector import MeasuredSweepSelector

PLAN=Path(__file__).resolve().parents[1]/".local-sim/oomwoo_upstream_waypoints.json"


class SweepTrialNode(Node):
    def __init__(self):
        super().__init__("oomwoo_safe_upstream_sweep_trial")
        q=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.m=None;self.cm=None;self.odom=None
        self.clock=None;self.unstable=None;self.status=None;self.covered=None
        self.bridge=None;self.gate=None;self.port=None;self.grid=None;self.saved=0
        self.meter_samples=0
        self.motion=deque(maxlen=500)
        self.errors=[]
        self.events=[]
        self.tf=Buffer()
        self.listener=TransformListener(self.tf,self,spin_thread=False)
        self.create_subscription(OccupancyGrid,"/map",lambda x:setattr(self,"m",x),q)
        self.create_subscription(OccupancyGrid,"/global_costmap/costmap",
              lambda x:setattr(self,"cm",x),10)
        self.create_subscription(Odometry,"/odom",self.on_odom,10)
        self.create_subscription(Clock,"/clock",
              lambda x:setattr(self,"clock",x.clock.sec+x.clock.nanosec*1e-9),10)
        self.create_subscription(Bool,"/coverage_meter/sim_unstable",
              lambda x:setattr(self,"unstable",bool(x.data)),q)
        self.create_subscription(OccupancyGrid,"/coverage_meter/covered_grid",
              self.on_covered,q)
        self.create_subscription(GoalStatusArray,"/navigate_to_pose/_action/status",
              lambda x:setattr(self,"status",x),q)

    def on_odom(self,msg):
        self.odom=msg
        self.motion.append((msg.pose.pose.position.x,msg.pose.pose.position.y))

    def pose_xy(self):
        if self.clock is None:return None
        try:
            t=self.tf.lookup_transform("map","base_link",Time())
            stamp=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
            if abs(self.clock-stamp)>2.5:return None
            return t.transform.translation.x,t.transform.translation.y
        except Exception:return None

    def on_covered(self,m):
        self.covered=m
        if self.bridge is None or self.clock is None or self.unstable is None:
            return
        try:
            active=(self.gate is not None and self.port is not None
                    and self.gate.snapshot()["active_goal"] is not None)
            if active:
                token=self.gate.snapshot()["active_goal"]
                item=self.port._pending.get(token)
                accepted=bool(item and item.handle is not None and item.handle.accepted)
            else:token=None;accepted=False
            z=self.bridge.consume(Raster.from_ros(m),
                  stamp_s=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
                  current_sim_s=self.clock,
                  simulation_unstable=self.unstable,
                  active=active and self.gate.job.status()["state"]=="running",
                  nav_goal_accepted=accepted,
                  allowed_targets=frozenset(self.gate.job.state["targets"]) if self.gate else self.grid.traversable)
            self.meter_samples+=1
            if z.new_cells:
                self.gate.acknowledge_coverage(
                     token=token,cells=z.new_cells,
                     producer="official_oomwoo_ground_truth",
                     map_digest=self.grid.fingerprint())
                self.saved+=len(z.new_cells)
        except Exception as exc:
            self.errors.append(str(exc))

    def terminal(self,token,outcome):
        self.events.append((token,outcome))
        if self.gate:self.gate.on_goal_result(token=token,outcome=outcome)

    def fault(self,token,code):
        self.errors.append("transport:"+code)
        if self.gate:self.gate.on_transport_fault(token=token,reason=code)


def main():
    meta=json.loads(PLAN.read_text())
    assert meta["never_spun_planner"] and meta["no_nav2_goal_sent"]
    poses=tuple(tuple(p) for p in meta["target_poses"])
    rclpy.init()
    node=SweepTrialNode()
    def until(check,seconds):
        stop=time.monotonic()+seconds
        while time.monotonic()<stop:
            rclpy.spin_once(node,timeout_sec=.045)
            if check():return True
        return False
    limit=int(os.environ.get("OOMWOO_TEST_WAYPOINT_LIMIT","2"))
    station_test=os.environ.get("OOMWOO_TEST_STATION_SERVICE","0")=="1"
    if station_test and limit<4:
        raise RuntimeError("station-service test requires at least four Nav2 goals")
    if not 1<=limit<=8:
        raise RuntimeError("unbounded or invalid waypoint run limit")
    trials=[]
    r={"ok":False,"upstream_waypoint_count":len(poses),
       "motion_scope":str(limit)+" imported waypoints only"}
    try:
        if not until(lambda:node.m is not None and node.cm is not None
                     and node.odom is not None and node.covered is not None
                     and node.unstable is False and node.pose_xy() is not None,25):
            raise RuntimeError("map, costmap, meter or localization unavailable")
        raster=Raster.from_ros(node.m)
        grid=Grid(raster.width,raster.height,raster.data,
                  resolution_m=raster.resolution,clearance_cells=5)
        node.grid=grid
        converted=adapt_upstream_plan(grid,raster,poses,max_shift_m=.24)
        r["safely_adapted_waypoints"]=len(converted.selected)
        r["unadmitted_original_waypoints"]=len(converted.rejected)
        # Missing upstream targets are NOT silently claimed cleaned.
        assert converted.rejected
        bridge=SimCoverageBridge(raster,grid)
        node.bridge=bridge
        # Establish baseline from current meter, before any Nav2 movement.
        current=node.covered
        node.on_covered(current)
        if not bridge.ready:
            raise RuntimeError("external meter baseline not available")

        checker=CostmapChecker(raster,Raster.from_ros(node.cm),max_cost=25)
        pos=node.pose_xy()
        start=raster.cell_at(*pos)
        if start not in grid.traversable:
            raise RuntimeError("robot not in conservatively traversable area")
        candidates=[]
        for i in converted.selected:
            tx,ty=raster.centre(i)
            d=math.hypot(tx-pos[0],ty-pos[1])
            if not .7<d<2.2:continue
            try:
                route=grid.path_to_nearest(start,frozenset((i,)))
                if checker.route_admitted(route):
                    candidates.append((i,tx,ty))
                    if len(candidates)>=110:break
            except Exception:
                continue
        if len(candidates)<2:raise RuntimeError("not enough costmap-admitted upstream waypoints")
        r["candidates_with_safe_routes"]=len(candidates)
        selector=MeasuredSweepSelector(grid,raster,converted,
                       swath_radius_m=.16,skip_when_covered_fraction=.80)

        with tempfile.TemporaryDirectory(prefix="oomwoo-upstream-") as d:
            job=CleaningJobs(grid,Path(d)/"job.json")
            job.start("upstream_waypoint_trial",mode="whole",start_cell=start,
                      resources=Resources(.9))
            journal=GoalJournal(Path(d)/"uuid.json")
            dock=DockCycle(Path(d)/"sim_dock_transaction.json",
                           job_id="upstream_waypoint_trial",map_digest=grid.fingerprint())
            service_proof=None
            yaw=raster.yaw
            port=AsyncNav2Transport.from_ros2(node,
                 lambda goal:pose_for_cell(goal,width=raster.width,height=raster.height,
                      resolution=raster.resolution,origin_x=raster.origin_x,
                      origin_y=raster.origin_y,origin_yaw=yaw,
                      stamp=node.get_clock().now().to_msg()),
                 on_terminal=node.terminal,on_fault=node.fault,journal=journal)
            gate=NavigationInterlock(job,port,
                 coverage_source="official_oomwoo_ground_truth",cancel_timeout_s=10)
            node.gate=gate;node.port=port
            if not port.wait_for_server(8.):
                raise RuntimeError("real Nav2 action not ready")
            statuses=node.status.status_list if node.status is not None else ()
            if any(s.status in (1,2,3) for s in statuses):
                raise RuntimeError("another active Nav2 goal: exclusive trial aborted")
            gate.confirm_backend_idle(verified_no_outstanding_goals=True)
            previous=None
            skipped_measured=0
            for i,tx,ty in candidates:
                pos=node.pose_xy()
                if pos is None:raise RuntimeError("lost map pose")
                cur=raster.cell_at(*pos)
                if cur not in grid.traversable:
                    raise RuntimeError("current robot pose not safe")
                if node.covered is not None:
                    current_meter=Raster.from_ros(node.covered)
                    measured=selector._coverage_fraction(i,current_meter,
                                                        grid.traversable)
                    if measured>=selector.threshold:
                        skipped_measured+=1
                        continue
                route=grid.path_to_nearest(cur,frozenset((i,)))
                current_checker=CostmapChecker(raster,Raster.from_ros(node.cm),max_cost=25)
                if not current_checker.route_admitted(route):
                    continue
                if previous is not None and math.hypot(tx-previous[0],ty-previous[1])<.35:
                    continue
                goal=gate.dispatch(pose_cell=cur,route_override=route,
                                   admit_route=current_checker.route_admitted)
                if not until(lambda:any(t==goal.token for t,_ in node.events)
                             or node.errors,28):
                    gate.pause()
                    raise RuntimeError("imported Nav2 goal did not finish")
                terminal=next((z for t,z in node.events if t==goal.token),None)
                if terminal!="succeeded":
                    raise RuntimeError("upstream target failed Nav2: "+str(terminal))
                trials.append({"cell":i,"destination":[round(tx,3),round(ty,3)],
                               "terminal":terminal,
                               "measured_cells_at_goal":node.saved})
                previous=(tx,ty)
                if station_test and len(trials)==3:
                    # Real OOMWOO Nav2 goal is now terminal SUCCESS and the
                    # external swept-grid already persisted. The following
                    # station service is intentionally SYNTHETIC: this
                    # Gazebo world has no charging/auto-empty/mop hardware.
                    before=job.status()["cleaned_cells"]
                    assert journal.read()["state"]=="terminal"
                    assert len(port.outstanding_tokens())==0
                    service_hold=gate.observe_resources(Resources(.04))
                    if service_hold["state"]!="paused_service":
                        raise RuntimeError("low battery did not suspend live Nav2 job")
                    dock.request(battery_low=True,bin_full=True,mop_dirty=True,
                        job_paused=True,goal_terminal=True)
                    nonce=dock.state["nonce"]
                    dock.confirm_dock_arrival(docked=True,nav2_terminal=True,
                        station_connected=True)  # simulated dock sensor
                    dock.start_service(station_ack_nonce=nonce,station_ready=True)
                    for observation in range(1,3):
                        dock.observe(battery_fraction=.96,bin_full=False,mop_dirty=False,
                            station_completed=tuple(dock.state["needed"]),
                            station_ack_nonce=nonce,docked=True,
                            observation_seq=observation)
                    dock.release(nav2_idle_verified=True,fresh_pose=True,
                        resources_healthy=True)
                    resume_xy=node.pose_xy()
                    if resume_xy is None:
                        raise RuntimeError("real Nav2 pose lost after synthetic station service")
                    fresh_cell=raster.cell_at(*resume_xy)
                    restored=gate.resume(pose_cell=fresh_cell,resources=Resources(.96))
                    if restored["state"]!="running" or restored["cleaned_cells"]<before:
                        raise RuntimeError("resume erased externally measured coverage")
                    service_proof={
                       "ok":True,"synthetic_station":True,
                       "services":["recharge","empty_bin","wash_mop"],
                       "external_meter_cells_before":before,
                       "external_meter_cells_after":restored["cleaned_cells"],
                       "durable_station_cycles":dock.state["cycles"],
                       "real_nav2_goals_before":len(trials)}
                if len(trials)>=limit:break
            if len(trials)!=limit:
                raise RuntimeError("fewer than requested safe upstream waypoints could be executed")
            gate.pause()
            assert not gate.can_dispatch()
            assert len(port.outstanding_tokens())==0
            assert job.status()["cleaned_cells"]==node.saved
            assert all(t["terminal"]=="succeeded" for t in trials)
            if node.saved<=0:
                raise RuntimeError("original external OOMWOO meter recorded no new cells")
            r.update({"ok":True,"nav_goals_sent":limit,"real_nav2":True,
                "upstream_executor_was_not_spun":True,
                "goal_results":trials,
                "externally_measured_new_cleaned_cells":node.saved,
                "current_job_completed":False,
                "false_coverage_from_nav2_arrival":False,
                "unexpected_faults":node.errors,
                "meter_frames":node.meter_samples,
                "already_measured_waypoints_skipped":skipped_measured,
                "station_service_test":service_proof,
                "final_job_state":job.status()["state"]})
            print("REAL_UPSTREAM_SWEEP "+json.dumps(r,sort_keys=True),flush=True)
    except Exception as e:
        r["error"]=str(e)
        print("REAL_UPSTREAM_SWEEP "+json.dumps(r,sort_keys=True),flush=True)
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
