"""End-to-end measured-coverage orchestration with real OOMWOO ground_truth meter.

Live OOMWOO Gazebo + Nav2 + official coverage_meter (unmodified).
BatteryState is *synthetic test input*; swept cells come from the official
ground-truth pose estimator, never from Nav2 goal success.
"""
from __future__ import annotations

import json
import math
import time
from collections import deque

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

from gazebo_persistent_probe import Operator
from oomwoo_cleaning_jobs.grid import Grid
from oomwoo_cleaning_jobs.map_admission import Raster, CostmapChecker


class MeasuredOperator(Operator):
    def __init__(self):
        super().__init__()
        latched=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.meter_msg=None
        self.meter_unstable=None
        self.measurements=deque(maxlen=200)
        self.create_subscription(OccupancyGrid,"/coverage_meter/covered_grid",
                                 self._on_meter,latched)
        self.create_subscription(Bool,"/coverage_meter/sim_unstable",
                                 lambda m:setattr(self,"meter_unstable",bool(m.data)),latched)

    def _on_meter(self,m):
        self.meter_msg=m
        self.measurements.append((time.monotonic(),sum(v==100 for v in m.data)))


def run():
    rclpy.init()
    op=MeasuredOperator()
    exec=SingleThreadedExecutor()
    exec.add_node(op)
    rid=0
    started=False
    report={"ok":False,"source":"official OOMWOO simulator ground-truth coverage_meter",
            "battery_source":"synthetic BatteryState from isolated harness"}
    def state():
        return op.messages[-1] if op.messages else {}

    def until(predicate, timeout=16):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            exec.spin_once(timeout_sec=.055)
            if predicate():return True
        return False

    def command(action,**params):
        nonlocal rid
        rid+=1
        request="verified-"+str(rid)
        op.command(op=action,request_id=request,**params)
        if not until(lambda:state().get("last_command",{}).get("request_id")==request,8):
            raise RuntimeError("no controller command response for "+action)
        return state()["last_command"]

    def ready():
        if not (op.map_msg and op.cost_msg and op.battery and op.clock is not None
                and op.meter_msg is not None and op.meter_unstable is False):
            return False
        status=state()
        if not all(status.get(key) for key in
                   ("nav2_active","map_ready","costmap_fresh","battery_fresh",
                    "clock_fresh","coverage_ready")):
            return False
        try:
            op.map_pose()
            return True
        except Exception:
            return False

    try:
        if not until(ready,38):
            raise RuntimeError("Nav2/meter/controller never became ready: "+
                               json.dumps(state())[-1700:])
        report["initial_meter_covered"]=sum(v==100 for v in op.meter_msg.data)
        report["initial_job_credited"]=state().get("coverage_credited_cells")
        assert report["initial_job_credited"]==0, "false pre-start cleaning credit"
        m=Raster.from_ros(op.map_msg)
        cm=Raster.from_ros(op.cost_msg)
        px,py=op.map_pose()
        start=m.cell_at(px,py)
        grid=Grid(m.width,m.height,m.data,resolution_m=m.resolution,clearance_cells=5)
        checker=CostmapChecker(m,cm,max_cost=25)
        # First rank cheap geometric candidates, then perform expensive
        # BFS/footprint admission for only the best few. The previous harness
        # tried full BFS for every candidate (~10k) and could stall for >100s.
        candidates=[]
        for i in grid.reachable(start):
            x,y=m.centre(i)
            d=math.hypot(x-px,y-py)
            if not 1.7<=d<=2.45 or x-px<1.4:
                continue
            candidates.append((abs(y-py)*2+abs(d-1.90),i,x,y))
        candidates.sort()
        selected=None
        for _,i,x,y in candidates[:24]:
            spot_targets=grid.targets(mode="spot",spot=i,radius_m=.24)
            path=grid.path_to_nearest(start,spot_targets)
            if checker.route_admitted(path):
                selected=(i,x,y,len(spot_targets))
                break
        if selected is None:
            raise RuntimeError("no traversable test spot found")
        idx,x,y,count=selected
        report["spot"]={"cell":idx,"x":x,"y":y,"radius_m":.24,"target_cells":count}
        arm=command("arm_fresh_sim",isolated_disposable_sim=True)
        if not arm["ok"]:raise RuntimeError("arming failed: "+str(arm))
        start_resp=command("start",mode="spot",job_id="verified_swept_meter",
                           x=x,y=y,radius_m=.24)
        if not start_resp["ok"]:raise RuntimeError("start failed: "+str(start_resp))
        started=True
        if not until(lambda:state().get("accepted_goals",0)>=1,12):
            raise RuntimeError("Nav2 did not accept first sweep route")
        report["first_goal_accepted"]=True
        baseline_cells=report["initial_job_credited"]
        # Live OOMWOO meter is 1 Hz. Observe evidence during actual motion.
        if not until(lambda:state().get("coverage_credited_cells",0)>baseline_cells
                     or state().get("frozen_reason") is not None
                     or state().get("interlock",{}).get("job",{}).get("state")
                       in ("completed","paused_recovery"),36):
            raise RuntimeError("real OOMWOO meter did not credit any swept cells")
        report["observed_new_cells"]=state().get("coverage_credited_cells",0)
        report["first_sweep_state"]=state().get("interlock",{}).get("job",{}).get("state")
        if report["observed_new_cells"]<=0:
            raise RuntimeError("job did not receive verified meter cells: "+json.dumps(state())[-1700:])
        if state().get("frozen_reason"):
            raise RuntimeError("coverage measurement froze controller: "+
                               str(state()["frozen_reason"]))
        report["coverage_source_confirmed"]=True

        # Force low battery only if job still RUNNING. Cleaned credit must
        # survive pause and resume; never claim the job is fully complete.
        paused_before=state()["interlock"]["job"]["cleaned_cells"]
        if state()["interlock"]["job"]["state"]=="running":
            op._sim_battery_fraction=.04
            if not until(lambda:state().get("interlock",{}).get("job",{}).get("state")=="paused_service"
                         and state().get("interlock",{}).get("active_goal") is None,12):
                raise RuntimeError("simulated battery pause failed to cancel action")
            report["battery_interrupted_after_coverage"]=True
            report["checkpoint_cells_on_pause"]=state()["interlock"]["job"]["cleaned_cells"]
            assert report["checkpoint_cells_on_pause"]>=paused_before
            op._sim_battery_fraction=.83
            if not until(lambda:op.battery is not None and op.battery.percentage>.8,5):
                raise RuntimeError("simulated battery service not restored")
            res=command("resume")
            if not res["ok"]:raise RuntimeError("resume after service: "+str(res))
            if not until(lambda:state().get("accepted_goals",0)>=2,12):
                raise RuntimeError("resume did not schedule a new Nav2 goal")
            report["resume_after_battery"]=True
            assert state()["interlock"]["job"]["cleaned_cells"]>=paused_before
        else:
            report["battery_interrupted_after_coverage"]=False
            report["resume_after_battery"]=False

        # Let integrated auto-continue run after successful Nav2 results.
        # End with an explicit cancel only when work remains in progress.
        deadline=time.monotonic()+22
        while time.monotonic()<deadline:
            exec.spin_once(timeout_sec=.065)
            job=state().get("interlock",{}).get("job",{})
            if job.get("state") in ("completed","paused_recovery","cancelled"):
                break
            if state().get("accepted_goals",0)>=3:
                break
        report["final_before_cleanup"]=state().get("interlock",{}).get("job")
        report["goals_accepted"]=state().get("accepted_goals",0)
        report["measured_cells_total"]=state().get("coverage_credited_cells",0)
        report["auto_continue_enabled"]=state().get("auto_continue")
        report["no_progress_count"]=state().get("no_progress_successes")
        assert report["measured_cells_total"]>0
        assert report["auto_continue_enabled"] is True
        assert report["final_before_cleanup"]["cleaned_cells"]>0
        # Explicitly stop any still active job; no extra unverified credit.
        if state().get("interlock",{}).get("job",{}).get("state")=="running":
            cancel=command("cancel")
            if not cancel["ok"]:raise RuntimeError("cleanup cancel refused")
            if not until(lambda:state().get("interlock",{}).get("active_goal") is None,12):
                raise RuntimeError("cleanup cancel not terminal")
        report["last_state"]=state().get("interlock",{}).get("job",{}).get("state")
        report["last_meter_state"]=state().get("coverage_last_state")
        report["ok"]=True
        print("GAZEBO_MEASURED_COVERAGE "+json.dumps(report,sort_keys=True),flush=True)
        return report
    finally:
        if started and state().get("interlock",{}).get("active_goal"):
            try:
                command("cancel")
                until(lambda:state().get("interlock",{}).get("active_goal") is None,8)
            except Exception:
                print("GAZEBO_MEASURED_COVERAGE_WARNING: unconfirmed stop",flush=True)
        exec.shutdown()
        op.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    try:run()
    except Exception as e:
        print("GAZEBO_MEASURED_COVERAGE "+json.dumps({"ok":False,
            "error":str(e),"type":type(e).__name__}),flush=True)
        raise
