"""Small-spot *complete* cleaning regression using OOMWOO's real simulator meter.

Do not confuse this with the separately tested full-home coverage planner:
only a small spot with a few raster cells is targeted, by a prototype
nearest-uncovered Nav2 executor. Full physical cleaning is not claimed.
"""
from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor

from gazebo_measured_coverage_probe import MeasuredOperator
from oomwoo_cleaning_jobs.grid import Grid
from oomwoo_cleaning_jobs.map_admission import Raster, CostmapChecker


def main():
    rclpy.init()
    op=MeasuredOperator()
    exe=SingleThreadedExecutor()
    exe.add_node(op)
    started=False
    req=0
    report={"ok":False,"source":"unmodified OOMWOO ground-truth meter",
            "battery":"synthetic test feed", "test":"small_spot_completion"}

    def until(fn,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            exe.spin_once(timeout_sec=.045)
            if fn():return True
        return False

    def state():
        return op.messages[-1] if op.messages else {}

    def command(action,**kw):
        nonlocal req
        req+=1
        key="smallspot-"+str(req)
        op.command(op=action,request_id=key,**kw)
        if not until(lambda:state().get("last_command",{}).get("request_id")==key,7):
            raise RuntimeError("missing command result")
        return state()["last_command"]

    def ready():
        s=state()
        if not (op.map_msg and op.cost_msg and op.battery and op.clock is not None
                and op.meter_msg is not None and op.meter_unstable is False):
            return False
        if not all(s.get(k) for k in ("nav2_active","map_ready","costmap_fresh",
                                     "battery_fresh","clock_fresh","coverage_ready")):
            return False
        try:op.map_pose()
        except Exception:return False
        return True

    try:
        if not until(ready,36):
            raise RuntimeError("sim/meter/controller not ready: "+json.dumps(state())[-1000:])
        report["initial_credited"]=state()["coverage_credited_cells"]
        assert report["initial_credited"]==0
        mapr=Raster.from_ros(op.map_msg)
        cost=Raster.from_ros(op.cost_msg)
        px,py=op.map_pose()
        start=mapr.cell_at(px,py)
        grid=Grid(mapr.width,mapr.height,mapr.data,
                  resolution_m=mapr.resolution,clearance_cells=5)
        checker=CostmapChecker(mapr,cost,max_cost=25)
        # Short controlled route on open floor, outside the initial
        # preexisting coverage disk. No unbounded BFS search.
        candidates=[]
        for idx in grid.reachable(start):
            x,y=mapr.centre(idx)
            d=math.hypot(x-px,y-py)
            if 1.15<=d<=1.75 and x-px>1.0:
                candidates.append((abs(y-py)*2+abs(d-1.3),idx,x,y))
        candidates.sort()
        selected=None
        for _,idx,x,y in candidates[:20]:
            scope=grid.targets(mode="spot",spot=idx,radius_m=.085)
            route=grid.path_to_nearest(start,scope)
            if checker.route_admitted(route):
                selected=(idx,x,y,len(scope))
                break
        if selected is None:raise RuntimeError("no approved spot")
        idx,x,y,num=selected
        report["target_cells"]=num
        report["target_pose"]=[x,y]
        assert command("arm_fresh_sim",isolated_disposable_sim=True)["ok"]
        response=command("start",mode="spot",job_id="complete_tiny_spot",
                         x=x,y=y,radius_m=.085)
        if not response["ok"]:raise RuntimeError(str(response))
        started=True
        end=time.monotonic()+60
        progress=[]
        last_progress=-1
        while time.monotonic()<end:
            exe.spin_once(timeout_sec=.05)
            s=state()
            job=s.get("interlock",{}).get("job",{})
            count=job.get("cleaned_cells",0)
            if count!=last_progress:
                progress.append({"cleaned_cells":count,
                                 "state":job.get("state"),
                                 "nav_goals":s.get("sent_goals")})
                last_progress=count
            if job.get("state") in ("completed","paused_recovery","error","cancelled"):
                break
            if s.get("sent_goals",0)>12:break
        # A completed coverage checkpoint starts Nav2 cancellation, but
        # the action result arrives asynchronously. Do NOT count PASS before
        # the terminal cancellation has actually been received.
        if state().get("interlock",{}).get("job",{}).get("state")=="completed":
            until(lambda:state().get("interlock",{}).get("active_goal") is None,
                  12)
        s=state()
        job=s.get("interlock",{}).get("job",{})
        report.update({"job_state":job.get("state"),
            "reason":job.get("reason_code"),"cleaned_cells":job.get("cleaned_cells",0),
            "goal_count":s.get("sent_goals"),"external_meter_credited":s.get("coverage_credited_cells"),
            "progress":progress,"frozen":s.get("frozen_reason"),
            "post_job_active_goal":s.get("interlock",{}).get("active_goal"),
            "last_terminal":s.get("last_terminal")})
        report["ok"]=(job.get("state")=="completed"
                     and report["cleaned_cells"]==num
                     and report["external_meter_credited"]==num
                     and report["post_job_active_goal"] is None
                     and report["last_terminal"]=="canceled")
        print("GAZEBO_SMALL_SPOT "+json.dumps(report,sort_keys=True),flush=True)
        if not report["ok"]:raise RuntimeError("small spot did not reach measured complete")
    finally:
        if started and state().get("interlock",{}).get("active_goal"):
            try:
                command("cancel")
                until(lambda:state().get("interlock",{}).get("active_goal") is None,8)
            except Exception:
                print("GAZEBO_SMALL_SPOT_WARNING outstanding goal not confirmed canceled",
                      flush=True)
        exe.shutdown()
        op.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    try:main()
    except Exception as e:
        print("GAZEBO_SMALL_SPOT_FAILURE "+json.dumps({"error":str(e),
            "exception":type(e).__name__}),flush=True)
        raise
