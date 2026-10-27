"""Complete a PERSISTED OPERATOR-LABELED virtual room in REAL OOMWOO Gazebo.

Actual ROS2 long-running ExperimentalCleaningNode + original OOMWOO
upstream sweep waypoints (geometry only, NEVER a competing executor) +
actual Nav2 + original OOMWOO ground-truth swept-grid meter.
BatteryState is SYNTHETIC. No real physical walls enclose the
operator-defined virtual room inside official test_room.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from gazebo_measured_coverage_probe import MeasuredOperator


ROOM_PATH=os.environ.get("OOMWOO_ROOM_PLAN","/tmp/oomwoo_room_lab_20261027.json")
PLAN_PATH="/workspace/valdi-cleaning-jobs/.local-sim/oomwoo_upstream_waypoints.json"
CHECKPOINT=os.environ.get("OOMWOO_ROOM_CHECKPOINT","/tmp/valdi_room_autorun_20261027.json")
ROOM_NAMES=tuple(os.environ.get("OOMWOO_ROOM_NAMES","Zona_simulada_A").split(","))
LOG=Path(CHECKPOINT+".node.log")


def run():
    if Path(CHECKPOINT).exists() or Path(CHECKPOINT+".nav2-goal.json").exists():
        raise RuntimeError("room test checkpoint already exists; use fresh test container")
    if not Path(ROOM_PATH).is_file() or not Path(PLAN_PATH).is_file():
        raise RuntimeError("missing original map room label or upstream exported planner")
    args=[sys.executable,"-m","oomwoo_cleaning_jobs.ros_experimental_node",
          "--ros-args","-p","simulation_lab:=true",
          "-p","use_sim_time:=true",
          "-p","require_nav2_lifecycle:=true",
          "-p","coverage_mode:=sim_ground_truth",
          "-p","room_autorun:=true",
          "-p","auto_continue:=false",
          "-p","max_no_progress_attempts:=5",
          "-p","room_plan_path:="+ROOM_PATH,
          "-p","upstream_sweep_path:="+PLAN_PATH,
          "-p","checkpoint_path:="+CHECKPOINT]
    out=LOG.open("w")
    proc=subprocess.Popen(args,stdout=out,stderr=subprocess.STDOUT,
                          stdin=subprocess.DEVNULL,start_new_session=True)
    rclpy.init()
    op=MeasuredOperator()
    ex=SingleThreadedExecutor()
    ex.add_node(op)
    rid=0
    started=False
    result={"ok":False,
        "target":list(ROOM_NAMES),
        "scope":"OPERATOR-DEFINED DIGITAL ROOM, physically OPEN test_room",
        "source":"real official OOMWOO Gazebo+Nav2+ground-truth meter",
        "battery":"synthetic isolated test"}
    def state():return op.messages[-1] if op.messages else {}
    def until(pred,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            ex.spin_once(timeout_sec=.045)
            if pred():return True
        return False
    def cmd(action,**kw):
        nonlocal rid
        rid+=1
        request_id="room-real-"+str(rid)
        op.command(op=action,request_id=request_id,**kw)
        if not until(lambda:state().get("last_command",{}).get("request_id")==request_id,9):
            raise RuntimeError("ROS2 room controller no response "+action)
        return state()["last_command"]
    def ready():
        s=state()
        if not all(s.get(k) for k in ("nav2_active","map_ready","costmap_fresh",
                                    "battery_fresh","clock_fresh","coverage_ready")):
            return False
        try:op.map_pose()
        except Exception:return False
        return True
    try:
        if not until(ready,48):
            raise RuntimeError("Nav2, ground-truth meter or experimental room node not ready: "+json.dumps(state())[-1700:])
        assert state().get("room_autorun") is True
        result["initial_credited"]=state()["coverage_credited_cells"]
        assert result["initial_credited"]==0
        arm=cmd("arm_fresh_sim",isolated_disposable_sim=True)
        if not arm["ok"]:raise RuntimeError("fresh simulator refused arm: "+str(arm))
        start=None
        # TF2 may briefly report an AMCL/odom extrapolation during startup.
        # Retry ONLY before a job exists, with a NEW request ID, and NEVER
        # waive the controller's actual fresh-pose safety requirement.
        for attempt in range(8):
            start=cmd("start",mode="rooms",rooms=list(ROOM_NAMES),
                      job_id="room_real_"+str(len(ROOM_NAMES)))
            if start["ok"]:break
            error=str(start.get("error",""))
            if ("ExtrapolationException" not in error
                or Path(CHECKPOINT).exists()
                or Path(CHECKPOINT+".nav2-goal.json").exists()):
                break
            until(lambda:False,.55)
        if not start["ok"]:raise RuntimeError("room start failed: "+str(start))
        started=True
        result["requested_cells"]=start["room_scope_cells"]
        trace=[]
        last_cells=-1
        max_seconds=float(os.environ.get("OOMWOO_ROOM_BENCH_MAX_SECONDS","202"))
        if not 60<=max_seconds<=480:raise RuntimeError("unbounded room test duration")
        end=time.monotonic()+max_seconds
        stop_reason="wall_time_limit"
        while time.monotonic()<end:
            ex.spin_once(timeout_sec=.05)
            status=state()
            job=status.get("interlock",{}).get("job",{})
            cells=job.get("cleaned_cells",0)
            if cells!=last_cells:
                trace.append({"covered":cells,"goals":status.get("sent_goals"),
                              "job_state":job.get("state"),"room_program":status.get("room_program",{}).get("state")})
                last_cells=cells
            if status.get("frozen_reason") is not None:
                stop_reason="controller_frozen"
                break
            if (job.get("state")=="completed"
                and status.get("interlock",{}).get("active_goal") is None
                and status.get("room_program",{}).get("state")=="completed"):
                stop_reason="all_selected_rooms_externally_verified"
                break
            if status.get("sent_goals",0)>60:
                stop_reason="bounded_nav2_goals_exceeded"
                break
        s=state()
        job=s.get("interlock",{}).get("job",{})
        room=s.get("room_program",{})
        result.update({
            "room_eligible_cells":room.get("selected_scope_cells"),
            "room_verified_cells":room.get("verified_cells"),
            "job_credited_cells":job.get("cleaned_cells"),
            "job_state":job.get("state"),"room_state":room.get("state"),
            "nav2_goals":s.get("sent_goals"),
            "nav2_accepted":s.get("accepted_goals"),
            "nav2_active_after":s.get("interlock",{}).get("active_goal"),
            "room_coverage":room.get("coverage_fraction"),
            "room_progress":room.get("room_progress"),
            "requested_room_names":list(ROOM_NAMES),
            "pending_wal_events":room.get("pending_coverage_events"),
            "frozen_reason":s.get("frozen_reason"),
            "meter_frames":len(op.measurements),
            "last_pose_error":s.get("last_pose_error"),
            "nav_lifecycle_positive_age_s":s.get("nav2_lifecycle_last_positive_age_s"),
            "repair_routes_issued":room.get("repair_routes_issued"),
            "no_progress_successes":s.get("no_progress_successes"),
            "stop_condition":stop_reason,
            "progress_trace":trace,
            "final_command":s.get("last_command")
        })
        result["ok"]=(
            result["room_eligible_cells"]==result["requested_cells"]
            and result["room_verified_cells"]==result["requested_cells"]
            and result["job_credited_cells"]==result["requested_cells"]
            and result["job_state"]=="completed"
            and result["room_state"]=="completed"
            and result["nav2_active_after"] is None
            and result["pending_wal_events"]==0
            and result["frozen_reason"] is None
        )
        print("REAL_ROOM_AUTORUN "+json.dumps(result,sort_keys=True),flush=True)
        if not result["ok"]:raise RuntimeError("persisted digital room not fully measured/completed")
    except Exception as err:
        result["error"]=str(err)
        print("REAL_ROOM_AUTORUN_FAILURE "+json.dumps(result,sort_keys=True),flush=True)
        raise
    finally:
        if started and state().get("interlock",{}).get("active_goal"):
            try:
                cmd("cancel")
                until(lambda:state().get("interlock",{}).get("active_goal") is None,10)
            except Exception: pass
        proc.terminate()
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill();proc.wait(timeout=4)
        out.close()
        ex.shutdown();op.destroy_node();rclpy.shutdown()


if __name__=="__main__":
    run()
