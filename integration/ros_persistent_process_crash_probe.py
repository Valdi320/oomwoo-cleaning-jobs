"""Kill/restart the actual experimental ROS2 job node during real Nav2 motion.

No host program is killed; this test owns its two child controller PIDs
inside a dedicated offline OOMWOO/Gazebo Podman container. It verifies:
- journaled UUID was persisted before dispatch and stays unresolved on crash
- restored node refuses to resume before reconciling that exact UUID
- ROS2 CancelGoal, terminal CANCELED and fresh low-motion odometry evidence
- subsequent explicit normal resume sends one new goal and cancels it.

BatteryState is SYNTHETIC from this test's Operator. No physical hardware.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess
import sys
import time
import math

import rclpy
from rclpy.executors import SingleThreadedExecutor

from gazebo_persistent_probe import Operator
from oomwoo_cleaning_jobs.goal_journal import GoalJournal

CHK=Path("/tmp/valdi_persistent_crash_recovery_20261027.json")
JOURNAL=Path(str(CHK)+".nav2-goal.json")
LOG_A=Path("/tmp/valdi_crash_recover_first.log")
LOG_B=Path("/tmp/valdi_crash_recover_second.log")


def main():
    if CHK.exists() or JOURNAL.exists():
        raise RuntimeError("need new empty private test world")
    rclpy.init()
    op=Operator()
    executor=SingleThreadedExecutor()
    executor.add_node(op)
    proc=None
    log=None
    rid=0
    report={"ok":False,"scope":"actual OOMWOO Gazebo + real Nav2 + two separate ROS2 job node processes",
            "battery_input":"SYNTHETIC"}
    def state():return op.messages[-1] if op.messages else {}
    def until(predicate,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=.047)
            if predicate():return True
        return False
    def launch(logpath):
        nonlocal log
        args=[sys.executable,"-m","oomwoo_cleaning_jobs.ros_experimental_node",
              "--ros-args","-p","simulation_lab:=true",
              "-p","use_sim_time:=true",
              "-p","checkpoint_path:="+str(CHK)]
        log=logpath.open("w")
        return subprocess.Popen(args,stdin=subprocess.DEVNULL,
                                stdout=log,stderr=subprocess.STDOUT,
                                start_new_session=True,env=os.environ.copy())

    def command(kind,**params):
        nonlocal rid
        rid+=1
        key="crash-"+str(rid)
        op.command(op=kind,request_id=key,**params)
        if not until(lambda:state().get("last_command",{}).get("request_id")==key,9):
            raise RuntimeError("no command acknowledgement "+kind+
                               " phase "+str(state())[-600:])
        return state()["last_command"]

    def ready():
        s=state()
        if not all(s.get(k) for k in
                   ("simulation_lab","nav2_active","map_ready",
                    "costmap_fresh","battery_fresh","clock_fresh")):
            return False
        try:
            op.map_pose()
            return True
        except Exception:return False

    try:
        proc=launch(LOG_A)
        assert until(ready,42),"first ROS2 node not ready"
        first_pid=proc.pid
        assert command("arm_fresh_sim",isolated_disposable_sim=True)["ok"]
        sent=command("start",mode="spot",job_id="crash_then_resume",
                     x=2.45,y=.1,radius_m=0.0)
        if not sent["ok"]:raise RuntimeError("first job start rejected: "+str(sent))
        if not until(lambda:state().get("accepted_goals",0)>=1
                     and state().get("unresolved_nav2_uuid") is not None,14):
            raise RuntimeError("first real Nav2 action not accepted or unjournaled")
        record_before=GoalJournal(JOURNAL).read()
        assert record_before["state"]=="accepted"
        assert record_before["uuid"]==state()["unresolved_nav2_uuid"]
        report["old_uuid"]=record_before["uuid"]
        report["journal_state_before_crash"]=record_before["state"]
        report["first_process_pid"]=first_pid

        if op.odom is None:raise RuntimeError("no Gazebo odometry")
        p0=op.odom.pose.pose.position
        start=(p0.x,p0.y)
        moved=0.
        deadline=time.monotonic()+12
        while time.monotonic()<deadline and moved<.060:
            executor.spin_once(timeout_sec=.045)
            if op.odom is not None:
                p=op.odom.pose.pose.position
                moved=max(moved,math.hypot(p.x-start[0],p.y-start[1]))
        if moved<.03:raise RuntimeError("no real Gazebo movement before crash")
        report["motion_before_crash_m"]=round(moved,4)
        # Kill specifically the child PID started in this function.
        proc.kill()
        proc.wait(timeout=8)
        log.close()
        log=None
        proc=None
        report["crashed_child_only"]=True

        old=GoalJournal(JOURNAL).read()
        assert old["state"]=="accepted"
        assert CHK.is_file()
        # Let DDS disconnect old controller, but do NOT shut down Nav2.
        assert until(lambda:True,.1)
        proc=launch(LOG_B)
        report["replacement_pid"]=proc.pid
        assert proc.pid!=first_pid
        if not until(lambda:state().get("interlock",{}).get("job",{}).get("state")=="paused_recovery"
                     and state().get("unresolved_nav2_uuid")==old["uuid"]
                     and state().get("nav2_active"),29):
            raise RuntimeError("replacement node failed to reattach disarmed checkpoint: "+
                               str(state())[-900:])
        s=state()
        assert not s["armed_disposable_sim"]
        report["checkpoint_restored_paused"]=True
        premature=command("resume")
        assert premature["ok"] is False,premature
        report["early_resume_rejected"]=True
        request=command("reconcile_orphan",isolated_disposable_sim=True)
        assert request["ok"],request
        if not until(lambda:state().get("recovered_orphan_verified")
                     or state().get("frozen_reason") is not None
                     or state().get("recovery_phase")=="failed",38):
            raise RuntimeError("new node did not finish exact-UUID recovery")
        s=state()
        if not s.get("recovered_orphan_verified"):
            raise RuntimeError("reconciliation failed: "+str(s)[-1700:])
        assert s["unresolved_nav2_uuid"] is None
        assert s["armed_disposable_sim"]
        assert s["interlock"]["job"]["state"]=="paused_recovery"
        assert s["interlock"]["idle_reconciled"]
        report["orphan_cancel_and_quiet_verified"]=True
        report["old_uuid_durable_terminal"]=GoalJournal(JOURNAL).read()["outcome"]=="canceled"

        response=command("resume")
        if not response["ok"]:raise RuntimeError("legitimate resume refused: "+str(response))
        if not until(lambda:state().get("accepted_goals",0)>=1
                     and state().get("unresolved_nav2_uuid") is not None,15):
            raise RuntimeError("resumed real Nav2 goal not accepted/journaled")
        new_uuid=state()["unresolved_nav2_uuid"]
        assert new_uuid!=old["uuid"]
        paused=command("pause")
        assert paused["ok"],paused
        if not until(lambda:state().get("interlock",{}).get("active_goal") is None
                     and state().get("last_terminal")=="canceled",16):
            raise RuntimeError("resumed goal was not terminal canceled")
        final=state()
        assert final["interlock"]["job"]["state"]=="paused_user"
        assert final["interlock"]["job"]["resume_count"]==1
        assert final["interlock"]["job"]["cleaned_cells"]==0
        assert GoalJournal(JOURNAL).read()["outcome"]=="canceled"
        report.update({"ok":True,"normal_resume_from_checkpoint":True,
                       "new_uuid":new_uuid,"old_uuid_not_reused":True,
                       "final_terminal":"canceled",
                       "fresh_goal_count":final["accepted_goals"],
                       "resume_count":final["interlock"]["job"]["resume_count"],
                       "no_false_cleaning_credit":True})
        print("REAL_NODE_PROCESS_RECOVERY "+json.dumps(report,sort_keys=True),flush=True)
    except Exception as e:
        report["error"]=str(e)
        print("REAL_NODE_PROCESS_RECOVERY "+json.dumps(report,sort_keys=True),flush=True)
        raise
    finally:
        if proc is not None:
            proc.terminate()
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait(timeout=4)
        if log is not None:log.close()
        executor.shutdown()
        op.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
