"""Fail-closed test: kill ONLY private FilterInfoServer in offline lab.

The robot job is already paused. An externally removed filter source must
make the controller refuse resume, without sending any Nav2 goal.
"""
from __future__ import annotations
import json,os,signal,time
from pathlib import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from gazebo_persistent_probe import Operator


def kill_only_private_filter_info():
    wanted="valdi_keepout_info_server"
    victims=[]
    for f in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            args=f.read_bytes().replace(b"\0",b" ").decode(errors="ignore")
            if ("/opt/ros/jazzy/lib/nav2_map_server/costmap_filter_info_server" in args
                and "__node:="+wanted in args):
                victims.append((int(f.parent.name),args))
        except (OSError,ValueError):
            continue
    if len(victims)!=1:
        raise RuntimeError("expected exactly one owned test FilterInfoServer")
    pid,_=victims[0]
    os.kill(pid,signal.SIGTERM)
    return pid


def main():
    rclpy.init()
    op=Operator()
    ex=SingleThreadedExecutor()
    ex.add_node(op)
    i=0
    def state():return op.messages[-1] if op.messages else {}
    def until(fn,sec):
        deadline=time.monotonic()+sec
        while time.monotonic()<deadline:
            ex.spin_once(timeout_sec=.04)
            if fn():return True
        return False
    def command(kind):
        nonlocal i
        i+=1
        key="failure-"+str(i)
        op.command(op=kind,request_id=key)
        if not until(lambda:state().get("last_command",{}).get("request_id")==key,8):
            raise RuntimeError("command not acknowledged")
        return state()["last_command"]
    try:
        if not until(lambda:state().get("keepout_filter_verified")
                     and state().get("interlock",{}).get("job",{}).get("state")=="paused_user"
                     and state().get("interlock",{}).get("active_goal") is None,18):
            raise RuntimeError("no safe paused job with filter currently proven")
        before=state()
        pid=kill_only_private_filter_info()
        if not until(lambda:state().get("keepout_filter_verified") is False,12):
            raise RuntimeError("lost filter_info server wasn't detected")
        outcome=command("resume")
        after=state()
        if outcome.get("ok") is not False or "keepout" not in outcome.get("error","").lower():
            raise RuntimeError("resume error not specifically from lost KeepoutFilter: "+str(outcome))
        assert after["sent_goals"]==before["sent_goals"]
        assert after["interlock"]["job"]["state"]=="paused_user"
        assert after["interlock"]["active_goal"] is None
        report={"ok":True,"source":"only owned isolated FilterInfoServer terminated",
            "owned_filter_info_pid":pid,
            "detected_filter_loss":True,
            "resumption_denied":True,
            "new_nav_goals":0,
            "paused_job_preserved":True,
            "error":outcome.get("error")}
        print("REAL_FILTER_LOSS "+json.dumps(report,sort_keys=True),flush=True)
    finally:
        ex.shutdown()
        op.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
