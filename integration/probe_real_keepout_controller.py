"""End-to-end private ROS2 job controller with verified live Nav2 KeepoutFilter.

Actual OOMWOO test_room Gazebo robot, real filtered global/local Nav2,
a SYNTHETIC battery test publisher, and no physical hardware.
Refuses forbidden goal BEFORE checkpoint/goal creation, then issues one
admissible simulated goal and confirms a real terminal Nav2 cancel.
"""
from __future__ import annotations
import json,time
import rclpy
from rclpy.executors import SingleThreadedExecutor
from gazebo_persistent_probe import Operator

def run():
    rclpy.init()
    op=Operator()
    ex=SingleThreadedExecutor()
    ex.add_node(op)
    seq=0
    report={"ok":False,"robot":"official OOMWOO test_room Gazebo",
            "source":"live Nav2 global/local KeepoutFilter + private ROS2 cleaning controller"}
    def state():return op.messages[-1] if op.messages else {}
    def until(pred,sec):
        end=time.monotonic()+sec
        while time.monotonic()<end:
            ex.spin_once(timeout_sec=.065)
            if pred():return True
        return False
    def command(kind,**fields):
        nonlocal seq
        seq+=1
        key=f"keepout-{seq}"
        op.command(op=kind,request_id=key,**fields)
        if not until(lambda:state().get("last_command",{}).get("request_id")==key,8):
            raise RuntimeError("no command acknowledgment "+kind)
        return state()["last_command"]

    try:
        def ready():
            status=state()
            if not all(status.get(k) for k in
                       ("keepout_filter_verified","nav2_active",
                        "costmap_fresh","battery_fresh","clock_fresh")):
                return False
            try:
                op.map_pose()  # validates fresh real map->base TF.
                return True
            except Exception:
                return False
        if not until(ready,35):
            raise RuntimeError("verified live filter/TF/controller not ready: "+
                               json.dumps(state())[-1200:])
        assert state()["keepout_filter_required"]
        report["mask_and_post_filter_costs_verified"]=True
        arm=command("arm_fresh_sim",isolated_disposable_sim=True)
        if not arm["ok"]:raise RuntimeError("simulation arming refused: "+str(arm))
        forbidden=command("start",job_id="forbidden_must_fail",mode="spot",
                          x=.85,y=.0,radius_m=0.)
        report["forbidden_start_response"]=forbidden
        assert forbidden["ok"] is False,forbidden
        assert ("costmap/keepout" in forbidden.get("error","")),forbidden
        assert state()["sent_goals"]==0
        report["forbidden_goal_never_sent"]=True
        # This fresh separate destination lies well outside the keepout.
        accepted=command("start",job_id="safe_filtered_goal",mode="spot",
                         x=2.02,y=.83,radius_m=0.)
        if not accepted["ok"]:raise RuntimeError("safe goal unexpectedly refused: "+str(accepted))
        if not until(lambda:state().get("accepted_goals",0)>=1,13):
            raise RuntimeError("safe Nav2 goal not accepted")
        assert not state()["interlock"]["can_dispatch"]
        report["safe_goal_accepted"]=True
        paused=command("pause")
        assert paused["ok"],paused
        if not until(lambda:state().get("last_terminal")=="canceled"
                     and state().get("interlock",{}).get("active_goal") is None,14):
            raise RuntimeError("real Nav2 terminal cancellation not received")
        final=state()
        assert final["interlock"]["job"]["state"]=="paused_user"
        assert final["interlock"]["job"]["cleaned_cells"]==0
        assert final["sent_goals"]==1
        assert final["keepout_filter_verified"]
        report["terminal"]=final["last_terminal"]
        report["goals_dispatched"]=final["sent_goals"]
        report["no_fake_cleaning_credit"]=True
        report["ok"]=True
        print("REAL_KEEPOUT_CONTROLLER "+json.dumps(report,sort_keys=True),flush=True)
    except Exception as e:
        report["error"]=str(e)
        print("REAL_KEEPOUT_CONTROLLER "+json.dumps(report,sort_keys=True),flush=True)
        raise
    finally:
        if state().get("interlock",{}).get("active_goal") is not None:
            try:
                command("cancel")
                until(lambda:state().get("interlock",{}).get("active_goal") is None,7)
            except Exception:
                print("REAL_KEEPOUT_CONTROLLER_WARNING cancel not confirmed",flush=True)
        ex.shutdown();op.destroy_node();rclpy.shutdown()


if __name__=="__main__":
    run()
