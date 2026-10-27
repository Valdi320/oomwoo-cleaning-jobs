"""Real ROS2 middleware regression for delayed coverage AFTER Nav2 terminal success.

The ActionServer and covered_grid are SYNTHETIC in this test. A separate
Gazebo regression proves actual OOMWOO motion and original ground-truth meter.
"""
from __future__ import annotations
import json
import tempfile
import time
import threading
from pathlib import Path
import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from ros_verified_meter_probe import MeterFeed,CompletionServer
from ros_persistent_probe import Operator
from oomwoo_cleaning_jobs.ros_experimental_node import ExperimentalCleaningNode


def main():
    rclpy.init()
    server=CompletionServer()
    executor2=MultiThreadedExecutor(num_threads=3)
    executor2.add_node(server)
    t=threading.Thread(target=executor2.spin,daemon=True)
    t.start()
    with tempfile.TemporaryDirectory(prefix="valdi-coverage-tail-") as d:
        node=ExperimentalCleaningNode(parameter_overrides=[
            Parameter("simulation_lab",value=True),
            Parameter("require_nav2_lifecycle",value=False),
            Parameter("coverage_mode",value="sim_ground_truth"),
            Parameter("auto_continue",value=True),
            Parameter("use_sim_time",value=True),
            Parameter("checkpoint_path",value=str(Path(d)/"meter.json"))])
        feed=MeterFeed()
        op=Operator()
        ex=SingleThreadedExecutor()
        for x in (node,feed,op):ex.add_node(x)
        seq=[0]
        def state():return op.received[-1] if op.received else {}
        def until(check,timeout=10):
            end=time.monotonic()+timeout
            while time.monotonic()<end:
                ex.spin_once(timeout_sec=.035)
                if check():return True
            return False
        def send(action,**kwargs):
            seq[0]+=1
            request_id="tail"+str(seq[0])
            op.send(op=action,request_id=request_id,**kwargs)
            if not until(lambda:state().get("last_command",{}).get("request_id")==request_id,7):
                raise AssertionError("command not acknowledged")
            return state()["last_command"]
        try:
            assert until(lambda:state().get("coverage_ready")
                         and state().get("battery_fresh")
                         and state().get("costmap_fresh")
                         and state().get("clock_fresh"),10),state()
            assert state()["coverage_credited_cells"]==0
            assert send("arm_fresh_sim",isolated_disposable_sim=True)["ok"]
            started=send("start",job_id="tail_sample",mode="spot",x=1.7,y=0.,radius_m=0.0)
            assert started["ok"],started
            assert until(lambda:server.accepted==1,8)
            server.complete_first.set()
            assert until(lambda:server.succeeded==1
                         and state().get("last_terminal")=="succeeded",9)
            assert state()["interlock"]["job"]["cleaned_cells"]==0
            # Deliver the external meter update ONLY AFTER Nav2 has already
            # published a terminal succeeded action result.
            idx=node._map.cell_at(1.7,0.)
            feed.covered.add(idx)
            assert until(lambda:state().get("interlock",{}).get("job",{}).get("state")=="completed",8),state()
            final=state()
            assert final["post_success_measured_cells"]==1,final
            assert final["coverage_credited_cells"]==1
            assert final["sent_goals"]==1
            assert server.accepted==1
            assert final["interlock"]["active_goal"] is None
            assert final["interlock"]["can_dispatch"] is False
            print("ROS_DELAYED_COVERAGE_TAIL "+json.dumps({
                "ok":True,"ROS2_middleware":True,
                "source":"SYNTHETIC covered_grid and fake ActionServer",
                "terminal_nav_success_preceded_final_measured_cell":True,
                "late_swept_cells":final["post_success_measured_cells"],
                "job_state":"completed","nav_goals":final["sent_goals"],
                "extra_navigation_prevented":True
            },sort_keys=True),flush=True)
        finally:
            ex.shutdown()
            op.destroy_node()
            feed.destroy_node()
            node.destroy_node()
            executor2.shutdown()
            server.destroy_node()
            t.join(timeout=2)
            rclpy.shutdown()


if __name__=="__main__":
    main()
