"""ROS2 no-progress liveness regression: bounded auto continuation.

Real ROS2 middleware, synthetic coverage grid, fake action server.
If Nav2 repeatedly says "succeeded" without independently verified floor
progress, never loop forever or claim the floor cleaned.
"""
from __future__ import annotations
import json
import tempfile
import threading
import time
from pathlib import Path

import rclpy
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer,GoalResponse,CancelResponse
from nav2_msgs.action import NavigateToPose

from ros_persistent_probe import Operator
from ros_verified_meter_probe import MeterFeed
from oomwoo_cleaning_jobs.ros_experimental_node import ExperimentalCleaningNode


class InstantNavServer(rclpy.node.Node):
    def __init__(self):
        super().__init__("valdi_no_progress_test_server")
        self.count=0
        self.server=ActionServer(self,NavigateToPose,"/navigate_to_pose",
            execute_callback=self.execute,
            goal_callback=lambda _:GoalResponse.ACCEPT,
            cancel_callback=lambda _:CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def execute(self,h):
        self.count+=1
        deadline=time.monotonic()+.20
        while time.monotonic()<deadline:
            if h.is_cancel_requested:
                h.canceled()
                return NavigateToPose.Result()
            time.sleep(.01)
        h.succeed()
        return NavigateToPose.Result()


def main():
    rclpy.init()
    srv=InstantNavServer()
    ex2=MultiThreadedExecutor(num_threads=3)
    ex2.add_node(srv)
    th=threading.Thread(target=ex2.spin,daemon=True)
    th.start()
    with tempfile.TemporaryDirectory(prefix="valdi-no-progress-") as d:
        node=ExperimentalCleaningNode(parameter_overrides=[
            Parameter("simulation_lab",value=True),
            Parameter("require_nav2_lifecycle",value=False),
            Parameter("coverage_mode",value="sim_ground_truth"),
            Parameter("auto_continue",value=True),
            Parameter("max_no_progress_attempts",value=2),
            Parameter("use_sim_time",value=True),
            Parameter("checkpoint_path",value=str(Path(d)/"job.json"))])
        feed=MeterFeed()
        op=Operator()
        ex=SingleThreadedExecutor()
        for x in (node,feed,op):
            ex.add_node(x)
        counter=0
        def state():return op.received[-1] if op.received else {}
        def until(fn,seconds):
            stop=time.monotonic()+seconds
            while time.monotonic()<stop:
                ex.spin_once(timeout_sec=.04)
                if fn():return True
            return False
        def cmd(action,**kw):
            nonlocal counter
            counter+=1
            key=str(counter)
            op.send(op=action,request_id=key,**kw)
            assert until(lambda:state().get("last_command",{}).get("request_id")==key,7)
            return state()["last_command"]
        try:
            assert until(lambda:state().get("coverage_ready")
                         and state().get("costmap_fresh")
                         and state().get("battery_fresh"),10),str(state())
            assert cmd("arm_fresh_sim",isolated_disposable_sim=True)["ok"]
            result=cmd("start",job_id="no_progress",mode="spot",
                        x=1.7,y=0.,radius_m=0)
            assert result["ok"],result
            assert until(lambda:state().get("interlock",{}).get("job",{}).get("state")
                         =="paused_recovery",12),str(state())
            final=state()
            assert final["frozen_reason"]=="COVERAGE_NO_PROGRESS"
            assert final["no_progress_successes"]==2
            assert final["interlock"]["job"]["cleaned_cells"]==0
            assert final["sent_goals"]==2
            assert srv.count==2
            assert not final["interlock"]["can_dispatch"]
            print("ROS_NO_PROGRESS_PROBE "+json.dumps({
                "ok":True,"ROS2_middleware":True,"source":"synthetic meter + fake Nav2 server",
                "terminal_nav_successes":2,"measured_cells":0,
                "paused_reason":"COVERAGE_NO_PROGRESS","third_goal_prevented":True
            },sort_keys=True),flush=True)
        finally:
            ex.shutdown()
            op.destroy_node()
            feed.destroy_node()
            node.destroy_node()
            ex2.shutdown()
            srv.destroy_node()
            th.join(timeout=2)
            rclpy.shutdown()


if __name__=="__main__":
    main()
