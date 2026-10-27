"""Real ROS2 typed service integration, experimental namespace only.

Actual ROS2 .srv messages generated offline by rosidl, actual ROS2
service/client middleware, and real action protocol against a harmless
fake NavigateToPose server with synthetic map, clock, battery and TF.
This test NEVER drives the Gazebo robot or hardware.
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
from rclpy.node import Node

from oomwoo_cleaning_jobs_lab_interfaces.srv import (
    StartCleaning,ControlCleaning,GetCleaningStatus)
from oomwoo_cleaning_jobs.ros_experimental_node import ExperimentalCleaningNode
from ros_persistent_probe import WorldFeed,TestServer


def main():
    rclpy.init()
    fake=TestServer()
    x2=MultiThreadedExecutor(num_threads=3)
    x2.add_node(fake)
    t=threading.Thread(target=x2.spin,daemon=True);t.start()
    with tempfile.TemporaryDirectory(prefix="oomwoo-typed-api-") as tmp:
        checkpoint=Path(tmp)/"job.json"
        controller=ExperimentalCleaningNode(parameter_overrides=[
            Parameter("simulation_lab",value=True),
            Parameter("require_nav2_lifecycle",value=False),
            Parameter("enable_typed_lab_api",value=True),
            Parameter("use_sim_time",value=True),
            Parameter("checkpoint_path",value=str(checkpoint))])
        world=WorldFeed()
        client=Node("oomwoo_typed_api_client")
        start=client.create_client(StartCleaning,"/valdi/cleaning_jobs/typed/start")
        ctrl=client.create_client(ControlCleaning,"/valdi/cleaning_jobs/typed/control")
        status=client.create_client(GetCleaningStatus,"/valdi/cleaning_jobs/typed/status")
        ex=SingleThreadedExecutor()
        for n in (controller,world,client):ex.add_node(n)
        def until(pred,seconds=10):
            limit=time.monotonic()+seconds
            while time.monotonic()<limit:
                ex.spin_once(timeout_sec=.05)
                if pred():return True
            return False
        def call(srv,request,seconds=12):
            future=srv.call_async(request)
            if not until(future.done,seconds):
                raise RuntimeError("typed ROS2 service timeout")
            answer=future.result()
            if answer is None:raise RuntimeError("typed service response missing")
            return answer
        def request_op(name,request_id,isolated=False):
            msg=ControlCleaning.Request()
            msg.operation=name
            msg.request_id=request_id
            msg.isolated_disposable_sim=isolated
            return call(ctrl,msg)
        def query_state():
            rq=GetCleaningStatus.Request();rq.request_id="status"
            answer=call(status,rq)
            assert answer.ok
            return json.loads(answer.status_json)
        def new_start(request_id="start-1",x=1.7):
            q=StartCleaning.Request()
            q.request_id=request_id;q.job_id="typed_demo"
            q.mode="spot";q.room_names=[]
            q.x=x;q.y=0.;q.radius_m=0.
            return call(start,q)
        try:
            if not until(lambda:start.service_is_ready() and
                         ctrl.service_is_ready() and status.service_is_ready()
                         and controller.snapshot()["map_ready"]
                         and controller.snapshot()["costmap_fresh"]
                         and controller.snapshot()["battery_fresh"],18):
                raise RuntimeError("typed services/map not ready")
            forbidden=new_start("early")
            assert not forbidden.ok and "armed" in forbidden.error
            assert request_op("arm_fresh_sim","arm",isolated=True).ok
            first=new_start()
            assert first.ok and first.goal_token
            assert until(lambda:fake.accepted>=1,10)
            duplicate=new_start()
            assert duplicate.ok and duplicate.goal_token==first.goal_token
            assert fake.accepted==1,"duplicate typed start sent extra goal"
            bad_replay=new_start("start-1",x=1.9)
            assert not bad_replay.ok and "different command" in bad_replay.error
            assert fake.accepted==1
            paused=request_op("pause","pause-1")
            assert paused.ok
            assert until(lambda:fake.canceled>=1
                         and controller.snapshot()["last_terminal"]=="canceled",11)
            resumed=request_op("resume","resume-1")
            assert resumed.ok
            assert until(lambda:fake.accepted>=2,10)
            canceled=request_op("cancel","cancel-1")
            assert canceled.ok
            assert until(lambda:fake.canceled>=2
                         and controller.snapshot()["interlock"]["active_goal"] is None,12)
            state=query_state()
            assert state["interlock"]["job"]["state"]=="cancelled"
            assert state["interlock"]["job"]["cleaned_cells"]==0
            assert state["sent_goals"]==2
            assert state["typed_api_enabled"] is True
            assert checkpoint.exists()
            print("OOMWOO_TYPED_API "+json.dumps({
                "ok":True,"transport":"real ROS2 srv with rosidl-generated messages",
                "action_server":"test double, no robot motion",
                "control_steps":["arm","start","pause","resume","cancel","status"],
                "retry_deduplicated":True,
                "conflicting_id_rejected":True,
                "accepted_nav_goals":fake.accepted,
                "confirmed_terminal_cancels":fake.canceled,
                "final_state":state["interlock"]["job"]["state"],
                "false_floor_credit":state["interlock"]["job"]["cleaned_cells"],
            },sort_keys=True),flush=True)
        finally:
            ex.shutdown()
            client.destroy_node()
            world.destroy_node()
            controller.destroy_node()
            x2.shutdown()
            fake.destroy_node()
            t.join(timeout=3)
            rclpy.shutdown()


if __name__=="__main__":
    main()
