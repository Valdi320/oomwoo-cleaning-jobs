"""End-to-end ROS2 test of verified coverage + automatic next goal.

Real ROS2 middleware/action types, but FAKE NavigateToPose server and
SYNTHETIC covered_grid (no Gazebo). This tests orchestration and provenance
fencing independently of the live Gazebo meter regression.
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
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose

from ros_persistent_probe import WorldFeed, Operator
from oomwoo_cleaning_jobs.ros_experimental_node import ExperimentalCleaningNode


class MeterFeed(WorldFeed):
    def __init__(self):
        super().__init__()
        latched=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.covered={40*80+8}  # Preexisting unscoped meter footprint.
        self.meter_pub=self.create_publisher(
            OccupancyGrid,"/coverage_meter/covered_grid",latched)
        self.unstable_pub=self.create_publisher(
            Bool,"/coverage_meter/sim_unstable",latched)
        self.create_timer(.15,self.publish_coverage)

    def publish_coverage(self):
        g=OccupancyGrid()
        g.header=self.raster.header
        g.info=self.raster.info
        data=[0]*6400
        for cell in self.covered:
            data[cell]=100
        g.data=data
        self.meter_pub.publish(g)
        self.unstable_pub.publish(Bool(data=False))


class CompletionServer(rclpy.node.Node):
    def __init__(self):
        super().__init__("valdi_meter_fake_action_server")
        self.accepted=0
        self.canceled=0
        self.succeeded=0
        self.complete_first=threading.Event()
        self.server=ActionServer(
            self,NavigateToPose,"/navigate_to_pose",
            execute_callback=self.execute,
            goal_callback=lambda _:GoalResponse.ACCEPT,
            cancel_callback=lambda _:CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())

    def execute(self,handle):
        self.accepted+=1
        first=self.accepted==1
        deadline=time.monotonic()+28.
        while time.monotonic()<deadline:
            if handle.is_cancel_requested:
                self.canceled+=1
                handle.canceled()
                return NavigateToPose.Result()
            if first and self.complete_first.is_set():
                self.succeeded+=1
                handle.succeed()
                return NavigateToPose.Result()
            time.sleep(.02)
        handle.abort()
        return NavigateToPose.Result()


def main():
    rclpy.init()
    server=CompletionServer()
    server_exec=MultiThreadedExecutor(num_threads=3)
    server_exec.add_node(server)
    thread=threading.Thread(target=server_exec.spin,daemon=True)
    thread.start()
    with tempfile.TemporaryDirectory(prefix="valdi-sim-meter-") as scratch:
        controller=ExperimentalCleaningNode(parameter_overrides=[
            Parameter("simulation_lab",value=True),
            Parameter("require_nav2_lifecycle",value=False), # fake server ONLY
            Parameter("coverage_mode",value="sim_ground_truth"),
            Parameter("auto_continue",value=True),
            Parameter("use_sim_time",value=True),
            Parameter("checkpoint_path",value=str(Path(scratch)/"job.json"))])
        feed=MeterFeed()
        op=Operator()
        exc=SingleThreadedExecutor()
        for x in (controller,feed,op):
            exc.add_node(x)
        number=0

        def until(pred, timeout=10):
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                exc.spin_once(timeout_sec=.04)
                if pred():
                    return True
            return False

        def state():
            return op.received[-1] if op.received else {}

        def command(action,**kwargs):
            nonlocal number
            number+=1
            rid="cov"+str(number)
            op.send(op=action,request_id=rid,**kwargs)
            assert until(lambda:state().get("last_command",{}).get("request_id")==rid,8)
            return state()["last_command"]

        try:
            assert until(lambda:state().get("coverage_ready")
                         and state().get("costmap_fresh")
                         and state().get("clock_fresh")
                         and state().get("battery_fresh"),12),str(state())
            assert state()["coverage_observed_cells"]==1
            assert state()["coverage_credited_cells"]==0
            assert command("arm_fresh_sim",isolated_disposable_sim=True)["ok"]
            started=command("start",job_id="meter_auto",
                            mode="spot",x=1.7,y=0.0,radius_m=0.0)
            assert started["ok"],started
            assert until(lambda:server.accepted>=1,10),"first goal not accepted"
            assert state()["interlock"]["job"]["cleaned_cells"]==0
            # Arrival success alone MUST NOT count floor cleaning.
            server.complete_first.set()
            assert until(lambda:server.succeeded==1 and
                         state().get("last_terminal")=="succeeded",8)
            assert state()["interlock"]["job"]["cleaned_cells"]==0
            # Auto scheduler should retry an uncleaned goal only after a
            # terminal success and the external meter's next observation.
            assert until(lambda:server.accepted>=2 and
                         state().get("accepted_goals",0)>=2,10),str(state())
            assert state()["interlock"]["job"]["cleaned_cells"]==0
            # Now the independent meter reports a NEW covered cell.
            # No goal success can create this evidence itself.
            target=controller._map.cell_at(1.7,0.0)
            feed.covered.add(target)
            assert until(lambda:state().get("interlock",{}).get("job",{}).get("state")=="completed"
                        and server.canceled>=1
                        and state().get("interlock",{}).get("active_goal") is None,12),str(state())
            st=state()
            assert st["interlock"]["job"]["cleaned_cells"]==1
            assert st["coverage_credited_cells"]==1
            assert st["interlock"]["job"]["state"]=="completed"
            assert st["sent_goals"]==2
            print("ROS_COVERAGE_ORCHESTRATION "+json.dumps({
               "ok":True,
               "real_ROS2_action_middleware":True,
               "coverage_source":"synthetic meter (test only)",
               "first_goal_succeeded_without_cleaning_credit":True,
               "next_goal_automatically_sent":True,
               "second_goal_canceled_on_external_measured_completion":True,
               "final_job_state":"completed",
               "credited_meter_cells":st["coverage_credited_cells"],
               "number_of_nav_goals":st["sent_goals"],
               "preexisting_meter_cells_not_credited":True
            },sort_keys=True),flush=True)
        finally:
            exc.shutdown()
            op.destroy_node()
            feed.destroy_node()
            controller.destroy_node()
            server_exec.shutdown()
            server.destroy_node()
            thread.join(timeout=3)
            rclpy.shutdown()


if __name__=="__main__":
    main()
