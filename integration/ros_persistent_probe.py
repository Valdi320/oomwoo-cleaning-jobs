"""Live ROS2 middleware test for the long-lived experimental cleaning-job node.

Runs *real* NavigateToPose ActionClient ↔ ActionServer, actual ROS2
subscriptions and JSON command/status topics, and a synthetic map/costmap/
TF/clock/battery feed. The action server has NO motor/Gazebo output.
"""
from __future__ import annotations

import json
from pathlib import Path
import tempfile
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from rosgraph_msgs.msg import Clock
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from tf2_ros import TransformBroadcaster
from nav2_msgs.action import NavigateToPose

from oomwoo_cleaning_jobs.ros_experimental_node import ExperimentalCleaningNode


def stamp(value: float) -> Time:
    m=Time()
    m.sec=int(value)
    m.nanosec=int((value-int(value))*1e9)
    return m


class WorldFeed(Node):
    def __init__(self):
        super().__init__("valdi_sim_world_feed")
        latched=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.clock_pub=self.create_publisher(Clock,"/clock",10)
        self.map_pub=self.create_publisher(OccupancyGrid,"/map",latched)
        self.cost_pub=self.create_publisher(OccupancyGrid,"/global_costmap/costmap",10)
        self.bat_pub=self.create_publisher(BatteryState,"/battery_state",10)
        self.tf_pub=TransformBroadcaster(self)
        self.epoch=time.monotonic()
        self.raster=self._grid()
        self.create_timer(.05,self.publish_all)

    def _grid(self):
        msg=OccupancyGrid()
        msg.header.frame_id="map"
        msg.info.width=80
        msg.info.height=80
        msg.info.resolution=.10
        msg.info.origin.position.x=-4.
        msg.info.origin.position.y=-4.
        msg.info.origin.orientation.w=1.
        msg.data=[0]*6400
        return msg

    def publish_all(self):
        sim=1000.+time.monotonic()-self.epoch
        c=Clock()
        c.clock=stamp(sim)
        self.clock_pub.publish(c)
        self.raster.header.stamp=c.clock
        self.map_pub.publish(self.raster)
        self.cost_pub.publish(self.raster)
        b=BatteryState()
        b.header.stamp=c.clock
        b.percentage=.83
        self.bat_pub.publish(b)
        t=TransformStamped()
        t.header.frame_id="map"
        t.child_frame_id="base_link"
        t.header.stamp=c.clock
        t.transform.rotation.w=1.
        self.tf_pub.sendTransform(t)


class TestServer(Node):
    def __init__(self):
        super().__init__("valdi_fake_nav2_for_job_node")
        self.accepted=0
        self.canceled=0
        self.started=threading.Event()
        self.server=ActionServer(
            self,NavigateToPose,"/navigate_to_pose",
            execute_callback=self.execute,
            goal_callback=lambda req: GoalResponse.ACCEPT,
            cancel_callback=lambda h: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup())
    def execute(self,handle):
        self.accepted+=1
        self.started.set()
        deadline=time.monotonic()+20.
        while time.monotonic()<deadline:
            if handle.is_cancel_requested:
                self.canceled+=1
                handle.canceled()
                return NavigateToPose.Result()
            time.sleep(.025)
        handle.abort()
        return NavigateToPose.Result()


class Operator(Node):
    def __init__(self):
        super().__init__("valdi_job_operator")
        latched=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                           durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher=self.create_publisher(String,"/valdi/cleaning_jobs/command",10)
        self.received=[]
        self.create_subscription(String,"/valdi/cleaning_jobs/status",
                                 self.on_status,latched)
    def on_status(self,msg):
        self.received.append(json.loads(msg.data))
    def send(self,**fields):
        m=String()
        m.data=json.dumps(fields)
        self.publisher.publish(m)


def run():
    rclpy.init()
    server=TestServer()
    server_exec=MultiThreadedExecutor(num_threads=3)
    server_exec.add_node(server)
    t=threading.Thread(target=server_exec.spin,daemon=True)
    t.start()
    with tempfile.TemporaryDirectory(prefix="valdi-ros-job-") as scratch:
        checkpoint=Path(scratch)/"job.json"
        controller=ExperimentalCleaningNode(parameter_overrides=[
            Parameter("simulation_lab",value=True),
            # Fake ActionServer has no Nav2 lifecycle state service. The real
            # Gazebo suite retains the default lifecycle safety check.
            Parameter("require_nav2_lifecycle",value=False),
            Parameter("use_sim_time",value=True),
            Parameter("checkpoint_path",value=str(checkpoint))])
        feed=WorldFeed()
        operator=Operator()
        operator_exec=SingleThreadedExecutor()
        for n in (controller,feed,operator):
            operator_exec.add_node(n)

        def until(predicate,timeout=13.):
            deadline=time.monotonic()+timeout
            while time.monotonic()<deadline:
                operator_exec.spin_once(timeout_sec=.035)
                if predicate():return True
            return False

        count=0
        def cmd(op,**fields):
            nonlocal count
            count+=1
            rid="case"+str(count)
            operator.send(op=op,request_id=rid,**fields)
            if not until(lambda:any(s.get("last_command",{}).get("request_id")==rid
                                    for s in operator.received[-30:]),8):
                raise AssertionError("no command result: "+op)
            record=next(s for s in reversed(operator.received)
                        if s.get("last_command",{}).get("request_id")==rid)
            return record["last_command"]

        try:
            def graph_ready():
                if not operator.received:
                    return False
                state=operator.received[-1]
                if not all(state.get(k) for k in
                           ("map_ready","costmap_fresh","battery_fresh","clock_fresh")):
                    return False
                try:
                    controller._fresh_pose()
                    return True
                except Exception:
                    return False
            ok=until(graph_ready,15)
            if not ok:
                raise AssertionError("initial ROS2 graph did not become ready: "+
                                     json.dumps(operator.received[-1] if operator.received else {}))
            unauthorized=cmd("start",job_id="rosjob",mode="spot",x=1.7,y=0.)
            assert unauthorized["ok"] is False
            assert "not armed" in unauthorized.get("error","") or "armed" in unauthorized.get("error","")

            armed=cmd("arm_fresh_sim",isolated_disposable_sim=True)
            assert armed["ok"],armed
            started=cmd("start",job_id="rosjob",mode="spot",x=1.7,y=0.)
            assert started["ok"],started
            assert until(lambda:server.accepted>=1,9),"Nav2 action not accepted"
            paused=cmd("pause")
            assert paused["ok"],paused
            assert until(lambda:server.canceled>=1 and
                         controller.snapshot().get("last_terminal")=="canceled",9), \
                   "no confirmed cancel"
            assert controller.snapshot()["interlock"]["job"]["state"]=="paused_user"
            assert not controller.snapshot()["interlock"]["can_dispatch"]

            resumed=cmd("resume")
            assert resumed["ok"],resumed
            assert until(lambda:server.accepted>=2,9),"second goal not accepted"
            canceled=cmd("cancel")
            assert canceled["ok"],canceled
            assert until(lambda:server.canceled>=2
                         and controller.snapshot()["interlock"]["active_goal"] is None,9), \
                   "cancel did not reach terminal"
            status=controller.snapshot()
            assert status["interlock"]["job"]["state"]=="cancelled"
            assert status["interlock"]["job"]["cleaned_cells"]==0
            assert status["sent_goals"]==2
            assert status["accepted_goals"]==2
            assert status["interlock"]["can_dispatch"] is False
            assert checkpoint.is_file()
            # A user command cannot start a second job on the same checkpoint.
            duplicate=cmd("start",job_id="another",mode="spot",x=.5,y=0)
            assert duplicate["ok"] is False
            report={"ok":True,"real_ros2_topics":True,"navigate_to_pose_action":True,
                    "node_remained_alive":True,"unauthorized_start_rejected":True,
                    "explicit_fresh_sim_arming":True,"start_pause_resume_cancel":True,
                    "terminal_cancellations":server.canceled,"accepted_goals":server.accepted,
                    "durable_checkpoint":True,"credited_floor_without_measurement":0,
                    "duplicate_job_rejected":True,"last_state":status["interlock"]["job"]["state"]}
            print("ROS_PERSISTENT_PROBE "+json.dumps(report,sort_keys=True),flush=True)
            return report
        finally:
            operator_exec.shutdown()
            operator.destroy_node()
            feed.destroy_node()
            controller.destroy_node()
            server_exec.shutdown()
            server.destroy_node()
            t.join(timeout=3)
            rclpy.shutdown()


if __name__ == "__main__":
    run()
