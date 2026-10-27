"""End-to-end persistent ROS2 cleaning-jobs node ↔ real Nav2 ↔ OOMWOO Gazebo.

Commands live on the experimental private /valdi/cleaning_jobs/command topic.
This is an isolated simulation only; no physical robot or cleaned-floor proof.
"""
from __future__ import annotations
import json
import math
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from oomwoo_cleaning_jobs.grid import Grid
from oomwoo_cleaning_jobs.map_admission import Raster, CostmapChecker


class Operator(Node):
    def __init__(self):
        super().__init__("valdi_real_gazebo_job_probe")
        latched=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.messages=deque(maxlen=250)
        self.map_msg=None
        self.cost_msg=None
        self.odom=None
        self.battery=None
        self.clock=None
        self.poses=deque(maxlen=1000)
        self.tf_buffer=Buffer()
        self.tf_listener=TransformListener(self.tf_buffer,self,spin_thread=False)
        self.pub=self.create_publisher(String,"/valdi/cleaning_jobs/command",10)
        # The official Gazebo world has no live BatteryState publisher.
        # Emit explicitly SYNTHETIC battery telemetry inside isolated CI only.
        self._sim_battery_pub=self.create_publisher(BatteryState,"/battery_state",10)
        self._sim_battery_fraction=.83
        self.create_timer(.2,self._publish_test_battery)
        self.create_subscription(String,"/valdi/cleaning_jobs/status",self._status,latched)
        self.create_subscription(OccupancyGrid,"/map",lambda m:setattr(self,"map_msg",m),latched)
        self.create_subscription(OccupancyGrid,"/global_costmap/costmap",lambda m:setattr(self,"cost_msg",m),10)
        self.create_subscription(Odometry,"/odom",self._odom,10)
        self.create_subscription(BatteryState,"/battery_state",lambda m:setattr(self,"battery",m),10)
        self.create_subscription(Clock,"/clock",lambda m:setattr(self,"clock",m.clock.sec+m.clock.nanosec*1e-9),10)

    def _publish_test_battery(self):
        b=BatteryState()
        b.percentage=self._sim_battery_fraction
        self._sim_battery_pub.publish(b)

    def _status(self,m):
        try:self.messages.append(json.loads(m.data))
        except ValueError:pass

    def _odom(self,m):
        self.odom=m
        self.poses.append((time.monotonic(),m.pose.pose.position.x,
                          m.pose.pose.position.y, m.twist.twist.linear.x))

    def command(self,**args):
        m=String()
        m.data=json.dumps(args)
        self.pub.publish(m)

    def map_pose(self):
        t=self.tf_buffer.lookup_transform("map","base_link",Time())
        sim_time=t.header.stamp.sec+t.header.stamp.nanosec*1e-9
        if self.clock is None or abs(self.clock-sim_time)>3.0:
            raise RuntimeError("stale robot map pose")
        return t.transform.translation.x,t.transform.translation.y


def run():
    rclpy.init()
    node=Operator()
    exe=SingleThreadedExecutor()
    exe.add_node(node)
    started=False
    rid=0
    measurements={}

    def until(pred,timeout=15):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            exe.spin_once(timeout_sec=.065)
            if pred():return True
        return False

    def state():
        return node.messages[-1] if node.messages else {}

    def send(op,**extra):
        nonlocal rid
        rid+=1
        request_id="phase3-"+str(rid)
        node.command(op=op,request_id=request_id,**extra)
        if not until(lambda:state().get("last_command",{}).get("request_id")==request_id,9):
            raise RuntimeError("no status acknowledgement for "+op)
        return state()["last_command"]

    def pose_ready():
        try:
            if not (node.map_msg is not None and node.cost_msg is not None
                    and node.clock is not None and node.odom is not None
                    and node.battery is not None and state().get("simulation_lab")
                    and state().get("nav2_active")
                    and state().get("costmap_fresh") and state().get("battery_fresh")
                    and state().get("clock_fresh")):
                return False
            node.map_pose()
            return True
        except Exception:
            return False

    try:
        if not until(pose_ready,40):
            raise RuntimeError("sim, costmap, battery or controller node not ready: "+
                               json.dumps(state())[-1700:])
        m=Raster.from_ros(node.map_msg)
        cm=Raster.from_ros(node.cost_msg)
        px,py=node.map_pose()
        cell=m.cell_at(px,py)
        grid=Grid(m.width,m.height,m.data,resolution_m=m.resolution,clearance_cells=5)
        checker=CostmapChecker(m,cm,max_cost=25)
        possible=[]
        for idx in grid.reachable(cell):
            gx,gy=m.centre(idx)
            distance=math.hypot(gx-px,gy-py)
            if 1.6<distance<3.0 and gx-px>1.0:
                possible.append((abs(gy-py)*2+abs(distance-2.2),idx,gx,gy))
        possible.sort()
        target=None
        for _,i,gx,gy in possible[:90]:
            path=grid.path_to_nearest(cell,frozenset({i}))
            if checker.route_admitted(path):
                target=(i,gx,gy,len(path))
                break
        if target is None:
            raise RuntimeError("no sufficiently clear route in live Nav2 costmap")
        measurements["selected_target"]={"index":target[0],"x":target[1],"y":target[2],
                                         "path_cells":target[3]}
        arm=send("arm_fresh_sim",isolated_disposable_sim=True)
        if not arm["ok"]:raise RuntimeError("arming: "+str(arm))
        result=send("start",job_id="gazebo_persistent",mode="spot",x=target[1],y=target[2])
        if not result["ok"]:raise RuntimeError("start: "+str(result))
        started=True
        measurements["start_result"]=result
        if not until(lambda:state().get("accepted_goals",0)>=1,12):
            raise RuntimeError("Nav2 never accepted first navigation goal")
        p=node.odom.pose.pose.position
        x0,y0=p.x,p.y
        maxmotion=0.0
        deadline=time.monotonic()+16
        while time.monotonic()<deadline and maxmotion<.065:
            exe.spin_once(timeout_sec=.045)
            if node.odom:
                p=node.odom.pose.pose.position
                maxmotion=max(maxmotion,math.hypot(p.x-x0,p.y-y0))
        if maxmotion<.035:
            raise RuntimeError("Nav2 accepted but robot never moved in Gazebo")
        measurements["moving_before_pause_m"]=round(maxmotion,5)
        pause=send("pause")
        if not pause["ok"]:raise RuntimeError("pause: "+str(pause))
        if not until(lambda:state().get("last_terminal")=="canceled"
                     and state().get("interlock",{}).get("active_goal") is None,12):
            raise RuntimeError("first Nav2 cancellation was not terminal")
        if state()["interlock"]["job"]["state"]!="paused_user":
            raise RuntimeError("job not user paused")
        # Confirm simulator odometry has settled for at least 1 simulated second.
        t0=time.monotonic()
        if not until(lambda:node.odom is not None
                     and abs(node.odom.twist.twist.linear.x)<.03,8):
            raise RuntimeError("simulation velocity never settled after pause")
        p=node.odom.pose.pose.position
        x1,y1=p.x,p.y
        t1=node.clock
        maxdrift=0.0
        while node.clock-t1<1.2 and time.monotonic()-t0<13:
            exe.spin_once(timeout_sec=.045)
            if node.odom:
                p=node.odom.pose.pose.position
                maxdrift=max(maxdrift,math.hypot(p.x-x1,p.y-y1))
        measurements["post_pause_drift_m"]=round(maxdrift,5)
        if maxdrift>.10:
            raise RuntimeError("post-cancel simulator drift exceeded 10cm")
        if state()["interlock"]["can_dispatch"]:
            raise RuntimeError("pausing did not block new navigation")
        # Resume verifies a fresh transform, resource status and live costmap.
        resume=send("resume")
        if not resume["ok"]:raise RuntimeError("resume: "+str(resume))
        if not until(lambda:state().get("accepted_goals",0)>=2,12):
            raise RuntimeError("resumed Nav2 goal was not accepted")
        # Real ROS2 subscription, synthetic sensor stimulus: simulate battery
        # dropping to 4% while the second Nav2 goal is active.
        node._sim_battery_fraction=.04
        if not until(lambda:state().get("interlock",{}).get("job",{}).get("state")=="paused_service"
                     and state().get("interlock",{}).get("job",{}).get("reason_code")=="LOW_BATTERY"
                     and state().get("interlock",{}).get("active_goal") is None,13):
            raise RuntimeError("battery-low did not cause terminal Nav2 cancellation")
        measurements["auto_low_battery_cancel"]=True
        # The resource is now restored by the simulated sensor (not a bypass).
        node._sim_battery_fraction=.83
        if not until(lambda:node.battery is not None and node.battery.percentage>.8,5):
            raise RuntimeError("restored simulator battery telemetry not observed")
        resume_after_battery=send("resume")
        if not resume_after_battery["ok"]:
            raise RuntimeError("resume after battery service: "+str(resume_after_battery))
        if not until(lambda:state().get("accepted_goals",0)>=3,12):
            raise RuntimeError("third Nav2 goal after simulated recharge missing")
        cancel=send("cancel")
        if not cancel["ok"]:raise RuntimeError("cancel: "+str(cancel))
        if not until(lambda:state().get("interlock",{}).get("active_goal") is None
                     and state().get("interlock",{}).get("job",{}).get("state")=="cancelled",12):
            raise RuntimeError("Nav2 terminal cancellation on third goal absent")
        settled=state()
        measurements.update({
            "ok":True,
            "genuine_nav2":True,"genuine_gazebo":True,
            "battery_data_source":"synthetic ROS2 BatteryState published by test harness",
            "accepted_goals":settled["accepted_goals"],
            "sent_goals":settled["sent_goals"],
            "final_state":settled["interlock"]["job"]["state"],
            "last_terminal":settled["last_terminal"],
            "unverified_cleaned_cells":settled["interlock"]["job"]["cleaned_cells"],
            "no_automatic_new_goal":not settled["interlock"]["can_dispatch"],
            "clock_sim_s":node.clock,
        })
        assert measurements["accepted_goals"]==3
        assert measurements["sent_goals"]==3
        assert measurements["unverified_cleaned_cells"]==0
        assert measurements["last_terminal"]=="canceled"
        print("GAZEBO_PERSISTENT_PROBE "+json.dumps(measurements,sort_keys=True),flush=True)
        return measurements
    finally:
        if started and state().get("interlock",{}).get("active_goal") is not None:
            try:
                send("cancel")
                until(lambda:state().get("interlock",{}).get("active_goal") is None,8)
            except Exception:
                print("GAZEBO_PERSISTENT_PROBE_WARNING unconfirmed cancellation",flush=True)
        exe.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    try:run()
    except Exception as exc:
        print("GAZEBO_PERSISTENT_PROBE "+json.dumps({
            "ok":False,"error":str(exc),"type":type(exc).__name__}),flush=True)
        raise
