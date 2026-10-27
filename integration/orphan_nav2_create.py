"""Phase A: start a real OOMWOO Nav2 action then intentionally lose its client.

Only in the dedicated offline OOMWOO Nav2+Gazebo container. Persist the
checkpoint/real ROS2 goal UUID and emulate abrupt job-controller death.
The separate reconciler will cancel that IDENTIFIED goal via ROS2.
"""
from __future__ import annotations
import json,os,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSDurabilityPolicy
from rclpy.action import ActionClient
from rclpy.time import Time
from tf2_ros import Buffer,TransformListener
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid,Odometry
from rosgraph_msgs.msg import Clock
from oomwoo_cleaning_jobs import Grid,CleaningJobs,Resources
from oomwoo_cleaning_jobs.nav_interlock import NavigationInterlock
from oomwoo_cleaning_jobs.ros_nav2 import AsyncNav2Transport,pose_for_cell
from oomwoo_cleaning_jobs.goal_journal import GoalJournal


PATH=Path("/tmp/valdi_orphan_nav2_20261027.json")
CHECKPOINT=Path("/tmp/valdi_orphan_job_20261027.json")
JOURNAL=Path("/tmp/valdi_orphan_durable_uuid_20261027.json")


def main():
    if PATH.exists() or CHECKPOINT.exists() or JOURNAL.exists():
        raise RuntimeError("Test checkpoint already exists; need new isolated simulator")
    rclpy.init()
    node=Node("oomwoo_orphan_goal_creator")
    latched=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    readings={"map":None,"odom":None,"clock":None,"status":None}
    node.create_subscription(OccupancyGrid,"/map",lambda m:readings.__setitem__("map",m),latched)
    node.create_subscription(Odometry,"/odom",lambda m:readings.__setitem__("odom",m),10)
    node.create_subscription(Clock,"/clock",
       lambda m:readings.__setitem__("clock",m.clock.sec+m.clock.nanosec*1e-9),10)
    from action_msgs.msg import GoalStatusArray
    node.create_subscription(GoalStatusArray,"/navigate_to_pose/_action/status",
       lambda m:readings.__setitem__("status",m),latched)
    buffer=Buffer();tf=TransformListener(buffer,node,spin_thread=False)
    client=ActionClient(node,NavigateToPose,"/navigate_to_pose")

    def until(test,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.07)
            if test():return True
        return False
    def fresh_pose():
        if readings["clock"] is None:return None
        try:
            trans=buffer.lookup_transform("map","base_link",Time())
            stamp=trans.header.stamp.sec+trans.header.stamp.nanosec*1e-9
            if abs(readings["clock"]-stamp)>2.5:return None
            return (trans.transform.translation.x,trans.transform.translation.y)
        except Exception:return None

    try:
        if not until(lambda:readings["map"] is not None
                     and readings["odom"] is not None and fresh_pose() is not None
                     and client.server_is_ready(),26):
            raise RuntimeError("real Nav2/AMCL/map/odom not ready")
        m=readings["map"]
        g=Grid(m.info.width,m.info.height,tuple(m.data),
               resolution_m=float(m.info.resolution),clearance_cells=5)
        x0,y0=fresh_pose()
        def index(x,y):
            col=math.floor((x-m.info.origin.position.x)/m.info.resolution)
            row=math.floor((y-m.info.origin.position.y)/m.info.resolution)
            if not 0<=col<g.width or not 0<=row<g.height:
                raise RuntimeError("target outside map")
            return row*g.width+col
        start=index(x0,y0)
        goal_cell=index(2.45,.10)
        if start not in g.traversable or goal_cell not in g.reachable(start):
            raise RuntimeError("unsafe or unreachable fixed test goal")
        if readings["status"] is not None and any(z.status in (1,2,3)
                                                  for z in readings["status"].status_list):
            raise RuntimeError("another active Nav2 action exists; abort")
        job=CleaningJobs(g,CHECKPOINT)
        started=job.start("orphan_nav2_crash_demo",mode="spot",
                         start_cell=start,spot=goal_cell,radius_m=0.0,
                         resources=Resources(.95))
        journal=GoalJournal(JOURNAL)
        q=m.info.origin.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        box={}
        def on_terminal(token,outcome):
            box["gate"].on_goal_result(token=token,outcome=outcome)
        def on_fault(token,reason):
            box["gate"].on_transport_fault(token=token,reason=reason)
        port=AsyncNav2Transport.from_ros2(
            node,
            lambda goal: pose_for_cell(goal,
               width=m.info.width,height=m.info.height,
               resolution=float(m.info.resolution),
               origin_x=m.info.origin.position.x,
               origin_y=m.info.origin.position.y,
               origin_yaw=yaw,stamp=node.get_clock().now().to_msg()),
            on_terminal=on_terminal,on_fault=on_fault,journal=journal)
        gate=NavigationInterlock(job,port,coverage_source="never_forge_coverage")
        box["gate"]=gate
        assert not gate.snapshot()["can_dispatch"]
        gate.confirm_backend_idle(verified_no_outstanding_goals=True)
        local_goal=gate.dispatch(pose_cell=start,admit_route=lambda route:bool(route))
        # The UUID is now DURABLY WRITTEN BEFORE any async ROS2 acceptance.
        persisted=journal.read()
        if persisted is None or persisted["state"]!="submitting":
            raise RuntimeError("UUID journal missing immediately after dispatch")
        if not until(lambda:port._pending.get(local_goal.token) is not None
                     and port._pending[local_goal.token].handle is not None
                     and journal.read()["state"]=="accepted",15):
            raise RuntimeError("persisted Nav2 goal was never accepted")
        handle=port._pending[local_goal.token].handle
        if not handle.accepted:raise RuntimeError("Nav2 rejected journaled goal")
        hex_id=bytes(handle.goal_id.uuid).hex()
        if hex_id!=journal.read()["uuid"]:
            raise RuntimeError("backend goal ID diverged from durable preallocated UUID")
        snapshot=readings["odom"].pose.pose.position
        ox,oy=snapshot.x,snapshot.y
        def active():
            msg=readings["status"]
            return msg is not None and any(
                bytes(entry.goal_info.goal_id.uuid).hex()==hex_id and entry.status in (1,2,3)
                for entry in msg.status_list)
        if not until(active,6):
            raise RuntimeError("goal UUID not shown active in server status")
        motion=0.
        limit=time.monotonic()+9
        while time.monotonic()<limit and motion<.055:
            rclpy.spin_once(node,timeout_sec=.05)
            p=readings["odom"].pose.pose.position
            motion=max(motion,math.hypot(p.x-ox,p.y-oy))
        if motion<.022:
            raise RuntimeError("Nav2 active but Gazebo robot did not move")
        if not active():
            raise RuntimeError("goal already terminal, cannot emulate orphan")
        meta={
            "uuid":hex_id,"checkpoint":str(CHECKPOINT),
            "start_cell":start,"goal_cell":goal_cell,
            "map_digest":g.fingerprint(),
            "motion_before_crash_m":round(motion,4),
            "creator_pid":os.getpid(),
            "status_active_before_crash":True,
            "journal":str(JOURNAL),
            "uuid_persisted_before_network_send":True,
            "journal_state_at_crash":journal.read()["state"],
            "local_goal_token":local_goal.token,
            "job":started["job_id"],
        }
        PATH.write_text(json.dumps(meta,sort_keys=True)+"\n")
        print("NAV2_ORPHAN_CREATED "+json.dumps(meta,sort_keys=True),flush=True)
        # Abrupt loss intentionally bypasses rclpy shutdown or goal cancellation,
        # matching a client/process crash WITHOUT bringing down Nav2 or Gazebo.
        os._exit(0)
    except:
        node.destroy_node()
        rclpy.shutdown()
        raise


if __name__=="__main__":
    main()
