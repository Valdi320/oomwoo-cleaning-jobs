"""Real OOMWOO Gazebo movement around experimental Nav2 KeepoutFilter.

Uses NavigateToPose in a PRIVATE offline ROS_DOMAIN_ID with robot spawned at
the known test_room pose. Observes the robot's live TF map->base_link and
cancels if its center enters the forbidden mask or a test timeout occurs.
AMCL localization is initialized using the known test-room spawn.
Do not infer hardware localization quality from this simulator.
"""
from __future__ import annotations
import json,math,time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSDurabilityPolicy
from tf2_ros import Buffer,TransformListener
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus


def cell(m,x,y):
    r=m.info.resolution
    ox=m.info.origin.position.x;oy=m.info.origin.position.y
    col=math.floor((x-ox)/r);row=math.floor((y-oy)/r)
    if not (0<=col<m.info.width and 0<=row<m.info.height):
        raise RuntimeError("robot left mapped simulator")
    return row*m.info.width+col

def distance_to_rect(x,y):
    dx=max(.67-x,0.,x-1.14)
    dy=max(-.55-y,0.,y-.55)
    return math.hypot(dx,dy)


def run():
    rclpy.init()
    node=Node("valdi_filtered_motion_probe",parameter_overrides=[
        Parameter("use_sim_time",value=True)])
    q=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    mask=[None]
    node.create_subscription(OccupancyGrid,"/valdi/keepout_filter_mask",
                             lambda m:mask.__setitem__(0,m),q)
    tf=Buffer();_listener=TransformListener(tf,node,spin_thread=False)
    client=ActionClient(node,NavigateToPose,"/navigate_to_pose")
    trace=[]
    terminal=None
    err=None
    accepted=False
    sent=False
    cancel_called=False
    handle=None

    def pose_now():
        try:
            t=tf.lookup_transform("map","base_link",Time())
            return (t.transform.translation.x,t.transform.translation.y)
        except Exception:
            return None

    def sample():
        xy=pose_now()
        if xy is not None:
            x,y=xy
            if mask[0] is not None:
                if mask[0].data[cell(mask[0],x,y)]==100:
                    raise RuntimeError("simulated robot centre entered forbidden mask")
            if not trace or math.hypot(x-trace[-1][0],y-trace[-1][1])>.008:
                trace.append((x,y,time.monotonic()))
        return xy

    def until(predicate,timeout):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.050)
            sample()
            if predicate():return True
        return False

    try:
        if not until(lambda:mask[0] is not None
                     and client.server_is_ready()
                     and pose_now() is not None,26):
            raise RuntimeError("Nav2, TF, or mask not ready for physical-sim movement")
        x0,y0=sample()
        assert abs(x0)<.5 and abs(y0)<.5,(x0,y0)
        goal=NavigateToPose.Goal()
        goal.pose=PoseStamped()
        goal.pose.header.frame_id="map"
        goal.pose.header.stamp=node.get_clock().now().to_msg()
        goal.pose.pose.position.x=1.85
        goal.pose.pose.position.y=0.
        goal.pose.pose.orientation.w=1.
        submitted=client.send_goal_async(goal)
        sent=True
        if not until(submitted.done,15):
            raise RuntimeError("Nav2 didn't answer navigation goal")
        handle=submitted.result()
        accepted=bool(handle.accepted)
        if not accepted:raise RuntimeError("Nav2 rejected test navigation")
        result_fut=handle.get_result_async()
        completed=until(result_fut.done,65)
        if not completed:
            raise RuntimeError("Nav2 movement exceeded bounded test window")
        outcome=result_fut.result()
        terminal=outcome.status
        if terminal!=GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError(f"Nav2 returned terminal status {terminal}, not succeeded")
        # Settle measurement without streaming more motor commands.
        until(lambda:True,1)
        final=pose_now()
        if final is None:raise RuntimeError("lost simulator pose")
        displacement=math.hypot(final[0]-x0,final[1]-y0)
        min_dist=min(distance_to_rect(x,y) for x,y,_ in trace)
        if displacement<.7:raise RuntimeError("robot didn't move substantially")
        report={"ok":True,"accepted":accepted,"goal_status":terminal,
                "start_map_pose":[round(x0,3),round(y0,3)],
                "final_map_pose":[round(final[0],3),round(final[1],3)],
                "travel_euclidean_m":round(displacement,3),
                "sampled_pose_count":len(trace),
                "min_center_to_keepout_boundary_m":round(min_dist,4),
                "forbidden_mask_center_entries":0,
                "world":"actual OOMWOO Gazebo test_room",
                "navigation":"actual Nav2 NavigateToPose with both KeepoutFilters",
                "localization":"Nav2 AMCL active with known test-room initial pose; not physical-hardware SLAM"}
        print("REAL_KEEP_OUT_MOTION "+json.dumps(report,sort_keys=True),flush=True)
    except Exception as e:
        err=str(e)
        if handle is not None and accepted:
            # No physical hardware. Still cancel the owned simulated goal
            # if we hit the test timeout or any mask crossing.
            try:
                fut=handle.cancel_goal_async()
                cancel_called=True
                until(fut.done,8)
            except Exception:
                pass
        report={"ok":False,"error":err,"goal_sent":sent,"accepted":accepted,
                "terminal_status":terminal,"sampled_positions":len(trace),
                "cancel_requested":cancel_called}
        print("REAL_KEEP_OUT_MOTION "+json.dumps(report,sort_keys=True),flush=True)
        raise
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=="__main__":
    run()
