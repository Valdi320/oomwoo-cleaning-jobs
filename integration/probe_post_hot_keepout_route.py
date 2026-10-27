"""Read-only REAL Nav2 ComputePathToPose after KeepoutFilter hot-reload.

New exclusion rectangle at map x=-1.02..-.57 y=-.55..+.55.
Never send motion commands: ask planner for one detour across the *new*
forbidden region, audit dense path positions against live mask. Also
verify the previously forbidden region x=.85,y=0 is now free.
"""
from __future__ import annotations
import math,json,time
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped
from oomwoo_cleaning_jobs.map_admission import Raster

def main():
    rclpy.init()
    n=Node("valdi_hot_keepout_planner_only")
    qos=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    msgs={}
    n.create_subscription(OccupancyGrid,"/valdi/keepout_filter_mask",
                          lambda m:msgs.__setitem__("mask",m),qos)
    n.create_subscription(OccupancyGrid,"/global_costmap/costmap",
                          lambda m:msgs.__setitem__("global",m),10)
    client=ActionClient(n,ComputePathToPose,"/compute_path_to_pose")
    def until(pred,seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.04)
            if pred():return True
        return False
    report={"ok":False,"motion_commands_sent":0,"mode":"ComputePathToPose only"}
    try:
        if not until(lambda:"mask" in msgs and "global" in msgs
                     and client.server_is_ready(),23):
            raise RuntimeError("planner or new mask unavailable")
        mask=Raster.from_ros(msgs["mask"])
        global_grid=Raster.from_ros(msgs["global"])
        assert sum(i==100 for i in mask.data)==198
        def cost(x,y):
            return global_grid.data[global_grid.cell_at(x,y)]
        assert mask.data[mask.cell_at(-.8,0)]==100
        assert cost(-.8,0)>=95
        assert mask.data[mask.cell_at(.85,0)]==0
        assert cost(.85,0)<=25
        def pose(xx,yy):
            p=PoseStamped()
            p.header.frame_id="map"
            p.header.stamp=n.get_clock().now().to_msg()
            p.pose.position.x=xx;p.pose.position.y=yy
            p.pose.orientation.w=1.
            return p

        def compute(x,y):
            goal=ComputePathToPose.Goal()
            goal.start=pose(0.,0.)
            goal.goal=pose(x,y)
            goal.use_start=True
            sent=client.send_goal_async(goal)
            if not until(sent.done,9):raise RuntimeError("planner accept timeout")
            h=sent.result()
            if not h.accepted:raise RuntimeError("real planner refused action handle")
            done=h.get_result_async()
            if not until(done.done,18):raise RuntimeError("planner result timeout")
            out=done.result().result
            line={"goal":[x,y],"error_code":int(out.error_code),
                  "path_poses":len(out.path.poses)}
            if out.path.poses:
                pos=out.path.poses[-1].pose.position
                line["endpoint_error_m"]=round(math.hypot(pos.x-x,pos.y-y),3)
            else:
                line["endpoint_error_m"]=None
            hits=0
            for a,b in zip(out.path.poses,out.path.poses[1:]):
                x0,y0=a.pose.position.x,a.pose.position.y
                x1,y1=b.pose.position.x,b.pose.position.y
                steps=max(1,math.ceil(math.hypot(x1-x0,y1-y0)/.025))
                for k in range(steps+1):
                    t=k/steps
                    xq=x0+(x1-x0)*t;yq=y0+(y1-y0)*t
                    if mask.data[mask.cell_at(xq,yq)]==100:
                        hits+=1
            line["new_mask_intersections"]=hits
            return line

        # Prove the old forbidden rectangle has been removed from Nav2.
        formerly_blocked=compute(.85,0.)
        # Prove a goal inside the NEW rectangle is NOT reached just
        # because Nav2 may call its truncated path "succeeded".
        now_forbidden=compute(-.8,0.)
        report["outcomes"]=[formerly_blocked,now_forbidden]
        report["formerly_blocked_now_accessible"]=(
            formerly_blocked["error_code"]==0
            and formerly_blocked["path_poses"]>0
            and formerly_blocked["endpoint_error_m"]<=.20
            and formerly_blocked["new_mask_intersections"]==0)
        report["newly_forbidden_goal_not_reached"]=(
            now_forbidden["new_mask_intersections"]==0
            and (now_forbidden["error_code"]!=0
                 or now_forbidden["endpoint_error_m"] is None
                 or now_forbidden["endpoint_error_m"]>.20))
        report["ok"]=(report["formerly_blocked_now_accessible"]
                      and report["newly_forbidden_goal_not_reached"])

        report["actual_nav2_planner"]=True
        report["new_mask_cells"]=198
        report["new_mask_globally_lethal"]=True
        report["old_area_now_permitted"]=True
        print("OOMWOO_POST_HOT_ROUTE "+json.dumps(report,sort_keys=True),flush=True)
        if not report["ok"]:raise RuntimeError("new forbidden goal or freed old goal failed live planning test")
    except Exception as e:
        report["error"]=str(e)
        print("OOMWOO_POST_HOT_ROUTE "+json.dumps(report,sort_keys=True),flush=True)
        raise
    finally:
        n.destroy_node();rclpy.shutdown()

if __name__=="__main__":main()
