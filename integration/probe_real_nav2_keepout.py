"""Real Nav2 costmaps + ComputePathToPose keepout verification, no motor commands."""
from __future__ import annotations
import json,math,time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSDurabilityPolicy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import CostmapFilterInfo
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock
from oomwoo_cleaning_jobs.map_admission import Raster,CostmapChecker


def cell(msg,x,y):
    p=msg.info.origin.position;q=msg.info.origin.orientation
    yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
    dx,dy=x-p.x,y-p.y;c,s=math.cos(yaw),math.sin(yaw)
    col=math.floor((c*dx+s*dy)/msg.info.resolution)
    row=math.floor((-s*dx+c*dy)/msg.info.resolution)
    if 0<=col<msg.info.width and 0<=row<msg.info.height:
        return row*msg.info.width+col
    raise RuntimeError("point outside costmap")

class Observer(Node):
    def __init__(self):
        super().__init__("valdi_keepout_lab_probe")
        q=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.mask=self.info=self.global_grid=self.local_grid=self.clock=self.base_map=None
        self.create_subscription(OccupancyGrid,"/map",
                                 lambda m:setattr(self,"base_map",m),q)
        self.create_subscription(OccupancyGrid,"/valdi/keepout_filter_mask",
                                 lambda m:setattr(self,"mask",m),q)
        self.create_subscription(CostmapFilterInfo,"/valdi/keepout_costmap_filter_info",
                                 lambda m:setattr(self,"info",m),q)
        self.create_subscription(OccupancyGrid,"/global_costmap/costmap",
                                 lambda m:setattr(self,"global_grid",m),10)
        self.create_subscription(OccupancyGrid,"/local_costmap/costmap",
                                 lambda m:setattr(self,"local_grid",m),10)
        self.create_subscription(Clock,"/clock",
                                 lambda m:setattr(self,"clock",m.clock.sec+m.clock.nanosec*1e-9),10)
        self.planner=ActionClient(self,ComputePathToPose,"/compute_path_to_pose")

    def pose(self,x,y):
        p=PoseStamped()
        p.header.frame_id="map"
        p.header.stamp=self.get_clock().now().to_msg()
        p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.
        return p


def run():
    rclpy.init()
    node=Observer()
    result={"ok":False,"scope":"isolated ROS2 costmap and planner; no motion"}
    def until(test,sec=30):
        end=time.monotonic()+sec
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.065)
            if test():return True
        return False
    try:
        ok=until(lambda:all(getattr(node,a) is not None for a in
                           ("mask","info","global_grid","local_grid","clock","base_map"))
                 and node.planner.server_is_ready(),50)
        result["sources"]={a:getattr(node,a) is not None for a in
                           ("mask","info","global_grid","local_grid","clock")}
        result["planner_server_ready"]=node.planner.server_is_ready()
        if not ok:raise RuntimeError("missing ROS2 topic or planner")
        assert node.info.type==0
        assert node.info.filter_mask_topic=="/valdi/keepout_filter_mask"
        result["mask_occupied"]=sum(v==100 for v in node.mask.data)
        assert result["mask_occupied"]==220
        result["mask_at_keepout"]=node.mask.data[cell(node.mask,.85,0.)]
        assert result["mask_at_keepout"]==100
        def values(map):
            return {"keepout":int(map.data[cell(map,.85,0.)]),
                    "control":int(map.data[cell(map,.20,0.)])}
        result["global_costs"]=values(node.global_grid)
        result["local_costs"]=values(node.local_grid)
        if result["global_costs"]["keepout"]<95:
            raise RuntimeError("global costmap filter not active")
        if result["local_costs"]["keepout"]<95:
            raise RuntimeError("local costmap filter not active")
        def compute(x,y):
            req=ComputePathToPose.Goal()
            req.start=node.pose(0.,0.)
            req.goal=node.pose(x,y)
            req.use_start=True
            sent=node.planner.send_goal_async(req)
            if not until(sent.done,12):raise RuntimeError("planner goal acceptance timeout")
            handle=sent.result()
            if not handle.accepted:raise RuntimeError("planner rejected action handle")
            finished=handle.get_result_async()
            if not until(finished.done,25):raise RuntimeError("planner result timeout")
            return finished.result().result

        around=compute(1.85,0.)
        result["around_error"]=int(around.error_code)
        result["around_error_msg"]=str(around.error_msg)
        if around.error_code:raise RuntimeError("failed plan around: "+str(around.error_msg))
        poses=around.path.poses
        result["path_poses"]=len(poses)
        assert len(poses)>10
        hits=0
        for a,b in zip(poses,poses[1:]):
            x0,y0=a.pose.position.x,a.pose.position.y
            x1,y1=b.pose.position.x,b.pose.position.y
            samples=max(1,math.ceil(math.hypot(x1-x0,y1-y0)/.025))
            for k in range(samples+1):
                r=k/samples;x=x0+r*(x1-x0);y=y0+r*(y1-y0)
                if node.mask.data[cell(node.mask,x,y)]==100:hits+=1
        result["path_keepout_crossings"]=hits
        assert hits==0,"computed plan cuts through banned mask"
        forbidden=compute(.85,0.)
        result["goal_inside_error"]=int(forbidden.error_code)
        result["goal_inside_path_poses"]=len(forbidden.path.poses)
        if forbidden.path.poses:
            last=forbidden.path.poses[-1].pose.position
            result["goal_inside_last_pose"]=[round(last.x,4),round(last.y,4)]
            result["goal_inside_end_offset_m"]=round(math.hypot(last.x-.85,last.y),4)
            inside_hits=sum(1 for p in forbidden.path.poses
                            if node.mask.data[cell(node.mask,
                                p.pose.position.x,p.pose.position.y)]==100)
            result["inside_goal_path_mask_hits"]=inside_hits
            assert inside_hits==0,"Nav2 computed path inside prohibited mask"
        static_map=Raster.from_ros(node.base_map)
        dynamic=Raster.from_ros(node.global_grid)
        checker=CostmapChecker(static_map,dynamic,footprint_radius_m=.21)
        target_index=static_map.cell_at(.85,0.0)
        result["independent_controller_admission_to_forbidden_goal"]=checker.route_admitted(
            (target_index,))
        assert result["independent_controller_admission_to_forbidden_goal"] is False,            "controller allowed destination inside forbidden area"
        result["conclusion"]="Both Nav2 costmaps lethal, around-path avoids mask; app gate rejects inside goal"

        result["ok"]=True
        print("REAL_NAV2_KEEPOUT "+json.dumps(result,sort_keys=True),flush=True)
    except Exception as e:
        result["error"]=str(e)
        print("REAL_NAV2_KEEPOUT "+json.dumps(result,sort_keys=True),flush=True)
        raise
    finally:
        node.destroy_node();rclpy.shutdown()

if __name__=="__main__":
    run()
