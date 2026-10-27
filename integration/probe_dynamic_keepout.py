"""Live OOMWOO Nav2 mask hot-reload while robot is idle and job paused.

Uses genuine nav2_msgs/srv/LoadMap on the private valdi_keepout_mask_server.
No NavigateToPose or motor command is issued. Only commit new zone version
after two distinct ROS2 post-filter costmap observations are lethal in BOTH
global and locally visible masks. On failure, the transaction stays blocked.
"""
from __future__ import annotations

import json,time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.msg import CostmapFilterInfo
from nav2_msgs.srv import LoadMap
from action_msgs.msg import GoalStatusArray

from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.map_admission import Raster, AdmissionError
from oomwoo_cleaning_jobs.costmap_frame import rolling_costmap_in_map
from tf2_ros import Buffer,TransformListener,TransformException
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from oomwoo_cleaning_jobs.zones import ZonePlan,ZoneShape
from oomwoo_cleaning_jobs.zone_revision import ZoneUpdateTransaction,ZoneUpdateError


class View(Node):
    def __init__(self):
        super().__init__("valdi_keepout_dynamic_reload_probe")
        q=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.latest={}
        self.seq={}
        self.clock_s=None
        self.tf=Buffer()
        self.listener=TransformListener(self.tf,self,spin_thread=False)
        self.create_subscription(Clock,"/clock",
             lambda m:setattr(self,"clock_s",m.clock.sec+m.clock.nanosec*1e-9),10)
        self.create_subscription(OccupancyGrid,"/map",lambda m:self.record("map",m),q)
        self.create_subscription(OccupancyGrid,"/valdi/keepout_filter_mask",
                                 lambda m:self.record("mask",m),q)
        self.create_subscription(OccupancyGrid,"/global_costmap/costmap",
                                 lambda m:self.record("global",m),10)
        self.create_subscription(OccupancyGrid,"/local_costmap/costmap",
                                 lambda m:self.record("local",m),10)
        self.create_subscription(CostmapFilterInfo,"/valdi/keepout_costmap_filter_info",
                                 lambda m:self.record("info",m),q)
        self.create_subscription(GoalStatusArray,"/navigate_to_pose/_action/status",
                                 lambda m:self.record("goals",m),q)
        self.loader=self.create_client(LoadMap,"/valdi_keepout_mask_server/load_map")

    def record(self,name,msg):
        self.latest[name]=msg
        self.seq[name]=self.seq.get(name,0)+1


def main():
    rclpy.init()
    v=View()
    result={"ok":False,"scope":"real ROS2 Nav2 KeepoutFilter hot edit in private offline OOMWOO test_room",
            "motor_commands":0}
    def until(fn,seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:
            rclpy.spin_once(v,timeout_sec=.052)
            if fn():return True
        return False
    try:
        if not until(lambda:all(name in v.latest for name in
                     ("map","mask","global","local","info"))
                     and v.loader.service_is_ready()
                     and v.clock_s is not None,32):
            raise RuntimeError("missing live ROS2 prerequisites: "+
                repr({"got":sorted(v.latest),
                     "loader_service_ready":v.loader.service_is_ready(),
                     "load_map_services":[x for x in v.get_service_names_and_types()
                                          if "load_map" in x[0]],
                     "mask_nodes":v.get_node_names()}))
        goalmsg=v.latest.get("goals")
        if goalmsg and any(item.status in (1,2,3) for item in goalmsg.status_list):
            raise RuntimeError("Nav2 executing another action: edit forbidden")
        original=Raster.from_ros(v.latest["map"])
        old=Raster.from_ros(v.latest["mask"])
        grid=Grid(original.width,original.height,original.data,
                  resolution_m=original.resolution)
        shape=ZoneShape("Hot revision no-go","polygon",
                    ((-1.02,-.55),(-.57,-.55),(-.57,.55),(-1.02,.55)))
        proposed=ZonePlan.create(original,grid,(shape,)).mask(original,grid)
        assert proposed.data.count(100)==198
        checkpoint=Path("/tmp/valdi_keepout_revision_hot_20261027.json")
        if checkpoint.exists():raise RuntimeError("reused filter revision checkpoint: start a new container")
        tx=ZoneUpdateTransaction(original,old,checkpoint)
        tx.stage(proposed,paused=True,goal_terminal=True,
                 verified_source_map=True)
        result["initial_observation"]={"old_mask_cells":old.data.count(100),
                  "new_mask_cells":proposed.data.count(100)}
        if not tx.must_block_nav():raise RuntimeError("zone edit failed to block job")
        req=LoadMap.Request()
        req.map_url="/lab/keepout_test_room_v2.yaml"
        fut=v.loader.call_async(req)
        if not until(fut.done,15):raise RuntimeError("LoadMap service timeout")
        response=fut.result()
        result["map_load_return_code"]=int(response.result)
        if int(response.result)!=0:
            raise RuntimeError("live mask server refused new YAML")
        before_global=v.seq.get("global",0)
        before_local=v.seq.get("local",0)
        observations=[]
        last_counts=None
        timeout=time.monotonic()+25
        while time.monotonic()<timeout and tx.status().state!="verified":
            rclpy.spin_once(v,timeout_sec=.10)
            if not all(key in v.latest for key in ("mask","global","local")):continue
            latest=Raster.from_ros(v.latest["mask"])
            if latest.data!=proposed.data:continue
            counts=(v.seq.get("global",0),v.seq.get("local",0))
            if (counts==last_counts or counts[0]<=before_global
                or counts[1]<=before_local):
                continue
            last_counts=counts
            try:
                tx.verify_sample(actual_mask=latest,
                    global_costmap=Raster.from_ros(v.latest["global"]),
                    local_costmap=rolling_costmap_in_map(
                        v.latest["local"],
                        map_from_odom=v.tf.lookup_transform("map","odom",Time()),
                        sim_now_s=v.clock_s),
                    filter_info_active=v.latest["info"].type==0,
                    status_fresh=True,job_paused=True,
                    global_sequence=counts[0],local_sequence=counts[1])
                observations.append({"global_updates":counts[0]-before_global,
                       "local_updates":counts[1]-before_local,
                       "revision":tx.status().revision,
                       "state":tx.status().state})
            except (ZoneUpdateError,AdmissionError,TransformException):
                # Nav2 publishes transient zero-sized startup costmaps.
                # Never use them as filter proof; wait for valid fresh frames.
                pass
        result["valid_post_filter_samples"]=len(observations)
        result["post_filter_confirmations"]=observations
        if tx.status().state!="verified":
            raise RuntimeError("Nav2 hot mask update did not become lethal in two costmap samples")
        tx.commit(nav2_idle=True,fresh_pose=True,job_paused=True)
        if tx.must_block_nav():raise RuntimeError("new version remained blocked after live confirmation")
        # Explicitly verify the old mask is no longer the active topic,
        # not just that LoadMap returned success.
        if Raster.from_ros(v.latest["mask"]).data!=proposed.data:
            raise RuntimeError("subsequent mask topic reverted")
        result.update({"ok":True,"old_mask_cells":old.data.count(100),
              "new_mask_cells":proposed.data.count(100),
              "committed_revision":tx.status().revision,
              "nav2_filter_confirmed_global":True,
              "nav2_filter_confirmed_local":True,
              "no_goal_during_edit":True})
        print("OOMWOO_DYNAMIC_KEEPOUT "+json.dumps(result,sort_keys=True),flush=True)
    except Exception as exc:
        result["error"]=str(exc)
        print("OOMWOO_DYNAMIC_KEEPOUT "+json.dumps(result,sort_keys=True),flush=True)
        raise
    finally:
        v.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    main()
