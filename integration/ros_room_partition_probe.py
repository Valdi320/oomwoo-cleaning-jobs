"""Read-only room segmentation against official OOMWOO Gazebo saved map.

This validates consistency/reproducibility of offline room proposals in
a real map frame; it NEVER sends Nav2 goals or modifies the map server.
"""
from __future__ import annotations
import json
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid

from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.rooms import segment_rooms, RoomPlan


def main():
    rclpy.init()
    node=Node('valdi_offline_room_partition_probe')
    state={"map":None}
    qos=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid,'/map',
         lambda msg:state.__setitem__('map',msg),qos)
    deadline=time.monotonic()+12
    while time.monotonic()<deadline and state["map"] is None:
        rclpy.spin_once(node,timeout_sec=.1)
    if state["map"] is None:raise RuntimeError('official /map unavailable')
    msg=state["map"]
    g=Grid(msg.info.width,msg.info.height,tuple(msg.data),
           resolution_m=float(msg.info.resolution),clearance_cells=5)
    out={"map_size":[g.width,g.height],
         "occupancy_cells":g.width*g.height,
         "conservative_safe_cells":len(g.traversable),
         "map_fingerprint":g.fingerprint(),
         "proposals":[]}
    for distance in (2,3,4):
        started=time.monotonic()
        plan=segment_rooms(g,bottleneck_distance_cells=distance,min_seed_cells=35)
        with tempfile.TemporaryDirectory(prefix='valdi-room-') as d:
            dest=Path(d)/'room_plan.json'
            plan.renamed(next(iter(plan.rooms)),'Zona principal').save(dest,g)
            revived=RoomPlan.load(dest,g)
            revived.validate_for(g)
        out["proposals"].append({
            "distance_threshold_cells":distance,
            "number_of_proposed_rooms":len(plan.rooms),
            "room_size_cells":sorted((len(v) for v in plan.rooms.values()),
                                     reverse=True)[:18],
            "partition_is_exact":sum(len(v) for v in plan.rooms.values())==len(g.traversable),
            "elapsed_s":round(time.monotonic()-started,3)})
    print("ROS_ROOM_PARTITION "+json.dumps(out,sort_keys=True),flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
