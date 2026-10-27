"""READ-ONLY OOMWOO Gazebo saved-map no-go zone/virtual-wall mask validation.

Outputs prototype candidate masks, never publishes Nav2 filter topics,
never changes the Nav2 configuration and never drives the robot.
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
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.zones import ZonePlan,ZoneShape
from oomwoo_cleaning_jobs.rooms import segment_rooms,RoomPlanError


def main():
    rclpy.init()
    node=Node('valdi_offline_zone_mask_probe')
    m=[None]
    qos=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid,'/map',lambda msg:m.__setitem__(0,msg),qos)
    t=time.monotonic()+12
    while time.monotonic()<t and m[0] is None:
        rclpy.spin_once(node,timeout_sec=.1)
    if m[0] is None:raise RuntimeError('no official map')
    raster=Raster.from_ros(m[0])
    grid=Grid(raster.width,raster.height,raster.data,
              resolution_m=raster.resolution,clearance_cells=5)
    # Conservative short virtual wall and polygon in test_room free-space.
    shapes=(ZoneShape('No-go zona central','polygon',
             ((.30,.30),(.70,.30),(.70,.70),(.30,.70))),
            ZoneShape('Pared virtual','wall',
             ((-.60,.10),(.60,.10)),width_m=.12))
    plan=ZonePlan.create(raster,grid,shapes)
    mask=plan.mask(raster,grid)
    safer=plan.grid_with_keepouts(raster,grid)
    assert safer.fingerprint()!=grid.fingerprint()
    assert safer.traversable<grid.traversable
    assert not set(mask.data)-{0,100}
    before=segment_rooms(grid)
    rejected_old=False
    try:before.grid_with_rooms(safer)
    except RoomPlanError:rejected_old=True
    assert rejected_old
    with tempfile.TemporaryDirectory(prefix='valdi-map-zones-') as d:
        path=Path(d)/'candidate_mask.json'
        plan.save(path,raster,grid)
        restored=ZonePlan.load(path,raster,grid)
        assert restored.mask(raster,grid)==mask
    result={
        "ok":True,
        "source":"official OOMWOO test_room /map; offline only",
        "map":[raster.width,raster.height,raster.resolution],
        "safe_before":len(grid.traversable),
        "safe_after":len(safer.traversable),
        "mask_100_cells":sum(v==100 for v in mask.data),
        "room_plan_stale_after_zone_edit_rejected":True,
        "labels_and_masks_roundtripped":True,
        "nav2_keepout_filter_tested":False,
        "contains_real_movement":False,
        "safety":"candidate masks only; NAV2 must independently load a KeepoutFilter before using them",
    }
    print('ROS_ZONE_MASK '+json.dumps(result,sort_keys=True),flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()
