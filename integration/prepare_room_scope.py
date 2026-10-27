"""Prepare one persisted MAP-BASED digital room on OOMWOO's ACTUAL /map.

The official test_room has an open floor, NOT a real partition wall. The
named area here is explicitly an OPERATOR-DEFINED VIRTUAL ROOM (roughly
one square metre) selected within that open floor. It is useful for
verifying full measured scope completion, NOT physical room recognition.
"""
import json,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.rooms import RoomPlan
from oomwoo_cleaning_jobs.room_bridge import imported_offline_plan

def main():
    out=Path("/tmp/oomwoo_room_lab_20261027.json")
    if out.exists():raise RuntimeError("room lab already generated in private container")
    rclpy.init()
    n=Node("valdi_room_plan_offline")
    value=[]
    latched=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    n.create_subscription(OccupancyGrid,"/map",lambda m:value.append(m),latched)
    deadline=time.monotonic()+16
    while time.monotonic()<deadline and not value:
        rclpy.spin_once(n,timeout_sec=.1)
    if not value:raise RuntimeError("official map not ready")
    raster=Raster.from_ros(value[-1])
    base=Grid(raster.width,raster.height,raster.data,
              resolution_m=raster.resolution,clearance_cells=5)
    # This is a digital region, not architectural room walls.
    inside=tuple(sorted(i for i in base.traversable
        if .79<=raster.centre(i)[0]<=1.74 and
           -.12<=raster.centre(i)[1]<=.34))
    outside=tuple(sorted(base.traversable-frozenset(inside)))
    if not 50<=len(inside)<=260:raise RuntimeError("test area size not meaningful")
    plan=RoomPlan(base.fingerprint(),
             {"Zona_simulada_A":inside,"Resto_abierto":outside},
             frozenset(),2,8)
    plan.validate_for(base)
    named=plan.grid_with_rooms(base)
    source=Path("/workspace/valdi-cleaning-jobs/.local-sim/oomwoo_upstream_waypoints.json")
    adopted=imported_offline_plan(named,raster,source)
    source_inside=sum(1 for i in adopted.selected if i in inside)
    if source_inside<2:raise RuntimeError("original OOMWOO plan has no sweep points in selected digital room")
    plan.save(out,base)
    report={"ok":True,"location":"isolated real OOMWOO /map",
            "virtual_room":"Zona_simulada_A","room_scope_cells":len(inside),
            "other_safe_cells":len(outside),
            "map_digest":base.fingerprint(),
            "upstream_source_waypoints":adopted.originally_requested,
            "safe_adopted_endpoints_inside_room":source_inside,
            "note":"Operator-defined virtual room on an open official test_room floor, not walled architecture",
            "room_plan":str(out)}
    print("OOMWOO_ROOM_MAP_READY "+json.dumps(report,ensure_ascii=False,sort_keys=True),flush=True)
    n.destroy_node();rclpy.shutdown()

if __name__=="__main__":
    main()
