"""Create two DISJOINT named digital zones on the official Gazebo saved map.

These zones deliberately lie on the SAME open test_room floor and DO NOT
represent walled architectural rooms. Two distinct labels exercise actual
multi-room ordering, persisted selected scopes and external measured coverage.
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

DEST=Path("/tmp/oomwoo_two_virtual_rooms_20261027.json")
def main():
    if DEST.exists():raise RuntimeError("two-room plan exists; require fresh isolated world")
    rclpy.init()
    n=Node("oomwoo_two_digital_rooms_readonly")
    qos=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    messages=[]
    n.create_subscription(OccupancyGrid,"/map",lambda m:messages.append(m),qos)
    deadline=time.monotonic()+15
    while time.monotonic()<deadline and not messages:
        rclpy.spin_once(n,timeout_sec=.09)
    if not messages:raise RuntimeError("official OOMWOO map not available")
    map_=Raster.from_ros(messages[-1])
    grid=Grid(map_.width,map_.height,map_.data,
              resolution_m=map_.resolution,clearance_cells=5)
    a=tuple(sorted(i for i in grid.traversable
        if .78<=map_.centre(i)[0]<=1.32 and
           -.13<=map_.centre(i)[1]<=.25))
    b=tuple(sorted(i for i in grid.traversable
        if 1.53<=map_.centre(i)[0]<=1.79 and
           -.13<=map_.centre(i)[1]<=.25))
    if not 25<=len(a)<=100 or not 25<=len(b)<=100:
        raise RuntimeError(f"test geometry invalid A={len(a)} B={len(b)}")
    if set(a)&set(b):raise RuntimeError("room scopes intersect")
    other=tuple(sorted(grid.traversable-set(a)-set(b)))
    plan=RoomPlan(grid.fingerprint(),
              {"Zona_simulada_A":a,"Zona_simulada_B":b,"Resto_abierto":other},
              frozenset(),2,8)
    plan.validate_for(grid)
    source=Path("/workspace/valdi-cleaning-jobs/.local-sim/oomwoo_upstream_waypoints.json")
    converted=imported_offline_plan(plan.grid_with_rooms(grid),map_,source)
    in_a=sum(i in a for i in converted.selected)
    in_b=sum(i in b for i in converted.selected)
    if in_a<1 or in_b<1:raise RuntimeError("missing original OOMWOO plan points in target zones")
    plan.save(DEST,grid)
    report={"ok":True,"plan":str(DEST),"source":"actual OOMWOO test_room map",
       "virtual_rooms":["Zona_simulada_A","Zona_simulada_B"],
       "scope_A":len(a),"scope_B":len(b),"selected_total":len(a)+len(b),
       "upstream_waypoints_A":in_a,"upstream_waypoints_B":in_b,
       "other_safe_cells":len(other),
       "physical_walls_separate_rooms":False,
       "map_digest":grid.fingerprint()}
    print("OOMWOO_TWO_DIGITAL_ROOMS "+json.dumps(report,sort_keys=True),flush=True)
    n.destroy_node();rclpy.shutdown()
if __name__=="__main__":main()
