"""Read-only, source-exact OOMWOO coverage-meter cleanable-mask diagnostic.
No robot control; fetches /map, /coverage_meter/covered_grid, and computes the
mask with the installed official OOMWOO code. The remaining indices from the
private checkpoint are examined, not altered.
"""
import json
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSDurabilityPolicy,QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from oomwoo_sim_support.coverage_meter_node import _dilate,_flood_fill,_nearest_free

rclpy.init()
n=Node('valdi_readonly_meter_mask_inspector')
q=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
v={'map':None,'covered':None}
n.create_subscription(OccupancyGrid,'/map',lambda msg:v.__setitem__('map',msg),q)
n.create_subscription(OccupancyGrid,'/coverage_meter/covered_grid',lambda msg:v.__setitem__('covered',msg),q)
for _ in range(100):
    if v['map'] and v['covered']:break
    rclpy.spin_once(n,timeout_sec=.1)
if not v['map'] or not v['covered']:raise RuntimeError('map or coverage unavailable')
m=v['map'];w,h=m.info.width,m.info.height;res=m.info.resolution
free=np.asarray(m.data,dtype=np.int16).reshape(h,w)
free=(free>=0)&(free<50)
origin=m.info.origin.position
cx=int((0.0-origin.x)/res)
cy=int((0.0-origin.y)/res)
start=_nearest_free(free,cx,cy)
reach=_flood_fill(free,start)
robotrad=max(1,round(.1745/res))
cleanrad=max(1,round(.16/res))
em=max(0,round(.15/res))
drivable=reach & ~_dilate(~free,robotrad)
cleanable=_dilate(drivable,cleanrad) & reach
if em>0:cleanable=cleanable & ~_dilate(~free,em)
visited=np.asarray(v['covered'].data).reshape(h,w)
target=[8317,8457,8598,8316,8456,8457,8597,8598]
pts=[{"index":i,"row":i//w,"col":i%w,"free":bool(free[i//w,i%w]),"reach":bool(reach[i//w,i%w]),"drivable":bool(drivable[i//w,i%w]),"cleanable_by_meter":bool(cleanable[i//w,i%w]),"covered":int(visited[i//w,i%w])} for i in sorted(set(target))]
p=Path('/tmp/valdi_phase4_coverage_20261027.json')
if p.exists():
    cp=json.loads(p.read_text())
    remain=sorted(set(cp['targets'])-set(cp['cleaned']))
    points=[{"index":i,"free":bool(free[i//w,i%w]),"meter_cleanable":bool(cleanable[i//w,i%w]),"covered":int(visited[i//w,i%w])} for i in remain]
else:points=[]
print('METER_MASK_PROBE '+json.dumps({"width":w,"height":h,"resolution":res,"drivable_cells":int(drivable.sum()),"meter_cleanable_cells":int(cleanable.sum()),"examples":pts,"remaining_now":points},sort_keys=True),flush=True)
n.destroy_node();rclpy.shutdown()
