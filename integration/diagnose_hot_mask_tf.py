"""Read-only TF diagnostics of Nav2 LOCAL odom rolling costmap vs MAP keepout."""
import json,time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSDurabilityPolicy
from tf2_ros import Buffer,TransformListener
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import OccupancyGrid
from oomwoo_cleaning_jobs.costmap_frame import rolling_costmap_in_map
from oomwoo_cleaning_jobs.map_admission import Raster


def main():
    rclpy.init()
    n=Node("valdi_keepout_tf_readonly")
    q=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    got={}
    buffer=Buffer()
    listener=TransformListener(buffer,n,spin_thread=False)
    n.create_subscription(Clock,"/clock",
       lambda m:got.__setitem__("clock",m.clock.sec+m.clock.nanosec*1e-9),10)
    n.create_subscription(OccupancyGrid,"/valdi/keepout_filter_mask",
       lambda m:got.__setitem__("mask",m),q)
    n.create_subscription(OccupancyGrid,"/global_costmap/costmap",
       lambda m:got.__setitem__("global",m),10)
    n.create_subscription(OccupancyGrid,"/local_costmap/costmap",
       lambda m:got.__setitem__("local",m),10)
    deadline=time.monotonic()+18
    while time.monotonic()<deadline:
        rclpy.spin_once(n,timeout_sec=.09)
    record={"got":list(got),"sim_clock":got.get("clock")}
    def samples(r):
        d={}
        for p,(x,y) in {"old":(.85,0.),"new":(-.8,0.),"control":(.15,0.)}.items():
            try:d[p]=int(r.data[r.cell_at(x,y)])
            except Exception as e:d[p]=str(e)[:120]
        return d
    for label in ("mask","global","local"):
        m=got.get(label)
        if m is None:continue
        rec={"frame":m.header.frame_id,"size":[m.info.width,m.info.height],
             "stamp":m.header.stamp.sec+m.header.stamp.nanosec*1e-9}
        try:
            if m.header.frame_id=="odom":
                tf=buffer.lookup_transform("map","odom",Time())
                rec["tf_stamp"]=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
                rec["tf_translation"]=[tf.transform.translation.x,tf.transform.translation.y]
                rr=rolling_costmap_in_map(m,map_from_odom=tf,
                                sim_now_s=got.get("clock",0.))
            else:
                rr=Raster.from_ros(m)
            rec["world_costs"]=samples(rr)
        except Exception as e:
            rec["error"]=type(e).__name__+": "+str(e)
        record[label]=rec
    print("OOMWOO_HOT_MASK_TF "+json.dumps(record,sort_keys=True),flush=True)
    n.destroy_node();rclpy.shutdown()

if __name__=="__main__":
    main()
