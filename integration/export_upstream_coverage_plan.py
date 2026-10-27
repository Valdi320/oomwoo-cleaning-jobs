"""Read-only geometry export from the actual OOMWOO boustrophedon planner.

Creates the OFFICIAL CoveragePlanner class but NEVER spins its executor,
starts no action server and publishes NO navigation commands. A separate
read-only node receives /map; after that we call the upstream pure
planning method directly and export only waypoints + map metadata.
"""
from __future__ import annotations
import hashlib
import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,QoSReliabilityPolicy,QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from oomwoo_coverage.coverage_planner_node import CoveragePlanner
from oomwoo_cleaning_jobs import Grid


def run():
    rclpy.init()
    reader=Node("valdi_readonly_coverage_map")
    val={"map":None}
    qos=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                   durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    reader.create_subscription(OccupancyGrid,"/map",
        lambda m:val.__setitem__("map",m),qos)
    deadline=time.monotonic()+12
    while time.monotonic()<deadline and val["map"] is None:
        rclpy.spin_once(reader,timeout_sec=.1)
    if val["map"] is None:raise RuntimeError("OOMWOO official map not ready")
    m=val["map"]
    controller=CoveragePlanner()
    try:
        controller._on_map(m)
        controller.robot_xy=(0.0,0.0)
        start=time.monotonic()
        poses=controller._plan_waypoints() # never calls send_goal_async
        elapsed=time.monotonic()-start
        if not poses:raise RuntimeError("no sweep waypoints")
        g=Grid(m.info.width,m.info.height,tuple(m.data),
               resolution_m=float(m.info.resolution),clearance_cells=4)
        def index(x,y):
            col=math.floor((x-m.info.origin.position.x)/m.info.resolution)
            row=math.floor((y-m.info.origin.position.y)/m.info.resolution)
            if not (0<=col<g.width and 0<=row<g.height):
                raise RuntimeError("planner waypoint outside map")
            return row*g.width+col
        cells=[index(p.pose.position.x,p.pose.position.y) for p in poses]
        invalid=[i for i in cells if i not in g.traversable]
        # Conservative Grid clearance may reject narrow places the
        # upstream planner admits; report, don't misrepresent it as safe.
        result={"ok":True,"plan_producer":"unmodified OOMWOO CoveragePlanner._plan_waypoints",
          "never_spun_planner":True,"no_nav2_goal_sent":True,
          "source_map_size":[g.width,g.height],
          "source_map_digest":g.fingerprint(),
          "resolution":g.resolution_m,
          "planner_waypoints":len(poses),
          "unique_waypoint_cells":len(set(cells)),
          "conservative_clearance_rejections":len(invalid),
          "planning_elapsed_s":round(elapsed,3),
          "target_poses":[[round(p.pose.position.x,4),round(p.pose.position.y,4)]
                           for p in poses],
          "target_cells":cells,
          "frame":"map",
          "planning_mode":"upstream unmodified offline geometry only",
          "hardware":False}
        print("OOMWOO_UPSTREAM_PLAN "+json.dumps(result,sort_keys=True),flush=True)
    finally:
        controller.destroy_node()
        reader.destroy_node()
        rclpy.shutdown()


if __name__=="__main__":
    run()
