"""Read-only reconciliation of the OOMWOO stock sweep against our safe grid.

Consumes exact waypoints exported from upstream CoveragePlanner while its
executor was NOT spun. Audits nearest safe endpoints, map fingerprint,
adjustment distances and rejects. No NavigateToPose is sent here.
"""
from __future__ import annotations
import json,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile,QoSDurabilityPolicy,QoSReliabilityPolicy
from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.upstream_sweep_adapter import adapt_upstream_plan,SweepAdaptationError


EXPORT=Path(__file__).resolve().parents[1]/".local-sim/oomwoo_upstream_waypoints.json"

def main():
    record=json.loads(EXPORT.read_text())
    assert record["no_nav2_goal_sent"] and record["never_spun_planner"]
    rclpy.init()
    node=Node("oomwoo_upstream_readonly_adapter")
    samples=[]
    q=QoSProfile(depth=1,reliability=QoSReliabilityPolicy.RELIABLE,
                 durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(OccupancyGrid,"/map",lambda m:samples.append(m),q)
    try:
        end=time.monotonic()+15
        while not samples and time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.10)
        if not samples:raise RuntimeError("no original map from OOMWOO")
        raster=Raster.from_ros(samples[-1])
        poses=tuple(tuple(x) for x in record["target_poses"])
        outcomes=[]
        for clearance in (4,5):
            grid=Grid(raster.width,raster.height,raster.data,
                      resolution_m=raster.resolution,clearance_cells=clearance)
            if clearance==4 and grid.fingerprint()!=record["source_map_digest"]:
                raise RuntimeError("upstream source map changed since export")
            for shift in (.18,.24,.35):
                t=time.monotonic()
                plan=adapt_upstream_plan(grid,raster,poses,max_shift_m=shift)
                outcomes.append({
                  "clearance_cells":clearance,
                  "shift_budget_m":shift,
                  "source_waypoints":plan.originally_requested,
                  "safe_unique_waypoints":len(plan.selected),
                  "shifted":plan.adjusted,
                  "max_adjustment_m":plan.maximum_adjustment_m,
                  "rejected":len(plan.rejected),
                  "planning_seconds":round(time.monotonic()-t,3),
                  "map_digest":plan.map_digest,
                  "plan_digest":plan.plan_digest,
                  "first_waypoints":list(plan.selected[:6]),
                })
        print("OOMWOO_SWEEP_COMPAT "+json.dumps({
            "ok":True,"source_upstream_waypoints":len(poses),
            "original_planner_does_not_execute":True,
            "actual_nav2_ownership":"none in this geometry-only test",
            "outcomes":outcomes},sort_keys=True),flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=="__main__":
    main()
