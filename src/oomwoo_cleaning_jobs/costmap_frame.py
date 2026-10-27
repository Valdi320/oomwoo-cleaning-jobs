"""Convert a rolling Nav2 costmap in odom to *real map coordinates*.

The map and local Nav2 costmaps are often in DIFFERENT frames.
Comparing a map-frame virtual wall with odom-frame OccupancyGrid pixels
without a TF transform can falsely declare a wall safe. Fail closed.

Pure Python / duck-typed ROS2 messages. Caller must supply a fresh
geometry_msgs/TransformStamped from tf2 lookup_transform('map','odom')
and the CURRENT simulation clock. No static fake map->odom shortcut.
"""
from __future__ import annotations
import math

from .map_admission import Raster,AdmissionError


class CostmapFrameError(AdmissionError):
    pass


def _yaw(q):
    values=(q.x,q.y,q.z,q.w)
    if any(not isinstance(v,(int,float)) or not math.isfinite(v)
           for v in values):
        raise CostmapFrameError("TF contains nonfinite quaternion")
    norm=math.sqrt(sum(v*v for v in values))
    if not .995<=norm<=1.005 or abs(q.x)>.03 or abs(q.y)>.03:
        raise CostmapFrameError("non-unit or non-planar TF rotation")
    return math.atan2(2*(q.w*q.z+q.x*q.y),
                      1-2*(q.y*q.y+q.z*q.z))


def _stamp_s(stamp):
    if (type(stamp.sec) is not int or type(stamp.nanosec) is not int
        or not 0<=stamp.nanosec<1_000_000_000):
        raise CostmapFrameError("invalid ROS2 timestamp")
    return stamp.sec+stamp.nanosec*1e-9


def rolling_costmap_in_map(
    grid, *,
    map_from_odom=None,
    sim_now_s:float,
    max_age_s:float=3.0,
    max_future_lead_s:float=6.0,
) -> Raster:
    """Transform OccupancyGrid's origin/yaw into the map frame.

    The map/odom transform is interpreted as translation+rotation of odom
    coordinates INTO map coordinates. This preserves local cell values
    and dimensions; only the Raster frame/origin changes.

    A stale, reversed or missing transform is NOT an admissible costmap.
    """
    if (type(sim_now_s) not in (float,int) or not math.isfinite(sim_now_s)
        or type(max_age_s) not in (float,int) or not .2<=max_age_s<=10.
        or type(max_future_lead_s) not in (float,int)
        or not 0.<=max_future_lead_s<=8.):
        raise CostmapFrameError("invalid reference clock/freshness threshold")
    msg_time=_stamp_s(grid.header.stamp)
    if abs(float(sim_now_s)-msg_time)>max_age_s:
        raise CostmapFrameError("local Nav2 costmap stale relative to ROS clock")
    if grid.header.frame_id=="map":
        try:return Raster.from_ros(grid)
        except AdmissionError as ex:
            raise CostmapFrameError("invalid map-frame Nav2 costmap") from ex
    if grid.header.frame_id!="odom":
        raise CostmapFrameError("unsupported local costmap frame")
    t=map_from_odom
    if (t is None or t.header.frame_id!="map"
        or t.child_frame_id!="odom"):
        raise CostmapFrameError("expected live map<-odom TF")
    tf_time=_stamp_s(t.header.stamp)
    # Nav2 AMCL may publish a future-dated TF under its configured
    # transform_tolerance (~5 s in OOMWOO test_room). Reject unbounded
    # future leads and transforms too old for current costmap admission.
    if (float(sim_now_s)-tf_time>max_age_s or
        tf_time-float(sim_now_s)>max_future_lead_s):
        raise CostmapFrameError("map<-odom TF stale/future")
    a=_yaw(t.transform.rotation)
    grid_yaw=_yaw(grid.info.origin.orientation)
    dx,dy=grid.info.origin.position.x,grid.info.origin.position.y
    cos,sin=math.cos(a),math.sin(a)
    tx=t.transform.translation.x
    ty=t.transform.translation.y
    if any(not math.isfinite(v) for v in (dx,dy,tx,ty)):
        raise CostmapFrameError("invalid map/odom offset")
    ox=tx+cos*dx-sin*dy
    oy=ty+sin*dx+cos*dy
    try:
        return Raster(grid.info.width,grid.info.height,grid.info.resolution,
               ox,oy,a+grid_yaw,tuple(grid.data),"map")
    except AdmissionError as ex:
        raise CostmapFrameError("converted costmap invalid") from ex
