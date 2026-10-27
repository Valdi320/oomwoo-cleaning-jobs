"""Conservative time admission for OOMWOO's simulation localization TF.

AMCL may publish future-dated TF under its configured transform
tolerance (~5s in the official test-room bringup). A forward stamp
within a strictly bounded lead is not stale. Old or far-future poses
still block navigation and any new Nav2 action dispatch.
"""
from __future__ import annotations
import math

class PoseTimeError(ValueError):
    pass

def require_fresh_amcl_pose(sim_time:float,stamp_s:float,*,
                            max_past_s:float=3.0,
                            max_future_s:float=6.0):
    args=(sim_time,stamp_s,max_past_s,max_future_s)
    if any(type(v) not in (float,int) or not math.isfinite(v) for v in args):
        raise PoseTimeError("nonfinite pose or simulation clock")
    if not .2<=max_past_s<=10 or not 0<=max_future_s<=8:
        raise PoseTimeError("invalid localization freshness policy")
    delta=sim_time-stamp_s
    if delta>max_past_s or delta< -max_future_s:
        raise PoseTimeError("map-to-robot TF outside bounded AMCL age")
    return delta
