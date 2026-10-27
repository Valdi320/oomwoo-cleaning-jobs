"""Conservative adapter for OOMWOO's existing coverage_planner waypoint poses.

OOMWOO's upstream boustrophedon planner deliberately navigates near
furniture legs / walls. The cleaning-jobs interlock uses a MORE conservative
grid/costmap clearance, so upstream waypoints MUST be audited and adjusted
before any dispatch. Never remove the safety gate just to follow a plan.

The original planner must run only in a NON-EXECUTING geometry-export context,
not concurrently with this package's Nav2 action owner.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math

from .grid import Grid
from .map_admission import Raster, AdmissionError


class SweepAdaptationError(ValueError):
    """Upstream plan cannot be admitted under current safety policy."""


@dataclass(frozen=True)
class SafeSweepPlan:
    """Provenance-bound, conservative ordered candidate waypoints."""
    map_digest: str
    plan_digest: str
    selected: tuple[int,...]
    originally_requested: int
    adjusted: int
    maximum_adjustment_m: float
    rejected: tuple[int,...]   # unsafe waypoints, deliberately NOT accepted

    def require_complete(self):
        """Never mistake incomplete adaptation for full-room coverage."""
        if self.rejected:
            raise SweepAdaptationError(
                f"{len(self.rejected)} upstream waypoints cannot be safely adapted")
        return self

    def candidate_at(self,index:int) -> int:
        if type(index) is not int or not 0<=index<len(self.selected):
            raise SweepAdaptationError("waypoint index out of range")
        return self.selected[index]


def adapt_upstream_plan(
    grid:Grid,raster:Raster,
    poses:tuple[tuple[float,float],...],
    *,
    max_shift_m:float=.24,
    max_points:int=2000,
) -> SafeSweepPlan:
    """Snap upstream plan to nearest *conservatively traversable* centres.

    This is a planning adapter, NOT an obstacle-proof robot trajectory:
    before EVERY action a real Nav2 costmap and footprint admission is
    mandatory; Nav2 still computes the actual travel route. No cleaned
    cells are credited simply because the adapter accepted a waypoint.
    """
    if (not isinstance(grid,Grid) or not isinstance(raster,Raster)
        or raster.width!=grid.width or raster.height!=grid.height
        or raster.data!=grid.occupancy
        or abs(raster.resolution-grid.resolution_m)>1e-6):
        raise SweepAdaptationError("map/room/keepout raster disagreement")
    if (type(poses) is not tuple or not poses or len(poses)>max_points
        or type(max_points) is not int or max_points<1):
        raise SweepAdaptationError("invalid upstream waypoint list")
    if (type(max_shift_m) not in (float,int)
        or not math.isfinite(max_shift_m) or not .02<=max_shift_m<=.6):
        raise SweepAdaptationError("unreasonable waypoint shift allowance")
    reachable=grid.traversable
    if not reachable:
        raise SweepAdaptationError("no safe cells in current grid")

    plan_repr=[]
    selected=[]
    failed=[]
    adjusted=0
    max_adjust=0.0
    max_radius_cells=int(math.ceil(max_shift_m/raster.resolution))
    for idx,pose in enumerate(poses):
        if (type(pose) is not tuple or len(pose)!=2
            or any(type(x) not in (float,int) or not math.isfinite(x)
                   for x in pose)):
            raise SweepAdaptationError("invalid non-finite waypoint")
        x,y=pose
        if raster.frame!="map":
            raise SweepAdaptationError("upstream waypoints must be in map frame")
        try:
            candidate=raster.cell_at(x,y)
        except AdmissionError:
            failed.append(idx)
            plan_repr.append((float(x),float(y),None))
            continue
        col,row=candidate%grid.width,candidate//grid.width
        positions=[]
        for dy in range(-max_radius_cells,max_radius_cells+1):
            for dx in range(-max_radius_cells,max_radius_cells+1):
                cx=col+dx;cy=row+dy
                if not (0<=cx<grid.width and 0<=cy<grid.height):
                    continue
                other=cy*grid.width+cx
                if other not in reachable:
                    continue
                wx,wy=raster.centre(other)
                d=math.hypot(wx-x,wy-y)
                if d<=max_shift_m+1e-9:
                    positions.append((d,other))
        if not positions:
            failed.append(idx)
            plan_repr.append((float(x),float(y),None))
            continue
        distance,chosen=min(positions)
        if distance>raster.resolution*.75:
            adjusted+=1
        max_adjust=max(max_adjust,distance)
        if not selected or selected[-1]!=chosen:
            selected.append(chosen)
        plan_repr.append((round(x,5),round(y,5),chosen))
    digest=hashlib.sha256(json.dumps({
        "source_map":grid.fingerprint(),
        "source_waypoints":plan_repr,
        "max_shift_m":max_shift_m},sort_keys=True,separators=(",",":")).encode()).hexdigest()
    if not selected:
        raise SweepAdaptationError("not a single waypoint admitted")
    return SafeSweepPlan(
       map_digest=grid.fingerprint(),plan_digest=digest,
       selected=tuple(selected),originally_requested=len(poses),
       adjusted=adjusted,maximum_adjustment_m=round(max_adjust,5),
       rejected=tuple(failed))
