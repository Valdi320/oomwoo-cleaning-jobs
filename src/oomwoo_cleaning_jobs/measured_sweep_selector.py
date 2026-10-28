"""Pick the next OOMWOO upstream sweep waypoint using REAL measured coverage.

The upstream 409-pose boustrophedon plan is an ordered geometry source;
its original executor is NOT spun. This selector skips a waypoint ONLY
if independent measured covered-grid cells show the surrounding sweep
disk was sufficiently covered. Nav2 success alone NEVER authorizes skip.

This module does not issue goals, persist fake clean credit or override
actual Nav2 keepout/costmap gate.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from .grid import Grid
from .map_admission import Raster,AdmissionError
from .upstream_sweep_adapter import (
    SafeSweepPlan, SweepAdaptationError, raster_geometry_digest,
)


@dataclass(frozen=True)
class SweepChoice:
    index: int
    target_cell: int
    measured_dirty_fraction: float
    skipped_fully_covered: int
    skipped_outside_scope: int


class MeasuredSweepSelector:
    """Stateless choice; recovery can rescan using fresh real meter evidence."""

    def __init__(
        self, grid:Grid, raster:Raster, plan:SafeSweepPlan,
        *,
        swath_radius_m:float=.16,
        skip_when_covered_fraction:float=.80,
    ):
        if (not isinstance(plan,SafeSweepPlan)
            or plan.map_digest!=grid.fingerprint()
            or plan.raster_geometry_digest!=raster_geometry_digest(raster)):
            raise SweepAdaptationError("sweep plan does not match source map geometry")
        if (raster.width!=grid.width or raster.height!=grid.height
            or raster.data!=grid.occupancy or abs(raster.resolution-grid.resolution_m)>1e-6):
            raise SweepAdaptationError("plan raster and grid diverge")
        if (type(swath_radius_m) not in (int,float)
            or not math.isfinite(swath_radius_m) or not .04<=swath_radius_m<=.6):
            raise SweepAdaptationError("invalid simulated cleaning radius")
        if (type(skip_when_covered_fraction) not in (int,float)
            or not math.isfinite(skip_when_covered_fraction)
            or not .5<=skip_when_covered_fraction<=1.):
            raise SweepAdaptationError("invalid measured skip threshold")
        self.grid=grid
        self.raster=raster
        self.plan=plan
        self.radius=float(swath_radius_m)
        self.threshold=float(skip_when_covered_fraction)

    def _coverage_fraction(self, cell:int, measured:Raster,
                           scope:frozenset[int],traversable:frozenset[int])->float:
        x,y=self.raster.centre(cell)
        cr=int(math.ceil(self.radius/self.raster.resolution))
        col,row=cell%self.grid.width,cell//self.grid.width
        mask=[]
        for yy in range(max(0,row-cr),min(self.grid.height,row+cr+1)):
            for xx in range(max(0,col-cr),min(self.grid.width,col+cr+1)):
                i=yy*self.grid.width+xx
                if i not in scope or i not in traversable:continue
                wx,wy=self.raster.centre(i)
                if math.hypot(wx-x,wy-y)<=self.radius:
                    mask.append(i)
        if not mask:return 1.0
        return sum(measured.data[i]==100 for i in mask)/len(mask)

    def select(
        self,measured:Raster, *,
        scope:frozenset[int],
        excluded:frozenset[int]=frozenset(),
        start_index:int=0,
    )->SweepChoice|None:
        """Return the earliest safe in-scope waypoint with measured dirty disk.

        Returns None only when all admitted waypoints are marked covered
        or excluded. This is NOT proof all target cells/room floor cleaned,
        especially if SafeSweepPlan contains rejected original points.
        """
        r=self.raster
        if (measured.frame!="map"
            or (measured.width,measured.height)!=(r.width,r.height)
            or abs(measured.resolution-r.resolution)>1e-6
            or abs(measured.origin_x-r.origin_x)>1e-6
            or abs(measured.origin_y-r.origin_y)>1e-6
            or abs(measured.yaw-r.yaw)>1e-7):
            raise SweepAdaptationError("external measurement mismatches planned map frame")
        if any(v not in (0,100) for v in measured.data):
            raise SweepAdaptationError("covered-grid not exact binary 0/100")
        traversable=self.grid.traversable
        if (not isinstance(scope,frozenset) or not scope
            or not scope<=traversable
            or not isinstance(excluded,frozenset)
            or not excluded<=traversable
            or type(start_index) is not int
            or not 0<=start_index<=len(self.plan.selected)):
            raise SweepAdaptationError("invalid scope/exclusion/cursor")
        covered_skip=0;other_skip=0
        for k in range(start_index,len(self.plan.selected)):
            cell=self.plan.selected[k]
            if cell not in scope or cell in excluded:
                other_skip+=1
                continue
            portion=self._coverage_fraction(cell,measured,scope,traversable)
            if portion>=self.threshold:
                covered_skip+=1
                continue
            return SweepChoice(k,cell,round(1.0-portion,4),
                               covered_skip,other_skip)
        return None
