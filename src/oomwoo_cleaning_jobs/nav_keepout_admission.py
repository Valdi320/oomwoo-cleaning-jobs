"""Fail-closed independent application gate for a REAL Nav2 keepout mask.

Publishing a virtual-wall OccupancyGrid is not enough. The real
global costmap must already mark all filter-mask prohibited cells as
lethal before the job controller can send a goal. Nav2's local-costmap
KeepoutFilter remains independently required and is verified by the
isolated Jazzy/Gazebo acceptance test.

This is an additional software interlock, not a physical safety device.
"""
from __future__ import annotations
from dataclasses import dataclass

from .map_admission import Raster, AdmissionError


class KeepoutNotEnforced(AdmissionError):
    """Requested no-go area is not proven lethal in active global costmap."""


@dataclass(frozen=True)
class VerifiedGlobalKeepout:
    base: Raster
    mask: Raster
    global_costmap: Raster
    min_lethal_cost: int = 95
    max_covered_cells: int = 30000

    def __post_init__(self):
        if not (isinstance(self.base,Raster) and isinstance(self.mask,Raster)
                and isinstance(self.global_costmap,Raster)):
            raise KeepoutNotEnforced("missing real map, mask or global costmap")
        m=self.mask;b=self.base;c=self.global_costmap
        if (m.frame != "map" or b.frame != "map" or c.frame != "map"
            or (m.width,m.height)!=(b.width,b.height)
            or abs(m.resolution-b.resolution)>1e-6
            or abs(m.origin_x-b.origin_x)>1e-6
            or abs(m.origin_y-b.origin_y)>1e-6
            or abs(m.yaw-b.yaw)>1e-7):
            raise KeepoutNotEnforced("mask geometry does not match active base map")
        if any(v not in (0,100) for v in m.data):
            raise KeepoutNotEnforced("mask must be exact binary 0/100 values")
        if type(self.min_lethal_cost) is not int or not 85<=self.min_lethal_cost<=100:
            raise KeepoutNotEnforced("invalid lethal cost threshold")
        if type(self.max_covered_cells) is not int or not 1<=self.max_covered_cells<=500000:
            raise KeepoutNotEnforced("invalid mask cell budget")
        forbidden=[i for i,cost in enumerate(m.data) if cost==100]
        if not forbidden or len(forbidden)>self.max_covered_cells:
            raise KeepoutNotEnforced("empty / oversized keepout mask")
        # Audit actual POST-FILTER costmap, not just filter_info,
        # static map or ROS topic discovery. Missing mask => refuse.
        for i in forbidden:
            x,y=m.centre(i)
            try:
                index=c.cell_at(x,y)
            except AdmissionError as exc:
                raise KeepoutNotEnforced("global costmap does not cover a mask cell") from exc
            if c.data[index]<self.min_lethal_cost:
                raise KeepoutNotEnforced("at least one prohibited cell is not lethal")
        object.__setattr__(self,"_forbidden",frozenset(forbidden))

    def accept_target(self,cell: int) -> bool:
        """Reject even when Nav2 would return a successful TRUNCATED plan."""
        return type(cell) is int and cell not in self._forbidden and 0<=cell<len(self.mask.data)

    def accept_route(self,route: tuple[int,...]) -> bool:
        """Fail if advisory path contains any forbidden mask-centre cell."""
        return type(route) is tuple and bool(route) and all(self.accept_target(i) for i in route)

    @property
    def forbidden_count(self) -> int:
        return len(self._forbidden)
