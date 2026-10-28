"""Map-coordinate conversion and fail-closed dynamic Nav2 costmap admission.

Pure Python, independent of ROS2 message implementations. The full robot
footprint, Nav2 collision monitor and MCU safety remain separate controls.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .swept_footprint import swept_disk_clear


class AdmissionError(ValueError):
    """Unsafe, stale or geometrically incompatible costmap."""


@dataclass(frozen=True)
class Raster:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    yaw: float
    data: tuple[int, ...]
    frame: str = "map"

    def __post_init__(self) -> None:
        if (type(self.width) is not int or type(self.height) is not int
            or self.width < 1 or self.height < 1
            or self.width*self.height > 4_000_000
            or type(self.resolution) not in (float, int)
            or not math.isfinite(self.resolution) or self.resolution <= 0
            or self.frame != "map"
            or len(self.data) != self.width*self.height):
            raise AdmissionError("invalid raster dimensions or map frame")
        if any(type(v) not in (int, float) or not math.isfinite(v) for v in
               (self.origin_x, self.origin_y, self.yaw)):
            raise AdmissionError("invalid raster origin")
        if any(type(x) is not int or not -1 <= x <= 100 for x in self.data):
            raise AdmissionError("invalid occupancy values")
        object.__setattr__(self, "data", tuple(self.data))

    def centre(self, cell: int) -> tuple[float, float]:
        if type(cell) is not int or not 0 <= cell < len(self.data):
            raise AdmissionError("invalid grid index")
        xx = (cell % self.width + 0.5)*self.resolution
        yy = (cell // self.width + 0.5)*self.resolution
        co,si=math.cos(self.yaw),math.sin(self.yaw)
        return (self.origin_x+co*xx-si*yy, self.origin_y+si*xx+co*yy)

    def cell_at(self, x: float, y: float) -> int:
        if not (math.isfinite(x) and math.isfinite(y)):
            raise AdmissionError("non-finite pose")
        xx,yy=x-self.origin_x,y-self.origin_y
        co,si=math.cos(self.yaw),math.sin(self.yaw)
        col=math.floor((co*xx+si*yy)/self.resolution)
        row=math.floor((-si*xx+co*yy)/self.resolution)
        if not 0 <= col < self.width or not 0 <= row < self.height:
            raise AdmissionError("pose outside map")
        return row*self.width+col

    @classmethod
    def from_ros(cls, grid) -> "Raster":
        q=grid.info.origin.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return cls(grid.info.width,grid.info.height,grid.info.resolution,
                   grid.info.origin.position.x,grid.info.origin.position.y,
                   yaw,tuple(grid.data),grid.header.frame_id)


@dataclass(frozen=True)
class CostmapChecker:
    static_map: Raster
    live_costmap: Raster
    footprint_radius_m: float = 0.21
    max_cost: int = 25

    def __post_init__(self) -> None:
        if not 0.05 <= self.footprint_radius_m <= 1.0:
            raise AdmissionError("invalid conservative radius")
        if type(self.max_cost) is not int or not 0 <= self.max_cost <= 99:
            raise AdmissionError("invalid cost threshold")

    def route_admitted(self, cells: Sequence[int]) -> bool:
        if not cells:
            return False
        previous=None
        previous_xy=None
        for cell in cells:
            if type(cell) is not int or not 0 <= cell < len(self.static_map.data):
                return False
            if self.static_map.data[cell] != 0:
                return False
            if previous is not None:
                a,b=cell%self.static_map.width,cell//self.static_map.width
                x,y=previous%self.static_map.width,previous//self.static_map.width
                if abs(a-x)+abs(b-y)!=1:
                    return False
            xy=self.static_map.centre(cell)
            # Both sources must independently clear the swept robot disk.
            # The static grid can contain mapped walls that a stale or
            # cropped rolling live costmap currently reports as free.
            # Only occupancy 0 is safe in the static saved map.
            start_xy = previous_xy if previous_xy is not None else xy
            if not swept_disk_clear(self.static_map, start_xy, xy,
                                    self.footprint_radius_m, 0):
                return False
            # A separate, thresholded test is still needed for live hazards.
            if not swept_disk_clear(self.live_costmap, start_xy, xy,
                                    self.footprint_radius_m, self.max_cost):
                return False
            previous=cell
            previous_xy=xy
        return True
