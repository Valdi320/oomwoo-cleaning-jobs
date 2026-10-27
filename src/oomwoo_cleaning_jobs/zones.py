"""Persisted polygon and virtual-wall masks for OOMWOO *offline* zone planning.

All shapes use the ROS2 map frame, metres and the actual OccupancyGrid
origin/resolution/yaw. This module produces a candidate mask; configuring
Nav2's KeepoutFilter and enforcing an exclusive goal owner is a separate
runtime obligation. Merely saving a mask cannot prevent a live robot from
entering that area, and is NOT a hardware safety system.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import os
from pathlib import Path
import re
import tempfile

from .grid import Grid
from .map_admission import Raster


class ZoneError(ValueError):
    """Invalid geometry, changed source map or corrupted zone plan."""


def _distance_segment(x,y,a,b):
    ax,ay=a
    bx,by=b
    dx,dy=bx-ax,by-ay
    denom=dx*dx+dy*dy
    if denom<=1e-14:
        return math.hypot(x-ax,y-ay)
    t=max(0.,min(1.,((x-ax)*dx+(y-ay)*dy)/denom))
    return math.hypot(x-(ax+t*dx),y-(ay+t*dy))


def _in_polygon(x,y,vertices):
    inside=False
    n=len(vertices)
    for k in range(n):
        a=vertices[k]
        b=vertices[(k+1)%n]
        if _distance_segment(x,y,a,b)<1e-9:
            return True
        if (a[1]>y)!=(b[1]>y):
            ix=(b[0]-a[0])*(y-a[1])/(b[1]-a[1])+a[0]
            if x<ix:
                inside=not inside
    return inside


def _cross(a,b,c):
    return (b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])


def _segments_cross(a,b,c,d):
    eps=1e-9
    return (_cross(a,b,c)*_cross(a,b,d)<-eps
            and _cross(c,d,a)*_cross(c,d,b)<-eps)


@dataclass(frozen=True)
class ZoneShape:
    name: str
    kind: str                       # polygon or wall
    vertices: tuple[tuple[float,float], ...]
    width_m: float = 0.0             # for walls; polygons do not use width

    def __post_init__(self):
        if (not isinstance(self.name,str) or not re.fullmatch(r'[\w -]{1,60}',self.name,re.UNICODE)
            or self.name.strip()!=self.name):
            raise ZoneError("invalid zone name")
        if self.kind not in ("polygon","wall"):
            raise ZoneError("unknown zone type")
        pts=self.vertices
        if (type(pts) is not tuple or len(pts)>(48 if self.kind=="polygon" else 2)
            or len(pts)<(3 if self.kind=="polygon" else 2)):
            raise ZoneError("invalid vertex count")
        if any((type(pt) is not tuple or len(pt)!=2
                or any(type(v) not in (int,float) or not math.isfinite(v)
                       or abs(v)>10000 for v in pt)) for pt in pts):
            raise ZoneError("invalid world coordinates")
        if len(set(pts))!=len(pts):
            raise ZoneError("duplicate polygon or wall vertex")
        if self.kind=="wall":
            if type(self.width_m) not in (float,int) or not .02<=self.width_m<=2.0:
                raise ZoneError("wall width out of range")
            if _distance_segment(*pts[0],pts[1],pts[1])<1e-8:
                raise ZoneError("zero length wall")
        else:
            if self.width_m!=0:
                raise ZoneError("polygon cannot have wall width")
            area=abs(sum(pts[i][0]*pts[(i+1)%len(pts)][1]
                         -pts[(i+1)%len(pts)][0]*pts[i][1]
                         for i in range(len(pts)))/2)
            if area<1e-6:
                raise ZoneError("polygon has negligible area")
            for i in range(len(pts)):
                for j in range(i+1,len(pts)):
                    if (j==i+1 or (i==0 and j==len(pts)-1)):
                        continue
                    if _segments_cross(pts[i],pts[(i+1)%len(pts)],
                                       pts[j],pts[(j+1)%len(pts)]):
                        raise ZoneError("self-intersecting polygon is ambiguous")

    def contains_centre(self,x,y,resolution):
        if self.kind=="polygon":
            return _in_polygon(x,y,self.vertices)
        return _distance_segment(x,y,*self.vertices) <= max(self.width_m/2,resolution/2)

    def as_json(self):
        return {"name":self.name,"kind":self.kind,
                "vertices":[[float(x),float(y)] for x,y in self.vertices],
                "width_m":float(self.width_m)}


@dataclass(frozen=True)
class ZonePlan:
    source_map_digest: str
    map_origin: tuple[float,float,float]
    shapes: tuple[ZoneShape, ...]

    @classmethod
    def create(cls, raster: Raster, grid: Grid, shapes: tuple[ZoneShape,...]):
        if not isinstance(shapes,tuple) or len(shapes)>40:
            raise ZoneError("invalid zone list")
        if len(set(x.name for x in shapes))!=len(shapes):
            raise ZoneError("duplicate zone names")
        if (raster.frame!="map" or raster.width!=grid.width
            or raster.height!=grid.height
            or abs(raster.resolution-grid.resolution_m)>1e-6
            or raster.data != grid.occupancy):
            raise ZoneError("source map/grid mismatch")
        return cls(grid.fingerprint(),(raster.origin_x,raster.origin_y,raster.yaw),
                   shapes)

    def validate(self, raster: Raster, grid: Grid):
        if (not isinstance(self.source_map_digest,str)
            or not re.fullmatch(r'[0-9a-f]{64}',self.source_map_digest)
            or type(self.map_origin) is not tuple or len(self.map_origin)!=3
            or any(type(v) not in (float,int) or not math.isfinite(v)
                   for v in self.map_origin)
            or type(self.shapes) is not tuple or len(self.shapes)>40
            or any(not isinstance(x,ZoneShape) for x in self.shapes)):
            raise ZoneError("invalid zone plan metadata")
        if (self.source_map_digest!=grid.fingerprint()
            or raster.width!=grid.width or raster.height!=grid.height
            or raster.frame!="map"
            or raster.data!=grid.occupancy
            or abs(raster.resolution-grid.resolution_m)>1e-6
            or any(abs(a-b)>1e-7 for a,b in
                   zip(self.map_origin,(raster.origin_x,raster.origin_y,raster.yaw)))):
            raise ZoneError("source occupancy, map origin or keepout changed")
        if len(set(x.name for x in self.shapes))!=len(self.shapes):
            raise ZoneError("duplicate zones")

    def mask(self,raster: Raster,grid: Grid) -> Raster:
        self.validate(raster,grid)
        if raster.width*raster.height>500_000:
            raise ZoneError("map too large for offline mask generation")
        result=[0]*(raster.width*raster.height)
        # Rasterize only each shape's world-space AABB, transformed to the
        # map's possibly ROTATED local frame. Checking every map cell for
        # every polygon is unnecessarily expensive for apartment maps.
        co,si=math.cos(raster.yaw),math.sin(raster.yaw)
        for sh in self.shapes:
            extra=(max(sh.width_m/2,raster.resolution/2)
                   if sh.kind=="wall" else 0.0)
            xs=[p[0] for p in sh.vertices]
            ys=[p[1] for p in sh.vertices]
            wx0,wx1=min(xs)-extra,max(xs)+extra
            wy0,wy1=min(ys)-extra,max(ys)+extra
            local=[]
            for x in (wx0,wx1):
                for y in (wy0,wy1):
                    dx,dy=x-raster.origin_x,y-raster.origin_y
                    local.append((co*dx+si*dy,-si*dx+co*dy))
            u=[v[0]/raster.resolution for v in local]
            v=[v[1]/raster.resolution for v in local]
            x0=max(0,int(math.floor(min(u)))-1)
            x1=min(raster.width-1,int(math.ceil(max(u))))
            y0=max(0,int(math.floor(min(v)))-1)
            y1=min(raster.height-1,int(math.ceil(max(v))))
            for row in range(y0,y1+1):
                for col in range(x0,x1+1):
                    i=row*raster.width+col
                    if result[i]:
                        continue
                    x,y=raster.centre(i)
                    if sh.contains_centre(x,y,raster.resolution):
                        result[i]=100
        return Raster(raster.width,raster.height,raster.resolution,
                      raster.origin_x,raster.origin_y,raster.yaw,
                      tuple(result),raster.frame)

    def grid_with_keepouts(self, raster: Raster, grid: Grid) -> Grid:
        new=frozenset(i for i,cost in enumerate(self.mask(raster,grid).data)
                      if cost==100)
        return Grid(grid.width,grid.height,grid.occupancy,
                    rooms=grid.rooms,keepout=grid.keepout|new,
                    resolution_m=grid.resolution_m,clearance_cells=grid.clearance_cells)

    def save(self, path: Path, raster: Raster, grid: Grid):
        self.validate(raster,grid)
        target=Path(path)
        if target.exists():
            ZonePlan.load(target,raster,grid)  # no stale-map overwrite
        data={"schema":1,"source_map_digest":self.source_map_digest,
              "map_origin":list(self.map_origin),
              "shapes":[s.as_json() for s in self.shapes]}
        target.parent.mkdir(parents=True,exist_ok=True)
        fd,name=tempfile.mkstemp(prefix=".keepout-",suffix=".tmp",dir=target.parent)
        try:
            with os.fdopen(fd,"w",encoding="utf-8") as f:
                json.dump(data,f,sort_keys=True,separators=(",",":"))
                f.write("\n")
                f.flush()
                os.fsync(f.fileno())
            os.replace(name,target)
            if hasattr(os,"O_DIRECTORY"):
                directory=os.open(target.parent,os.O_DIRECTORY)
                try:os.fsync(directory)
                finally:os.close(directory)
        finally:
            if os.path.exists(name):os.unlink(name)

    @classmethod
    def load(cls,path: Path,raster: Raster,grid: Grid) -> "ZonePlan":
        try:
            v=json.loads(Path(path).read_text(encoding="utf-8"))
            if not isinstance(v,dict) or v.get("schema")!=1:
                raise ZoneError("invalid zone file schema")
            shapes=[]
            for x in v["shapes"]:
                shapes.append(ZoneShape(x["name"],x["kind"],
                    tuple(tuple(point) for point in x["vertices"]),
                    x["width_m"]))
            origin=v["map_origin"]
            if type(origin) is not list or len(origin)!=3:
                raise ZoneError("invalid map origin")
            plan=cls(v["source_map_digest"],tuple(origin),tuple(shapes))
            plan.validate(raster,grid)
            return plan
        except (OSError,UnicodeError,ValueError,KeyError,TypeError) as ex:
            raise ZoneError("zone plan missing, corrupted or incompatible") from ex
