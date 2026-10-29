"""Conservative straight-line THROUGH sweep for small residual missed clusters.

A Nav2 NavigateToPose goal can succeed 0.15-0.2m before the requested
point. Near room corners that may leave 2x2 cells never under the
simulated cleaning disk. Propose endpoints BEYOND a missing cell whose
geometric approach segment actually crosses its swept disk.

Advisory only: a real Nav2 costmap gate MUST approve the route, Nav2 may
replan, and only the independent coverage meter can grant floor credit.
"""
from __future__ import annotations

import math
from .grid import Grid
from .sweep_target import NoSafeSweep


# Exact Manhattan-radius-three admission, bounded independently of dirty-set size.
_ROUTE_OFFSETS = tuple(
    (dx, dy) for dy in range(-3, 4) for dx in range(-3, 4)
    if abs(dx) + abs(dy) <= 3
)


def _sample_dirty_cells(
    x: float, y: float, missing: frozenset[int], *,
    width: int, height: int, resolution_m: float, sweep_radius_m: float,
) -> set[int]:
    """Same inclusive distance test as full scan; index only nearby cells.

    At exceptionally fine resolutions, avoid enumerating a huge window:
    retain the old full-set exact scan when radius exceeds 32 cells.
    """
    radius_cells = sweep_radius_m / resolution_m
    if not math.isfinite(radius_cells) or radius_cells > 32:
        return {
            i for i in missing
            if math.hypot((i % width - x) * resolution_m,
                          (i // width - y) * resolution_m) <= sweep_radius_m
        }
    # Small rounding guard on the bounds; the exact hypot check below
    # retains final membership and never widens the swept disk.
    epsilon = 1e-10
    xlo = max(0, math.ceil(x - radius_cells - epsilon))
    xhi = min(width - 1, math.floor(x + radius_cells + epsilon))
    ylo = max(0, math.ceil(y - radius_cells - epsilon))
    yhi = min(height - 1, math.floor(y + radius_cells + epsilon))
    nearby: set[int] = set()
    for yy in range(ylo, yhi + 1):
        for xx in range(xlo, xhi + 1):
            i = yy * width + xx
            if i in missing and math.hypot(
                (xx - x) * resolution_m, (yy - y) * resolution_m
            ) <= sweep_radius_m:
                nearby.add(i)
    return nearby


def _route_near_missing(
    route: tuple[int, ...], missing: frozenset[int], width: int, height: int
) -> bool:
    """Equivalent to any Manhattan distance <=3; avoids route x missing."""
    for index in route:
        row, col = divmod(index, width)
        for dx, dy in _ROUTE_OFFSETS:
            xx, yy = col + dx, row + dy
            if 0 <= xx < width and 0 <= yy < height and yy * width + xx in missing:
                return True
    return False



def select_line_through_route(
    grid:Grid,pose_cell:int,missing:frozenset[int],*,
    admit,previously_tried:frozenset[int]=frozenset(),
    overshoot_m:float=.43,sweep_radius_m:float=.10,
    max_missing:int=32
)->tuple[int,...]:
    safe = grid.traversable
    if (pose_cell not in safe or
        not isinstance(missing,frozenset) or not missing
        or not missing <= safe or not callable(admit)
        or not .2<=overshoot_m<=.6 or not .05<=sweep_radius_m<=.18
        or not 1<=max_missing<=100):
        raise NoSafeSweep("inadmissible line-through sweep request")
    width=grid.width
    resolution=grid.resolution_m
    x0,y0=grid.xy(pose_cell)
    targets=sorted(missing,key=lambda i:
          ((i%width-x0)**2+(i//width-y0)**2,i))
    best=None
    bestscore=None

    def clearance_line(target_x,target_y):
        vx=target_x-x0
        vy=target_y-y0
        length=math.hypot(vx,vy)
        if length<1e-9:return ()
        results=[]
        for angle in (0.,.16,-.16,.30,-.30):
            c,s=math.cos(angle),math.sin(angle)
            dx=(c*vx-s*vy)/length
            dy=(s*vx+c*vy)/length
            for over in (overshoot_m,min(.60,overshoot_m+.1)):
                col=round(target_x+dx*over/resolution)
                row=round(target_y+dy*over/resolution)
                if not (0<=col<width and 0<=row<grid.height):
                    continue
                endpoint=row*width+col
                if (endpoint==pose_cell or endpoint in previously_tried
                    or endpoint not in safe):
                    continue
                # Sample the STRAIGHT ideal Nav2 approach. A 4-connected
                # shortest path may instead bend around obstacles; never
                # presume actual Nav2 follows this segment.
                distance=math.hypot(col-x0,row-y0)
                samples=max(1,math.ceil(distance*2.))
                encountered=set()
                valid=True
                for j in range(samples+1):
                    t=j/samples
                    x=x0+(col-x0)*t
                    y=y0+(row-y0)*t
                    px,py=round(x),round(y)
                    if not (0<=px<width and 0<=py<grid.height):
                        valid=False;break
                    i=py*width+px
                    if i not in grid.traversable:
                        valid=False;break
                    encountered.update(_sample_dirty_cells(
                        x, y, missing, width=width, height=grid.height,
                        resolution_m=resolution, sweep_radius_m=sweep_radius_m
                    ))
                if valid and encountered:
                    results.append((endpoint,len(encountered),length,over))
        return results

    for cell in targets[:max_missing]:
        tx,ty=grid.xy(cell)
        for candidate,count,travel,over in clearance_line(tx,ty):
            try:
                route=grid.path_to_nearest(pose_cell,frozenset((candidate,)))
                if admit(route) is not True:continue
                # Nav2's interlock independently requires the ACTUAL
                # admitted corridor to pass within three 4-neighbour
                # cells of a missing target. A visually ideal straight
                # segment is not enough: obstacle routing may detour.
                if not _route_near_missing(route, missing, width, grid.height):
                    continue
            except Exception:continue
            score=(-count,len(route),round(abs(over-overshoot_m),4),candidate)
            if bestscore is None or score<bestscore:
                bestscore=score
                best=route
        if best is not None:
            break
    if best is None:
        raise NoSafeSweep("no safe straight-through residual gap route")
    return best
