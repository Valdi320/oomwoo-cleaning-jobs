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


def select_line_through_route(
    grid:Grid,pose_cell:int,missing:frozenset[int],*,
    admit,previously_tried:frozenset[int]=frozenset(),
    overshoot_m:float=.43,sweep_radius_m:float=.10,
    max_missing:int=32
)->tuple[int,...]:
    if (pose_cell not in grid.traversable or
        not isinstance(missing,frozenset) or not missing
        or not missing<=grid.traversable or not callable(admit)
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
                    or endpoint not in grid.traversable):
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
                    for miss in missing:
                        d=math.hypot((miss%width-x)*resolution,
                                     (miss//width-y)*resolution)
                        if d<=sweep_radius_m:encountered.add(miss)
                if valid and encountered:
                    results.append((endpoint,len(encountered),length,over))
        return results

    for cell in targets[:max_missing]:
        tx,ty=grid.xy(cell)
        for candidate,count,travel,over in clearance_line(tx,ty):
            try:
                route=grid.path_to_nearest(pose_cell,frozenset((candidate,)))
                if admit(route) is not True:continue
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
