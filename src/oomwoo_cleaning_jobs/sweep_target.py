"""Prototype goal selector for physical cleaning-disk coverage in Gazebo. The overshoot is > Nav2's
0.15m goal tolerance, so the robot should actually pass the uncleaned
cell instead of terminating just short of it.

Nav2 can terminate a NavigateToPose goal within ~0.15m of its target.
A goal at an *uncleaned cell centre* can therefore stop too soon to
sweep that cell. Select a reachable point THROUGH AND BEYOND a missing
cell, subject to exactly the same dynamic footprint-costmap admission.

Experimental only: not a full-room sweep planner. Fail closed rather
than claim a cell cleaned just because Nav2 reached a destination.
"""
from __future__ import annotations

import math
from typing import Callable, Iterable

from .grid import Grid


class NoSafeSweep(RuntimeError):
    """No admissible approach past an uncleaned cell was found."""


def _count_nearby_dirty(
    route: tuple[int, ...], remaining: frozenset[int], grid: Grid,
) -> int:
    """Count distinct dirty cells near route, without dirty x route scans.

    Radius and inclusive threshold match the original 0.11m criterion.
    Only a bounded disk (<=32 grid cells of radius) is enumerated; at very
    fine map resolution the exact prior scan is retained to avoid huge
    intermediate allocations. No measured clean credit is minted here.
    """
    if not route and remaining:
        raise ValueError("empty route")
    radius_cells = 0.11 / grid.resolution_m
    radius_squared = radius_cells ** 2
    if not math.isfinite(radius_cells) or radius_cells > 32:
        route_xy = tuple(grid.xy(i) for i in route)
        return sum(
            1 for target in remaining
            if min(
                (target % grid.width - x) ** 2
                + (target // grid.width - y) ** 2
                for x, y in route_xy
            ) <= radius_squared
        )

    limit = math.ceil(radius_cells)
    offsets = tuple(
        (dx, dy)
        for dy in range(-limit, limit + 1)
        for dx in range(-limit, limit + 1)
        if dx * dx + dy * dy <= radius_squared
    )
    close: set[int] = set()
    for index in route:
        row, col = divmod(index, grid.width)
        for dx, dy in offsets:
            x, y = col + dx, row + dy
            if 0 <= x < grid.width and 0 <= y < grid.height:
                cell = y * grid.width + x
                if cell in remaining:
                    close.add(cell)
        if len(close) == len(remaining):
            break
    return len(close)


def select_sweep_route(
    grid: Grid,
    pose_cell: int,
    remaining: frozenset[int],
    *,
    admit: Callable[[tuple[int, ...]], bool],
    previously_tried: frozenset[int] = frozenset(),
    overshoot_m: float = .40,
    max_missing: int = 16,
) -> tuple[int, ...]:
    """Find a safe cell-centre route that crosses/passes an uncovered cell.

    Nav2 may still replan dynamically; measured coverage is the only proof.
    The route is an advisory corridor only. The REAL ROS2 ActionClient
    passes its terminal target; Nav2 keeps independent collision controls.
    """
    if (type(pose_cell) is not int or pose_cell not in grid.traversable or
        not isinstance(remaining, frozenset) or not remaining
        or not remaining <= grid.traversable or not callable(admit)):
        raise NoSafeSweep("missing/inadmissible target scope or start")
    if not (type(overshoot_m) in (float,int) and math.isfinite(overshoot_m)
            and .12 <= overshoot_m <= .6):
        raise ValueError("invalid overshoot_m")
    if type(max_missing) is not int or not 1 <= max_missing <= 100:
        raise ValueError("invalid max_missing")
    sx, sy = grid.xy(pose_cell)
    targets = sorted(remaining,
                     key=lambda i:((i % grid.width-sx)**2 +
                                   (i // grid.width-sy)**2,i))
    angles = (0.0, .25, -.25, .5, -.5)
    strides = (overshoot_m, min(overshoot_m+.09,.6))
    evaluated = set()
    best = None
    best_score = None
    for target in targets[:max_missing]:
        tx,ty=grid.xy(target)
        vx,vy=tx-sx,ty-sy
        if vx==0 and vy==0:
            directions=((1.,0.),(0.,1.),(-1.,0.),(0.,-1.))
        else:
            unit=math.hypot(vx,vy)
            directions=((vx/unit,vy/unit),)
        for d0x,d0y in directions:
            for angle in angles:
                co,si=math.cos(angle),math.sin(angle)
                dx,dy=d0x*co-d0y*si,d0x*si+d0y*co
                for extra in strides:
                    col=round(tx+dx*extra/grid.resolution_m)
                    row=round(ty+dy*extra/grid.resolution_m)
                    if not (0<=col<grid.width and 0<=row<grid.height):
                        continue
                    candidate=row*grid.width+col
                    if (candidate in evaluated or candidate in previously_tried
                        or candidate not in grid.traversable or candidate==pose_cell):
                        continue
                    evaluated.add(candidate)
                    try:
                        route=grid.path_to_nearest(pose_cell,frozenset((candidate,)))
                    except ValueError:
                        continue
                    # Actual admissibility remains with current costmap,
                    # inflated footprint and independently active Nav2.
                    try:
                        if admit(route) is not True:
                            continue
                    except Exception:
                        continue
                    # At least one uncovered cell must lie close to the
                    # traversed corridor. Avoid claiming this is cleaned;
                    # the measured meter is the only acknowledgement.
                    close=_count_nearby_dirty(route,remaining,grid)
                    if close==0:
                        continue
                    # Prioritize likely new swept cells with modest travel.
                    score=(-close,len(route),abs(extra-overshoot_m),candidate)
                    if best_score is None or score<best_score:
                        best_score=score
                        best=route
        if best is not None:
            # Avoid scanning the entire map for a tiny spot.
            break
    if best is None:
        raise NoSafeSweep("no costmap-admitted through-and-beyond sweep waypoint")
    return best
