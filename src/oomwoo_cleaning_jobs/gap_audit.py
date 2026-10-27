"""Auditable, conservative treatment of missed cells after a sweep.

The upstream executor's unreachable or rejected waypoints must not be
silently reported as "100% cleaned." This module partitions *unverified*
centre cells into connected clusters, tests reachability from a fresh
pose, and proposes a bounded, costmap-admitted repair corridor where
one exists. Measured new coverage is the ONLY mechanism to erase gaps.

No ROS2 commands or real suction/motor assumptions.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import hashlib
import json

from .grid import Grid,MapError
from .sweep_target import select_sweep_route,NoSafeSweep


class GapAuditError(RuntimeError):
    pass


@dataclass(frozen=True)
class Gap:
    key: str
    cells: tuple[int,...]
    reachable: bool
    size: int
    bounding_box: tuple[int,int,int,int]


@dataclass(frozen=True)
class GapAudit:
    scope_size: int
    credited_cells: int
    missing_cells: int
    reachable_gaps: tuple[Gap,...]
    unreachable_gaps: tuple[Gap,...]
    unresolved_upstream_waypoints: tuple[int,...]
    require_manual_review: bool
    verified_complete: bool

    def summary(self):
        return {
            "scope_size":self.scope_size,"credited_cells":self.credited_cells,
            "missing_cells":self.missing_cells,
            "reachable_clusters":len(self.reachable_gaps),
            "unreachable_clusters":len(self.unreachable_gaps),
            "unresolved_upstream_points":len(self.unresolved_upstream_waypoints),
            "manual_review":self.require_manual_review,
            "verified_complete":self.verified_complete}


class GapAuditor:
    def __init__(self,grid:Grid,scope:frozenset[int], *,
                 unresolved_upstream:tuple[int,...]=()):
        if (not isinstance(grid,Grid) or not isinstance(scope,frozenset)
            or not scope or not scope <= grid.traversable
            or not isinstance(unresolved_upstream,tuple)
            or any(type(i) is not int or i<0 for i in unresolved_upstream)):
            raise GapAuditError("invalid gap scope or unresolved waypoint metadata")
        self.grid=grid
        self.scope=scope
        self.rejected=unresolved_upstream

    def audit(self, *, credited:frozenset[int],pose_cell:int)->GapAudit:
        if (not isinstance(credited,frozenset)
            or not credited <= self.scope or pose_cell not in self.grid.traversable):
            raise GapAuditError("coverage outside job or invalid robot pose")
        missing=set(self.scope-credited)
        reachable=self.grid.reachable(pose_cell)
        clusters=[]
        while missing:
            seed=min(missing)
            todo=deque([seed]);component={seed}
            missing.remove(seed)
            while todo:
                idx=todo.popleft()
                for nei in self.grid.neighbors(idx):
                    if nei in missing:
                        missing.remove(nei);component.add(nei);todo.append(nei)
            ordered=tuple(sorted(component))
            xs=[i%self.grid.width for i in ordered]
            ys=[i//self.grid.width for i in ordered]
            key=hashlib.sha256(json.dumps(ordered).encode()).hexdigest()[:12]
            clusters.append(Gap(key,ordered,bool(component & reachable),
                                len(ordered),(min(xs),min(ys),max(xs),max(ys))))
        clusters.sort(key=lambda z:(not z.reachable,-z.size,z.key))
        active=tuple(g for g in clusters if g.reachable)
        blocked=tuple(g for g in clusters if not g.reachable)
        return GapAudit(
            len(self.scope),len(credited),len(self.scope)-len(credited),
            active,blocked,self.rejected,
            bool(clusters or self.rejected),
            not clusters)  # External confirmed full scope, even with rejected waypoints.

    def propose_repair(
        self, *, credited:frozenset[int],pose_cell:int,
        admit_route,already_tried:frozenset[int]=frozenset(),
    )->tuple[int,...]:
        audit=self.audit(credited=credited,pose_cell=pose_cell)
        if not audit.reachable_gaps:
            raise GapAuditError("nothing safely reachable to repair")
        remaining=frozenset(i for g in audit.reachable_gaps for i in g.cells)
        try:
            route=select_sweep_route(
                self.grid,pose_cell,remaining,
                admit=admit_route,previously_tried=already_tried,
                max_missing=32)
        except (NoSafeSweep,ValueError) as ex:
            raise GapAuditError("no admissible dynamic-costmap repair corridor") from ex
        # Final admission always required; don't trust offline plan blindly.
        if admit_route(route) is not True:
            raise GapAuditError("repair path rejected by fresh Nav2 costmap")
        return route
