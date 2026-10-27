"""Conservative, deterministic room partitioning and persistent labels for OOMWOO.

This is an *offline* map segmentation reference for cleaning-jobs, not a
claim that arbitrary cluttered buildings can be perfectly recognized from
occupancy alone. A low-clearance bottleneck heuristic proposes room splits.
Operators can additionally mark doorway cells. All candidate labels are
rejected when the source map, keepout, clearance, or dimensions change.

No Nav2 goal, motor command, global ROS topic or keepout filter is emitted.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import tempfile
from types import MappingProxyType
from typing import Mapping

from .grid import Grid, MapError


class RoomPlanError(ValueError):
    """Invalid, stale or ambiguous room partition; do not schedule a job."""


_SCHEMA = 1
_ROOM_NAME = re.compile(r'[\w -]{1,60}\Z',re.UNICODE)


def _groups(grid: Grid, indices: set[int]) -> list[set[int]]:
    """Stable four-neighbour connected components of a known index set."""
    todo = set(indices)
    groups = []
    while todo:
        first = min(todo)
        todo.remove(first)
        seen = {first}
        q = deque([first])
        while q:
            for j in grid.neighbors(q.popleft()):
                if j in todo:
                    todo.remove(j)
                    seen.add(j)
                    q.append(j)
        groups.append(seen)
    return sorted(groups, key=min)


def _wall_distances(grid: Grid) -> list[int]:
    """Manhattan distance in cells to a blocked cell or outer map boundary."""
    n = grid.width*grid.height
    distance = [-1]*n
    queue = deque()
    walls = grid.blocked
    for i in range(n):
        x,y = i%grid.width,i//grid.width
        if (i in walls or x==0 or y==0
            or x==grid.width-1 or y==grid.height-1):
            distance[i]=0
            queue.append(i)
    while queue:
        i=queue.popleft()
        nd=distance[i]+1
        for j in grid.neighbors(i):
            if distance[j]<0:
                distance[j]=nd
                queue.append(j)
    return distance


@dataclass(frozen=True)
class RoomPlan:
    source_map_digest: str
    rooms: Mapping[str, tuple[int, ...]]
    door_seals: frozenset[int]
    bottleneck_distance_cells: int
    min_seed_cells: int

    def __post_init__(self) -> None:
        if not isinstance(self.source_map_digest,str) or len(self.source_map_digest)!=64:
            raise RoomPlanError("bad source map fingerprint")
        copied = {}
        for name, cells in self.rooms.items():
            if not isinstance(name,str) or _ROOM_NAME.fullmatch(name) is None:
                raise RoomPlanError("invalid room identifier")
            if type(cells) is not tuple or not cells or any(type(i) is not int for i in cells):
                raise RoomPlanError("invalid room cell list")
            if tuple(sorted(set(cells))) != cells:
                raise RoomPlanError("room cells must be sorted and unique")
            copied[name]=tuple(cells)
        if not copied or len(copied)>100:
            raise RoomPlanError("invalid room count")
        object.__setattr__(self,"rooms",MappingProxyType(copied))
        object.__setattr__(self,"door_seals",frozenset(self.door_seals))

    def renamed(self, current_name: str, human_name: str) -> "RoomPlan":
        """Associate a verified component with an editable human label."""
        if current_name not in self.rooms:
            raise RoomPlanError("unknown room to rename")
        if (not isinstance(human_name,str) or _ROOM_NAME.fullmatch(human_name) is None
            or human_name.strip()!=human_name or human_name in self.rooms and human_name!=current_name):
            raise RoomPlanError("invalid, duplicate or unsafe human room label")
        revised={human_name if name==current_name else name:cells
                 for name,cells in self.rooms.items()}
        return RoomPlan(self.source_map_digest,revised,self.door_seals,
                        self.bottleneck_distance_cells,self.min_seed_cells)

    def validate_for(self, grid: Grid) -> None:
        if self.source_map_digest != grid.fingerprint():
            raise RoomPlanError("room plan map/keepout/clearance changed; refuse to reuse")
        traversable = grid.traversable
        seen = set()
        for name, cells in self.rooms.items():
            for i in cells:
                if i not in traversable or i in seen:
                    raise RoomPlanError("room plan overlaps, leaks into keepout or is unsafe")
                seen.add(i)
        if seen != traversable:
            raise RoomPlanError("room plan does not cover exactly all safe cells")
        if not self.door_seals <= traversable:
            raise RoomPlanError("door markers outside safe map")

    def grid_with_rooms(self, grid: Grid) -> Grid:
        self.validate_for(grid)
        return Grid(grid.width,grid.height,grid.occupancy,
                    rooms=self.rooms,keepout=grid.keepout,
                    resolution_m=grid.resolution_m,clearance_cells=grid.clearance_cells)

    def save(self, path: Path, grid: Grid) -> None:
        """Atomically save versioned room labels; never overwrite a stale map."""
        self.validate_for(grid)
        dst=Path(path)
        if dst.exists():
            # Existing plan MUST belong to this exact map before an update.
            # Map/keepout mutation requires a new path and human review.
            RoomPlan.load(dst,grid)
        data={"schema":_SCHEMA,"source_map_digest":self.source_map_digest,
              "rooms":{k:list(v) for k,v in sorted(self.rooms.items())},
              "door_seals":sorted(self.door_seals),
              "bottleneck_distance_cells":self.bottleneck_distance_cells,
              "min_seed_cells":self.min_seed_cells}
        dst.parent.mkdir(parents=True,exist_ok=True)
        fd,name=tempfile.mkstemp(prefix=".room-plan-",suffix=".tmp",dir=dst.parent)
        try:
            with os.fdopen(fd,"w",encoding="utf-8") as f:
                json.dump(data,f,sort_keys=True,separators=(",",":"))
                f.write("\n")
                f.flush()
                os.fsync(f.fileno())
            os.replace(name,dst)
            if hasattr(os,"O_DIRECTORY"):
                directory=os.open(dst.parent,os.O_DIRECTORY)
                try:os.fsync(directory)
                finally:os.close(directory)
        finally:
            if os.path.exists(name):
                os.unlink(name)

    @classmethod
    def load(cls, path: Path, grid: Grid) -> "RoomPlan":
        """Strictly validate all persisted labels, no silent remapping."""
        try:
            data=json.loads(Path(path).read_text(encoding="utf-8"))
            if not isinstance(data,dict) or data.get("schema")!=_SCHEMA:
                raise RoomPlanError("unsupported room-plan schema")
            raw=data["rooms"]
            if not isinstance(raw,dict):
                raise RoomPlanError("room-plan rooms must be an object")
            marker=data["door_seals"]
            if type(marker) is not list or any(type(i) is not int for i in marker):
                raise RoomPlanError("invalid door cells")
            assert_data=(data["bottleneck_distance_cells"],data["min_seed_cells"])
            if not all(type(v) is int for v in assert_data):
                raise RoomPlanError("invalid segmentation settings")
            plan=cls(data["source_map_digest"],
                     {str(k):tuple(v) for k,v in raw.items()},
                     frozenset(marker),*assert_data)
            plan.validate_for(grid)
            return plan
        except (OSError,UnicodeError,json.JSONDecodeError,KeyError,TypeError,ValueError) as ex:
            raise RoomPlanError("room plan missing, corrupted or incompatible") from ex


def segment_rooms(
    grid: Grid, *, door_seals: frozenset[int] = frozenset(),
    bottleneck_distance_cells: int = 2, min_seed_cells: int = 8,
) -> RoomPlan:
    """Conservatively partition open space into deterministically labeled rooms.

    Wide connected components become room seeds; narrow chokepoints/door
    passages are then assigned to their nearest seed by four-connected BFS.
    Tiny, disconnected spaces without a wide seed become separate labels.
    Manually identified door cut cells may split otherwise ambiguous rooms.

    This is a proposal on a STATIC saved occupancy grid, not a semantic room
    detector. Review the resulting labels and explicit door markers before
    commanding Nav2; dynamic obstacles and navigability still need costmaps.
    """
    if (grid.width*grid.height>500_000 or type(bottleneck_distance_cells) is not int
        or not 1<=bottleneck_distance_cells<=12 or type(min_seed_cells) is not int
        or not 1<=min_seed_cells<=5000):
        raise RoomPlanError("unsupported size or segmentation parameters")
    allowed = grid.traversable
    if not allowed:
        raise RoomPlanError("no traversable map cells")
    if (not isinstance(door_seals,frozenset)
        or any(type(i) is not int or i not in allowed for i in door_seals)):
        raise RoomPlanError("door seals must be safe grid indices")
    dist=_wall_distances(grid)
    broad={i for i in allowed if dist[i]>=bottleneck_distance_cells
           and i not in door_seals}
    seeds=[g for g in _groups(grid,broad) if len(g)>=min_seed_cells]
    if not seeds:
        seeds=_groups(grid,set(allowed)-set(door_seals))
        if not seeds:
            seeds=_groups(grid,set(allowed))
    # Assign all traversable cells to their closest seed, preserving seed
    # identity and never bridging through unsafe/blocked cells.
    owners={}
    q=deque()
    for k,seed in enumerate(seeds):
        for i in sorted(seed):
            owners[i]=k
            q.append(i)
    while q:
        i=q.popleft()
        for j in grid.neighbors(i):
            if j in allowed and j not in owners:
                owners[j]=owners[i]
                q.append(j)
    # Disconnected tiny areas with no seed must not silently disappear.
    unassigned=set(allowed)-set(owners)
    for g in _groups(grid,unassigned):
        k=len(seeds)
        seeds.append(g)
        for i in g:
            owners[i]=k
    grouped={}
    for i,k in owners.items():
        grouped.setdefault(k,[]).append(i)
    if len(grouped)>100:
        raise RoomPlanError("too many rooms; review segmentation")
    ordered=sorted(grouped.values(),key=lambda cells:min(cells))
    rooms={f"room_{k:03d}":tuple(sorted(cells))
           for k,cells in enumerate(ordered,1)}
    result=RoomPlan(grid.fingerprint(),rooms,door_seals,
                    bottleneck_distance_cells,min_seed_cells)
    result.validate_for(grid)
    return result
