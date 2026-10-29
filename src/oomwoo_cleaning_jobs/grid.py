"""Deterministic, conservative cell-centre routing for headless job tests.

NOT a replacement for Nav2 global/local costmaps or hardware safety. All paths
are four-connected, unknown cells are blocked and corner-cutting is disallowed.
"""
from __future__ import annotations

from collections import deque
from array import array
from dataclasses import dataclass, field
import hashlib
import json
import math
from typing import Mapping
from types import MappingProxyType


class MapError(ValueError):
    """An input map, mode, start pose or requested target is invalid."""


class UnreachableArea(MapError):
    """Some requested coverage cells cannot be reached by the proposed grid."""


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    occupancy: tuple[int, ...]
    rooms: Mapping[str, tuple[int, ...]] = field(default_factory=dict)
    keepout: frozenset[int] = field(default_factory=frozenset)
    resolution_m: float = 0.05
    # Explicit, conservative Chebyshev inflation of occupancy and keepout.
    clearance_cells: int = 0

    def __post_init__(self) -> None:
        if type(self.width) is not int or type(self.height) is not int or not (1 <= self.width <= 10000 and 1 <= self.height <= 10000):
            raise MapError('width/height must be positive integers <=10000')
        if self.width * self.height > 4_000_000:
            raise MapError('too many grid cells')
        if len(self.occupancy) != self.width * self.height:
            raise MapError('occupancy length differs from width * height')
        if any(type(x) is not int or not -1 <= x <= 100 for x in self.occupancy):
            raise MapError('occupancy must contain integers -1..100')
        if not math.isfinite(self.resolution_m) or not (0 < self.resolution_m <= 10):
            raise MapError('resolution_m must be finite and positive')
        if type(self.clearance_cells) is not int or not 0 <= self.clearance_cells <= 100:
            raise MapError('clearance_cells must be an integer 0..100')
        self._indices(self.keepout, 'keepout')
        for name, cells in self.rooms.items():
            if not isinstance(name, str) or not name or len(name) > 100:
                raise MapError('room name invalid')
            self._indices(cells, 'room ' + name)
        # Frozen dataclasses are only shallowly frozen: copy mutable caller inputs.
        object.__setattr__(self, 'occupancy', tuple(self.occupancy))
        object.__setattr__(self, 'keepout', frozenset(self.keepout))
        object.__setattr__(self, 'rooms', MappingProxyType(
            {name: tuple(cells) for name, cells in self.rooms.items()}))

    def _indices(self, cells: object, desc: str) -> None:
        for idx in cells:
            if type(idx) is not int or not 0 <= idx < len(self.occupancy):
                raise MapError(f'{desc}: invalid cell index {idx!r}')

    def xy(self, index: int) -> tuple[int, int]:
        self._indices((index,), 'position')
        return (index % self.width, index // self.width)

    def _base_blocked(self) -> set[int]:
        return {i for i, v in enumerate(self.occupancy) if v != 0} | set(self.keepout)

    @property
    def blocked(self) -> frozenset[int]:
        """Conservatively inflate obstacles in Chebyshev distance.

        Sparse maps use small per-obstacle spans; dense maps use an exact
        2D rectangular difference field. Never weaken clearance or keepouts.
        The temporary dense field costs 4*(width+1)*(height+1) bytes on
        normal 32-bit signed-int array platforms, and is per-call only.
        """
        base = self._base_blocked()
        r = self.clearance_cells
        if not r:
            return frozenset(base)
        w, h = self.width, self.height
        size = w * h
        # The original conservative outer-map margin alone covers all cells.
        if 2 * r >= w or 2 * r >= h:
            return frozenset(range(size))
        result = set(base)
        # Avoid a dense temporary buffer for a nearly empty or huge map.
        nominal = len(base) * min(w, 2 * r + 1) * min(h, 2 * r + 1)
        if nominal <= size * 3:
            for idx in base:
                x, y = idx % w, idx // w
                left, right = max(0, x - r), min(w, x + r + 1)
                for yy in range(max(0, y - r), min(h, y + r + 1)):
                    result.update(range(yy * w + left, yy * w + right))
        else:
            # Each occupied/unknown/keepout cell contributes one rectangle.
            # A signed-int count cannot overflow: <= 4,000,000 rectangles.
            stride = w + 1
            diff = array("i", [0]) * (stride * (h + 1))
            for idx in base:
                x, y = idx % w, idx // w
                left, right = max(0, x - r), min(w, x + r + 1)
                top, bottom = max(0, y - r), min(h, y + r + 1)
                a, b = top * stride, bottom * stride
                diff[a + left] += 1
                diff[a + right] -= 1
                diff[b + left] -= 1
                diff[b + right] += 1
            # In-place 2D prefix. Never approximate or threshold occupancy.
            for y in range(h):
                row = y * stride
                horizontal = 0
                for x in range(w):
                    pos = row + x
                    horizontal += diff[pos]
                    count = horizontal + (diff[pos - stride] if y else 0)
                    diff[pos] = count
                    if count > 0:
                        result.add(y * w + x)
        # Exact original outer-boundary clearance, without scanning the
        # entire interior just to add a perimeter.
        for y in range(r):
            result.update(range(y * w, (y + 1) * w))
        for y in range(h - r, h):
            result.update(range(y * w, (y + 1) * w))
        for y in range(r, h - r):
            row = y * w
            result.update(range(row, row + r))
            result.update(range(row + w - r, row + w))
        return frozenset(result)

    @property
    def traversable(self) -> frozenset[int]:
        # A small immutable occupancy map is queried repeatedly by sweep
        # candidates. Retain a bounded per-instance safe-cell snapshot,
        # never globally cache or pin a multi-million-cell map in RAM.
        cache_allowed = self.width * self.height <= 65_536
        if cache_allowed:
            cached = self.__dict__.get('_traversable_cache')
            if cached is not None:
                return cached
        bad = self.blocked
        safe = frozenset(i for i in range(len(self.occupancy)) if i not in bad)
        if cache_allowed:
            # Inputs were deeply snapshotted in __post_init__. A concurrent
            # duplicate computation is semantically identical and safe.
            object.__setattr__(self, '_traversable_cache', safe)
        return safe

    def neighbors(self, index: int) -> tuple[int, ...]:
        x, y = self.xy(index)
        choices = []
        # Stable order: left, right, up, down. Never diagonally cut a corner.
        if x: choices.append(index - 1)
        if x < self.width-1: choices.append(index + 1)
        if y: choices.append(index - self.width)
        if y < self.height-1: choices.append(index + self.width)
        return tuple(choices)

    def reachable(self, start: int) -> frozenset[int]:
        safe = self.traversable
        if start not in safe:
            raise MapError('start pose is occupied, unknown, keepout or lacks clearance')
        seen = {start}
        queue = deque((start,))
        while queue:
            for nxt in self.neighbors(queue.popleft()):
                if nxt in safe and nxt not in seen:
                    seen.add(nxt)
                    queue.append(nxt)
        return frozenset(seen)

    def path_to_nearest(self, start: int, goals: frozenset[int]) -> tuple[int, ...]:
        safe = self.traversable
        if start not in safe:
            raise MapError('current pose unsafe')
        if not goals or not goals <= safe:
            raise MapError('goals empty or unsafe')
        parents: dict[int, int | None] = {start: None}
        queue = deque((start,))
        while queue:
            node = queue.popleft()
            if node in goals:
                path = [node]
                while parents[node] is not None:
                    node = parents[node]
                    path.append(node)
                return tuple(reversed(path))
            for nxt in self.neighbors(node):
                if nxt in safe and nxt not in parents:
                    parents[nxt] = node
                    queue.append(nxt)
        raise UnreachableArea('no remaining goal is reachable from current pose')

    def targets(self, *, mode: str, rooms: tuple[str, ...] = (),
                spot: int | None = None, radius_m: float = 0) -> frozenset[int]:
        allowed = self.traversable
        if mode == 'whole':
            if rooms or spot is not None or radius_m != 0:
                raise MapError('whole mode does not accept rooms or spot')
            target = allowed
        elif mode == 'rooms':
            if not rooms or spot is not None or radius_m != 0:
                raise MapError('rooms mode requires names only')
            if len(set(rooms)) != len(rooms) or any(name not in self.rooms for name in rooms):
                raise MapError('duplicate or unknown room')
            target = allowed & frozenset(i for name in rooms for i in self.rooms[name])
        elif mode == 'spot':
            if rooms or spot is None or type(spot) is not int or not math.isfinite(radius_m) or radius_m < 0:
                raise MapError('spot mode requires cell index and finite non-negative radius_m')
            cx, cy = self.xy(spot)
            reach_cells = radius_m / self.resolution_m
            target = allowed & frozenset(i for i in range(len(self.occupancy))
                if ((i%self.width-cx)**2 + (i//self.width-cy)**2) <= reach_cells**2 + 1e-12)
        else:
            raise MapError('unknown mode')
        if not target:
            raise MapError('requested cleaning area has no safe reachable-cell candidates')
        return target

    def fingerprint(self) -> str:
        # Integrity reference on restore. A changed map never silently reuses coverage.
        doc = {'width':self.width, 'height':self.height, 'occupancy':list(self.occupancy),
               'rooms':{name:sorted(set(v)) for name, v in sorted(self.rooms.items())},
               'keepout':sorted(self.keepout), 'resolution_m':self.resolution_m,
               'clearance_cells':self.clearance_cells}
        return hashlib.sha256(json.dumps(doc,sort_keys=True,separators=(',',':')).encode()).hexdigest()
