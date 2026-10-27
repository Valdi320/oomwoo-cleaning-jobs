"""Validated bridge from OOMWOO's SIMULATION ground-truth coverage meter.

The upstream coverage_meter publishes /coverage_meter/covered_grid as a
nav_msgs/OccupancyGrid: 0 = not covered, 100 = swept by the simulated
cleaning disk. The meter uses simulator ground truth. It is NOT a robot
hardware sensor and its values are not cryptographically authenticated.

Do not infer cleaned floor from Nav2 goal acceptance or completion.
Only NEW 100-valued cells reported while a matching active, accepted
cleaning goal exists may be offered to the durable CleaningJobs core.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from .grid import Grid
from .map_admission import Raster


class CoverageError(ValueError):
    """Malformed, missing, stale, or implausible external coverage evidence."""


@dataclass(frozen=True)
class CoverageDelta:
    """Outcome of consuming one coverage measurement."""
    state: str
    new_cells: tuple[int, ...]
    observed_covered_cells: int
    unscoped_cells: int


class SimCoverageBridge:
    """Stateful ground-truth SIMULATOR bridge with baseline and replay fences.

    Zero credit before the first baseline. Data seen during pauses updates the
    baseline but cannot be credited on resume. Reject geometry mismatch,
    unstable simulation, out-of-order timestamps and bit reversals (which
    indicate meter reset/invalid provenance). The ROS2 caller must *halt*
    the job if a CoverageError is raised during running motion.

    This bridge does not determine whether the vacuum motor is physically on.
    It is exclusively for simulation-first OOMWOO integration.
    """

    def __init__(self, static_map: Raster, grid: Grid, *, max_age_s: float = 3.0):
        if (static_map.width != grid.width or static_map.height != grid.height
            or abs(static_map.resolution-grid.resolution_m)>1e-5):
            raise CoverageError("static map/grid geometry mismatch")
        if type(max_age_s) not in (float,int) or not math.isfinite(max_age_s) or max_age_s<=0:
            raise CoverageError("max_age_s must be positive")
        self.static_map=static_map
        self.grid=grid
        self.max_age_s=float(max_age_s)
        self._last_stamp: float | None=None
        self._prev: frozenset[int] | None=None
        self.observed=0
        self.credited=0
        self.received=0

    @property
    def ready(self) -> bool:
        return self._prev is not None

    def consume(
        self, evidence: Raster, *,
        stamp_s: float, current_sim_s: float,
        simulation_unstable: bool,
        active: bool, nav_goal_accepted: bool,
        allowed_targets: frozenset[int],
    ) -> CoverageDelta:
        """Return only *newly observed* scoped cells that may be acknowledged.

        The caller, not the bridge, invokes NavigationInterlock.acknowledge_coverage
        while the *matching* real Nav2 action token remains live.
        """
        if type(simulation_unstable) is not bool or simulation_unstable:
            raise CoverageError("coverage source reported unstable simulation")
        if not all(type(v) in (float,int) and math.isfinite(v) for v in (stamp_s,current_sim_s)):
            raise CoverageError("invalid coverage timestamp")
        if stamp_s<0 or abs(current_sim_s-stamp_s)>self.max_age_s:
            raise CoverageError("stale or future-dated coverage measurement")
        if self._last_stamp is not None and stamp_s<self._last_stamp-1e-6:
            raise CoverageError("out-of-order or restarted coverage clock")
        m=self.static_map
        if (evidence.width != m.width or evidence.height != m.height
            or evidence.frame != "map"
            or abs(evidence.resolution-m.resolution)>1e-6
            or abs(evidence.origin_x-m.origin_x)>1e-5
            or abs(evidence.origin_y-m.origin_y)>1e-5
            or abs(math.remainder(evidence.yaw-m.yaw,2*math.pi))>1e-5):
            raise CoverageError("coverage grid map-frame geometry mismatch")
        if any(x not in (0,100) for x in evidence.data):
            raise CoverageError("covered_grid requires exact 0/100 values")
        if not isinstance(allowed_targets,frozenset) or not allowed_targets <= self.grid.traversable:
            raise CoverageError("target scope missing or unsafe")
        now=frozenset(i for i,v in enumerate(evidence.data) if v==100)
        # A cleaning disk can sweep mapped-free floor that the robot's CENTRE
        # cannot traverse due to our conservative inflation. Reject only
        # physically occupied or unmapped base cells; never CREDIT the
        # non-traversable fringe as a centre-cell target.
        if any(m.data[i]<0 or m.data[i]>=50 for i in now):
            raise CoverageError("coverage marks physically occupied/unknown map cells")
        if self._prev is not None and not self._prev <= now:
            raise CoverageError("coverage grid regressed: meter reset or corruption")
        if self._last_stamp is not None and stamp_s <= self._last_stamp and now != self._prev:
            raise CoverageError("coverage changed without increasing timestamp")
        old=self._prev
        self._prev=now
        self._last_stamp=float(stamp_s)
        self.observed=len(now)
        self.received+=1
        if old is None:
            return CoverageDelta("baseline",(),len(now),len(now-allowed_targets))
        additions=now-old
        if not active or not nav_goal_accepted:
            return CoverageDelta("not_active",(),len(now),len(now-allowed_targets))
        eligible=tuple(sorted(additions & allowed_targets))
        if eligible:
            self.credited+=len(eligible)
        return CoverageDelta("new_measurement" if eligible else "no_change",
                             eligible,len(now),len(now-allowed_targets))
