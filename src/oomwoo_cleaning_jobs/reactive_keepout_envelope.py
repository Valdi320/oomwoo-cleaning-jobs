"""Optional synthetic keepout envelope for a *reactive* sweep /cmd_vel owner.

The upstream Nav2 KeepoutFilter cannot protect a direct /cmd_vel writer.
This independent, conservative geometric gate uses the EXISTING exact
swept-disk raster primitive. It is not a ROS2 node or motor safety layer.
All timestamps and provenance must be verified by an external adapter.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from .dock_handoff import Velocity
from .map_admission import Raster
from .reactive_velocity import ReactiveMuxError
from .sequenced_motion_ingress import SequencedMotionMux
from .swept_footprint import swept_disk_clear


@dataclass(frozen=True)
class PoseEvidence:
    x: float
    y: float
    yaw: float
    sampled_at: float  # trusted monotonic clock; never callback receipt time


@dataclass(frozen=True)
class KeepoutEvidence:
    static_map: Raster
    live_costmap: Raster
    forbidden_mask: Raster
    static_sampled_at: float
    live_sampled_at: float
    mask_sampled_at: float


class ReactiveKeepoutEnvelope:
    """Allow one sweep command only if its swept circular footprint is clear.

    Conservative bounded unicycle projection. Each arc segment's chord is
    inflated by a rigorous |v*w| * dt**2/8 curvature upper bound; the base
    radius also includes an independently budgeted stop/reaction margin.
    The stop margin must be engineered for the actual drive and MCU:
    none is established here. Motion over the prediction horizon only
    is checked; physical stopping beyond it is not guaranteed.

    Call this for every intended output tick. Do not publish the underlying
    mux output in parallel, and do not use this sweep-only wrapper for Nav2.
    """

    def __init__(self, mux: SequencedMotionMux, *,
                 footprint_radius_m: float = .21,
                 stop_margin_m: float = .06,
                 prediction_horizon_s: float = .8,
                 max_pose_age_s: float = .2,
                 max_map_age_s: float = .3,
                 live_max_cost: int = 25):
        if not isinstance(mux, SequencedMotionMux):
            raise ValueError("expected sequenced synthetic mux")
        for value, lower, upper in (
                (footprint_radius_m, .08, .50),
                (stop_margin_m, 0., .50),
                (prediction_horizon_s, .3, 1.5),
                (max_pose_age_s, .05, .3),
                (max_map_age_s, .05, 1.0)):
            if (type(value) not in (int, float)
                    or not math.isfinite(value)
                    or not lower <= value <= upper):
                raise ValueError("invalid bounded sweep envelope parameter")
        if type(live_max_cost) is not int or not 0 <= live_max_cost <= 99:
            raise ValueError("invalid live occupancy threshold")
        self._mux = mux
        self.radius = float(footprint_radius_m)
        self.margin = float(stop_margin_m)
        self.horizon = float(prediction_horizon_s)
        self.pose_age = float(max_pose_age_s)
        self.map_age = float(max_map_age_s)
        self.live_max_cost = live_max_cost
        # A Raster is an immutable validated data tuple. Re-scan each newly
        # supplied mask ONCE, not on every 20 Hz output callback.
        self._validated_mask = None

    @staticmethod
    def _fresh(stamp: float, now: float, age: float) -> bool:
        return (type(stamp) in (float, int)
                and math.isfinite(stamp)
                and 0 <= now - stamp <= age)

    @staticmethod
    def _valid_pose(pose: PoseEvidence, now: float, max_age: float) -> bool:
        return (isinstance(pose, PoseEvidence)
                and all(type(v) in (float, int) and math.isfinite(v)
                        for v in (pose.x, pose.y, pose.yaw))
                and ReactiveKeepoutEnvelope._fresh(
                    pose.sampled_at, now, max_age))

    def _valid_maps(self, maps: KeepoutEvidence, now: float, max_age: float) -> bool:
        if not isinstance(maps, KeepoutEvidence):
            return False
        a, b, c = maps.static_map, maps.live_costmap, maps.forbidden_mask
        if not all(type(grid) is Raster and grid.frame == "map" for grid in (a, b, c)):
            return False
        if (a.width, a.height, a.resolution, a.origin_x, a.origin_y, a.yaw) != (
                c.width, c.height, c.resolution, c.origin_x, c.origin_y, c.yaw):
            return False
        if c is not self._validated_mask:
            if any(v not in (0, 100) for v in c.data):
                return False
            self._validated_mask = c
        return all(ReactiveKeepoutEnvelope._fresh(stamp, now, max_age)
                   for stamp in (maps.static_sampled_at, maps.live_sampled_at,
                                 maps.mask_sampled_at))

    @staticmethod
    def _location(pose: PoseEvidence, velocity: Velocity, t: float):
        v, w = velocity.linear, velocity.angular
        if abs(w) < 1e-12:
            return (pose.x + v * t * math.cos(pose.yaw),
                    pose.y + v * t * math.sin(pose.y))
        return (pose.x + (v / w) * (math.sin(pose.yaw + w*t) - math.sin(pose.yaw)),
                pose.y - (v / w) * (math.cos(pose.yaw + w*t) - math.cos(pose.yaw)))

    def _clear(self, velocity: Velocity, pose: PoseEvidence, maps: KeepoutEvidence) -> str | None:
        # Cap each arc at <= 0.08 s AND <= 0.10 rad of rotation.
        # With max |v| 0.25 m/s and |w| 0.8 rad/s, <= 19 segments.
        n = max(1, math.ceil(self.horizon/.08),
                math.ceil(abs(velocity.angular)*self.horizon/.1))
        dt = self.horizon / n
        # The sagitta bound is valid also at zero angular velocity.
        swept_radius = (self.radius + self.margin
                        + abs(velocity.linear*velocity.angular)*dt*dt/8.)
        last = (pose.x, pose.y)
        for k in range(1, n+1):
            point = self._location(pose, velocity, self.horizon*k/n)
            for name, grid, max_cost in (
                    ("static", maps.static_map, 0),
                    ("keepout", maps.forbidden_mask, 0),
                    ("live", maps.live_costmap, self.live_max_cost)):
                if not swept_disk_clear(grid, last, point, swept_radius, max_cost):
                    return name
            last = point
        return None

    def output(self, *, now: float, pose: PoseEvidence | None,
               maps: KeepoutEvidence | None) -> Velocity:
        # Downstream watchdog and health validation must happen first.
        cmd = self._mux.output(now=now)
        if cmd == Velocity():
            return cmd
        if self._mux.owner != "reactive_sweep":
            self._mux.stop("sweep-only guard cannot relay Nav2/dock velocity")
            return Velocity()
        if not self._valid_pose(pose, now, self.pose_age):
            self._mux.stop("pose evidence missing, invalid or stale")
            return Velocity()
        if not self._valid_maps(maps, now, self.map_age):
            self._mux.stop("map evidence missing, mismatched or stale")
            return Velocity()
        try:
            hit = self._clear(cmd, pose, maps)
        except (ValueError, ArithmeticError, OverflowError, TypeError, IndexError):
            hit = "invalid geometry"
        if hit is not None:
            self._mux.stop("reactive sweep blocked by " + hit)
            return Velocity()
        return cmd
