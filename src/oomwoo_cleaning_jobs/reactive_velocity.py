"""Simulation-only three-way velocity ownership for Nav2 transit, reactive sweep, dock.

Opt-in experiment, not a ROS2 node or physical motor safety controller.
No ROS2 /cmd_vel publisher is created. MCU braking remains independent.
Health and Nav2 terminal proofs must come from a trusted external adapter.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import re
import secrets

from .dock_handoff import Velocity


class ReactiveMuxError(RuntimeError):
    """A velocity owner, sensor proof or transition was not safe."""


@dataclass(frozen=True)
class MotionHealth:
    pose_fresh: bool
    resources_ok: bool
    map_current: bool
    safety_loop_ok: bool
    isolated_sources: bool
    nav2_idle: bool
    nav2_active_goal: str | None
    dock_inactive: bool
    sweep_inactive: bool


class ThreeWayVelocityMux:
    """A single ephemeral STOP/NAV/SWEEP/DOCK lease with continuous fail-closed output.

    Only one owner can send nonzero commands. A fresh quiet odometry period,
    explicit Nav2 terminal handoff and independent health witnesses are
    mandatory. A crash always starts STOP. No method proves real motor stop.
    """

    STOP = "stopped"
    NAV = "navigation"
    SWEEP = "reactive_sweep"
    DOCK = "dock_drive"
    MODES = frozenset((NAV, SWEEP, DOCK))

    def __init__(self, *, max_age_s: float = .30, quiet_required_s: float = .75,
                 max_linear_mps: float = .25, max_angular_rps: float = .8):
        for value, lower, upper in (
            (max_age_s, .10, 1.0), (quiet_required_s, .40, 3.0),
            (max_linear_mps, .05, .50), (max_angular_rps, .10, 1.40),
        ):
            if type(value) not in (int, float) or not math.isfinite(value) or not lower <= value <= upper:
                raise ValueError("invalid bounded motion parameter")
        self.max_age = float(max_age_s)
        self.quiet_required = float(quiet_required_s)
        self.max_linear = float(max_linear_mps)
        self.max_angular = float(max_angular_rps)
        self._last_time = None
        self.owner = self.STOP
        self.token = None
        self._expected_nav_uuid = None
        self._nav_seen_active = False
        self._nav_left_active = False
        self._last_verified_terminal = None
        self._last_odom = None
        self._quiet_since = None
        self._health = None
        self._health_time = None
        self._command = None
        self.stop_reason = "cold start: ownership not reconciled"

    def stop(self, reason: str = "explicit stop") -> None:
        if not isinstance(reason, str) or not reason:
            raise ReactiveMuxError("invalid stop reason")
        self.owner = self.STOP
        self.token = None
        self._expected_nav_uuid = None
        self._nav_seen_active = False
        self._nav_left_active = False
        self._last_verified_terminal = None
        self._last_odom = None
        self._quiet_since = None
        self._health = None
        self._health_time = None
        self._command = None
        self.stop_reason = reason

    def _clock(self, now: float) -> None:
        if type(now) not in (int, float) or not math.isfinite(now) or now < 0:
            self.stop("invalid monotonic clock")
            raise ReactiveMuxError("invalid monotonic clock")
        if self._last_time is not None and now < self._last_time:
            self.stop("clock moved backwards")
            raise ReactiveMuxError("clock moved backwards")
        self._last_time = float(now)
        # Apply the existing lease expiry before any callback can replace
        # its old timestamps. Otherwise callback-before-output ordering can
        # revive a lease after the watchdog window without a new claim.
        if self.owner != self.STOP:
            if (self._health is None or not self._fresh(self._health_time, now)
                    or not self._fresh(self._last_odom, now)
                    or not self._safe(self.owner, self._health)):
                self.stop("health/odometry lease expired")
            elif (self._command is not None
                  and not self._fresh(self._command[0], now)):
                self.stop("cmd_vel lease expired")

    @staticmethod
    def _proof_valid(proof: MotionHealth) -> bool:
        if not isinstance(proof, MotionHealth):
            return False
        flags = (proof.pose_fresh, proof.resources_ok, proof.map_current,
                 proof.safety_loop_ok, proof.isolated_sources, proof.nav2_idle,
                 proof.dock_inactive, proof.sweep_inactive)
        return (all(type(f) is bool for f in flags)
                and (proof.nav2_active_goal is None
                     or (type(proof.nav2_active_goal) is str
                         and bool(proof.nav2_active_goal.strip()))))

    def _safe(self, mode: str, proof: MotionHealth) -> bool:
        common = (proof.pose_fresh and proof.resources_ok and proof.map_current
                  and proof.safety_loop_ok and proof.isolated_sources)
        if not common:
            return False
        if mode == self.NAV:
            return (proof.dock_inactive and proof.sweep_inactive and
                    ((proof.nav2_idle and proof.nav2_active_goal is None)
                     or (not proof.nav2_idle and
                         proof.nav2_active_goal == self._expected_nav_uuid)))
        if mode == self.SWEEP:
            return (proof.nav2_idle and proof.nav2_active_goal is None
                    and proof.dock_inactive)
        if mode == self.DOCK:
            return (proof.nav2_idle and proof.nav2_active_goal is None
                    and proof.sweep_inactive)
        return False

    def observe_odom(self, *, now: float, linear_mps: float, angular_rps: float) -> None:
        self._clock(now)
        if (type(linear_mps) not in (int, float) or
            type(angular_rps) not in (int, float) or
            not math.isfinite(linear_mps) or not math.isfinite(angular_rps)):
            self.stop("invalid odometry")
            raise ReactiveMuxError("invalid odometry")
        if self._last_odom is not None and now - self._last_odom > self.max_age:
            self._quiet_since = None
        if abs(linear_mps) <= .025 and abs(angular_rps) <= .04:
            if self._quiet_since is None:
                self._quiet_since = float(now)
        else:
            self._quiet_since = None
        self._last_odom = float(now)

    def observe_health(self, *, now: float, proof: MotionHealth) -> None:
        self._clock(now)
        if not self._proof_valid(proof):
            self.stop("invalid health witness")
            raise ReactiveMuxError("invalid health witness")
        self._health = proof
        self._health_time = float(now)
        if self.owner == self.NAV:
            if proof.nav2_idle and self._nav_seen_active:
                self._nav_left_active = True
                self._command = None
            elif (not proof.nav2_idle
                  and proof.nav2_active_goal == self._expected_nav_uuid):
                if self._nav_left_active:
                    self.stop("Nav2 resumed after idle/terminal boundary")
                    raise ReactiveMuxError("Nav2 resumed after idle/terminal boundary")
                self._nav_seen_active = True
        if self.owner != self.STOP and not self._safe(self.owner, proof):
            self.stop("health contradicts active owner")
            raise ReactiveMuxError("health contradicts active owner")

    def _fresh(self, stamp: float | None, now: float) -> bool:
        return stamp is not None and 0 <= now - stamp <= self.max_age

    def _quiet(self, now: float) -> bool:
        return (self._quiet_since is not None
                and self._fresh(self._last_odom, now)
                and now - self._quiet_since >= self.quiet_required)

    def _arm(self, mode: str, *, now: float) -> str:
        self._clock(now)
        if (self.owner != self.STOP or self._health is None
            or not self._fresh(self._health_time, now)
            or not self._quiet(now) or not self._safe(mode, self._health)):
            raise ReactiveMuxError("new owner lacks fresh quiet/health/idle proofs")
        self.owner = mode
        self.token = secrets.token_hex(16)
        self.stop_reason = None
        self._command = None
        # Odometry and health must keep arriving while moving; no handoff
        # can reuse quiet odometry observed before the previous owner moved.
        self._quiet_since = None
        return self.token

    def claim_transit(self, *, now: float, nav_goal_uuid: str) -> str:
        if (type(nav_goal_uuid) is not str or
            re.fullmatch(r"[0-9a-f]{32}", nav_goal_uuid) is None):
            raise ReactiveMuxError("new Nav2 transit needs an exact UUID")
        self._clock(now)
        if (self._health is None or not self._health.nav2_idle
            or self._health.nav2_active_goal is not None):
            raise ReactiveMuxError("Nav2 backend is not idle")
        # Admission can reject a duplicate request while another lease
        # is still current. Do not replace that lease's UUID or witnesses
        # unless _arm has actually accepted the new owner.
        token = self._arm(self.NAV, now=now)
        self._expected_nav_uuid = nav_goal_uuid
        self._nav_seen_active = False
        self._nav_left_active = False
        return token

    def release_transit(self, *, now: float, terminal_uuid: str,
                        terminal_verified: bool, nav_input_isolated: bool) -> None:
        self._clock(now)
        if (self.owner != self.NAV or
            type(terminal_uuid) is not str or
            terminal_uuid != self._expected_nav_uuid or
            terminal_verified is not True or not self._nav_seen_active or
            nav_input_isolated is not True or
            self._health is None or not self._health.nav2_idle or
            self._health.nav2_active_goal is not None or
            not self._fresh(self._health_time, now)):
            self.stop("Nav2 release not proven")
            raise ReactiveMuxError("Nav2 release not proven")
        self.stop("Nav2 terminal; await fresh quiet odometry")
        self._last_verified_terminal = terminal_uuid

    def _claim_after_nav(self, *, mode: str, now: float,
                         nav_released: bool, prior_terminal_uuid: str | None) -> str:
        self._clock(now)
        # Empty-start and post-transit are different proofs. The adapter
        # must attest backend reconciliation even with no prior UUID.
        if (nav_released is not True or
            self._last_verified_terminal != prior_terminal_uuid or
            (prior_terminal_uuid is not None and
             (type(prior_terminal_uuid) is not str or not prior_terminal_uuid))):
            raise ReactiveMuxError("Nav2 release witness missing or mismatched")
        token = self._arm(mode, now=now)
        self._last_verified_terminal = None  # one-shot witness, never replay
        return token

    def claim_sweep(self, *, now: float, nav_released: bool,
                    prior_terminal_uuid: str | None) -> str:
        return self._claim_after_nav(mode=self.SWEEP, now=now,
                                     nav_released=nav_released,
                                     prior_terminal_uuid=prior_terminal_uuid)

    def claim_dock(self, *, now: float, nav_released: bool,
                   prior_terminal_uuid: str | None) -> str:
        return self._claim_after_nav(mode=self.DOCK, now=now,
                                     nav_released=nav_released,
                                     prior_terminal_uuid=prior_terminal_uuid)

    def release_non_nav(self, *, now: float, token: str,
                        source_stopped: bool, source_isolated: bool) -> None:
        self._clock(now)
        if (self.owner not in (self.SWEEP, self.DOCK)
            or token != self.token or source_stopped is not True
            or source_isolated is not True):
            self.stop("reactive/dock release not proven")
            raise ReactiveMuxError("reactive/dock release not proven")
        self.stop("source released; await fresh quiet odometry")

    def accept_twist(self, *, source: str, token: str, now: float,
                     velocity: Velocity) -> None:
        self._clock(now)
        if (type(source) is not str or source not in self.MODES or
            not isinstance(velocity, Velocity) or
            type(velocity.linear) not in (int, float) or
            type(velocity.angular) not in (int, float) or
            not math.isfinite(velocity.linear) or
            not math.isfinite(velocity.angular) or
            abs(velocity.linear) > self.max_linear or
            abs(velocity.angular) > self.max_angular):
            self.stop("invalid wheel command")
            raise ReactiveMuxError("invalid wheel command")
        if source != self.owner or type(token) is not str or token != self.token:
            if velocity != Velocity():
                self.stop("foreign/stale nonzero cmd_vel")
            raise ReactiveMuxError("source or lease token does not own cmd_vel")
        self._command = (float(now), velocity)

    def output(self, *, now: float) -> Velocity:
        self._clock(now)
        if self.owner == self.STOP:
            return Velocity()
        if self.owner == self.NAV and (not self._nav_seen_active or self._nav_left_active or self._health.nav2_idle):
            # No Nav2 motion before exact goal is observed ACTIVE or after it
            # leaves ACTIVE, even if stale Twist packets keep arriving.
            return Velocity()
        if self._command is None:
            return Velocity()
        _, velocity = self._command
        return velocity
