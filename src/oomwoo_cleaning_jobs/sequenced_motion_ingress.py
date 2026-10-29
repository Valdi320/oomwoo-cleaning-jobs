"""Strict, opt-in ingress for synthetic three-way velocity ownership.

A receiver must verify measurement provenance BEFORE calling this wrapper.
Source labels, sequence numbers, and time stamps are consistency checks;
they do not authenticate ROS2 publishers or prove a physical motor stop.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

from .dock_handoff import Velocity
from .reactive_velocity import MotionHealth, ReactiveMuxError, ThreeWayVelocityMux


@dataclass(frozen=True)
class EvidenceEnvelope:
    """External monotonic sample identity, in the receiver's clock domain."""

    source: str
    sequence: int
    sampled_at: float


class SequencedMotionMux:
    """Fail closed on replayed/stale odometry or aggregate health evidence.

    Only this public facade should receive data from a trusted adapter; no
    mutable underlying mux handle is exposed. This is still a Python
    experimental policy, not an authenticated ROS2 or MCU interlock.
    """

    def __init__(self, *, odom_source: str, health_source: str,
                 max_evidence_age_s: float = 0.20, **mux_options):
        if any(type(s) is not str or not s or s.strip() != s
               for s in (odom_source, health_source)):
            raise ValueError("nonempty fixed source IDs required")
        self.__core = ThreeWayVelocityMux(**mux_options)
        if (type(max_evidence_age_s) not in (int, float)
                or not math.isfinite(max_evidence_age_s)
                or not 0 < max_evidence_age_s <= self.__core.max_age):
            raise ValueError("evidence age must fit the watchdog window")
        self.__expected = {"odom": odom_source, "health": health_source}
        self.__last = {}
        self.__max_evidence_age = float(max_evidence_age_s)

    @property
    def owner(self) -> str:
        return self.__core.owner

    @property
    def token(self) -> str | None:
        return self.__core.token

    @property
    def stop_reason(self) -> str | None:
        return self.__core.stop_reason

    def _reject(self, message: str) -> None:
        self.__core.stop(message)
        raise ReactiveMuxError(message)

    def _check_envelope(self, channel: str, *, now: float,
                        envelope: EvidenceEnvelope) -> None:
        if (type(now) not in (float, int) or not math.isfinite(now) or now < 0
                or not isinstance(envelope, EvidenceEnvelope)
                or type(envelope.source) is not str
                or envelope.source != self.__expected[channel]
                or type(envelope.sequence) is not int
                or not 0 <= envelope.sequence < 2**63
                or type(envelope.sampled_at) not in (int, float)
                or not math.isfinite(envelope.sampled_at)
                or not 0 <= now - envelope.sampled_at <= self.__max_evidence_age):
            self._reject(channel + " evidence invalid, foreign, stale, or future")
        previous = self.__last.get(channel)
        if previous is not None and (
                envelope.sequence <= previous[0]
                or envelope.sampled_at <= previous[1]):
            self._reject(channel + " evidence replayed or reordered")

    def observe_odom(self, *, now: float, envelope: EvidenceEnvelope,
                     linear_mps: float, angular_rps: float) -> None:
        self._check_envelope("odom", now=now, envelope=envelope)
        self.__core.observe_odom(
            now=now, linear_mps=linear_mps, angular_rps=angular_rps)
        self.__last["odom"] = (envelope.sequence, float(envelope.sampled_at))

    def observe_health(self, *, now: float, envelope: EvidenceEnvelope,
                       proof: MotionHealth) -> None:
        self._check_envelope("health", now=now, envelope=envelope)
        self.__core.observe_health(now=now, proof=proof)
        self.__last["health"] = (envelope.sequence, float(envelope.sampled_at))

    def claim_transit(self, *, now: float, nav_goal_uuid: str) -> str:
        return self.__core.claim_transit(now=now, nav_goal_uuid=nav_goal_uuid)

    def release_transit(self, *, now: float, terminal_uuid: str,
                        terminal_verified: bool, nav_input_isolated: bool) -> None:
        self.__core.release_transit(
            now=now, terminal_uuid=terminal_uuid,
            terminal_verified=terminal_verified,
            nav_input_isolated=nav_input_isolated)

    def claim_sweep(self, *, now: float, nav_released: bool,
                    prior_terminal_uuid: str | None) -> str:
        return self.__core.claim_sweep(
            now=now, nav_released=nav_released,
            prior_terminal_uuid=prior_terminal_uuid)

    def claim_dock(self, *, now: float, nav_released: bool,
                   prior_terminal_uuid: str | None) -> str:
        return self.__core.claim_dock(
            now=now, nav_released=nav_released,
            prior_terminal_uuid=prior_terminal_uuid)

    def release_non_nav(self, *, now: float, token: str,
                        source_stopped: bool, source_isolated: bool) -> None:
        self.__core.release_non_nav(
            now=now, token=token,
            source_stopped=source_stopped,
            source_isolated=source_isolated)

    def accept_twist(self, *, source: str, token: str, now: float,
                     velocity: Velocity) -> None:
        self.__core.accept_twist(
            source=source, token=token, now=now, velocity=velocity)

    def output(self, *, now: float) -> Velocity:
        return self.__core.output(now=now)

    def stop(self, reason: str = "explicit stop") -> None:
        # Last accepted sequence numbers intentionally survive a STOP.
        # A new process starts STOP and must reconcile all external sources.
        self.__core.stop(reason)
