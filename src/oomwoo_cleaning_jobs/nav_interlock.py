"""Fail-closed handoff between CleaningJobs and an asynchronous Nav2 action client.

The core does NOT publish ROS 2 commands or claim to stop a physical robot.
A ROS 2 adapter supplies GoalTransport and must satisfy NAVIGATION_HANDOFF.md.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import time
from typing import Callable, Protocol
from uuid import uuid4

from .engine import CleaningJobs, JobError, Resources


@dataclass(frozen=True)
class Goal:
    """Advisory candidate. Only the target is sent to Nav2 by the adapter."""
    token: str
    job_id: str
    target_cell: int
    candidate_cells: tuple[int, ...]
    map_digest: str


class GoalTransport(Protocol):
    """The ROS2 adapter owns the REAL Nav2 action handle and cancellation future."""
    def send_goal(self, goal: Goal) -> None: ...
    def request_cancel(self, token: str) -> None: ...


class NavigationInterlock:
    """One active goal, cancellation barrier, stale-feedback fencing and restart gate.

    IMPORTANT: never infer that a robot stopped merely because a cancel request
    returned. New navigation is forbidden until a terminal goal result arrives.
    An uncertain action-client outcome requires explicit external reconciliation.
    """

    TERMINAL_RESULTS = frozenset(('succeeded', 'canceled', 'aborted', 'rejected'))

    def __init__(
        self,
        job: CleaningJobs,
        transport: GoalTransport,
        *,
        coverage_source: str,
        cancel_timeout_s: float = 4.0,
        clock: Callable[[], float] = time.monotonic,
    ):
        if not isinstance(coverage_source, str) or not coverage_source.strip():
            raise ValueError('coverage_source must be a configured nonempty producer')
        if type(cancel_timeout_s) not in (int, float) or not math.isfinite(cancel_timeout_s) or cancel_timeout_s <= 0:
            raise ValueError('cancel_timeout_s must be finite and positive')
        self.job = job
        self.transport = transport
        self.coverage_source = coverage_source
        self.cancel_timeout_s = float(cancel_timeout_s)
        self.clock = clock
        self._epoch = uuid4().hex
        self._seq = 0
        self._active: Goal | None = None
        self._cancel_pending = False
        self._cancel_deadline: float | None = None
        self._idle_reconciled = False  # Every new process starts disarmed.
        self._fault: str | None = None

    def snapshot(self) -> dict:
        """The adapter should publish this as diagnostics, NOT motor-safe proof."""
        return {
            'job': self.job.status(),
            'active_goal': self._active.token if self._active else None,
            'awaiting_terminal_result': self._cancel_pending,
            'idle_reconciled': self._idle_reconciled,
            'interlock_fault': self._fault,
            'can_dispatch': self.can_dispatch(),
        }

    def can_dispatch(self) -> bool:
        return (self._idle_reconciled and self._active is None
                and self._fault is None and self.job.status()['state'] == 'running')

    def confirm_backend_idle(self, *, verified_no_outstanding_goals: bool) -> None:
        """Only after the adapter has explicitly canceled/queried old action goals.

        An omitted or false verification must never arm. This method cannot
        independently establish Nav2/MCU status; the caller must prove it.
        """
        if type(verified_no_outstanding_goals) is not bool or not verified_no_outstanding_goals:
            raise JobError('backend idle is not verified; refuse to arm')
        self._active = None
        self._cancel_pending = False
        self._cancel_deadline = None
        self._fault = None
        self._idle_reconciled = True

    def dispatch(self, *, pose_cell: int,
                 admit_route: Callable[[tuple[int, ...]], bool]) -> Goal:
        """Send one candidate after Nav2 costmap/footprint admission.

        The caller must supply a real admission check, not just a map grid
        lookup. A positive check does NOT replace Nav2 planning or MCU safety.
        """
        if not self.can_dispatch():
            raise JobError('goal blocked: inactive job, unresolved cancel or interlock fault')
        route = self.job.waypoint(pose_cell)
        if not callable(admit_route):
            raise JobError('missing route admission callback')
        try:
            admitted = admit_route(route)
        except Exception as exc:
            self._hold('ROUTE_ADMISSION_ERROR')
            raise JobError('admission check failed; job paused') from exc
        if admitted is not True:
            self._hold('ROUTE_NOT_ADMITTED')
            raise JobError('route not admitted; job paused')
        self._seq += 1
        s = self.job.status()
        goal = Goal(
            token=f'{self._epoch}-{self._seq}',
            job_id=s['job_id'],
            target_cell=route[-1],
            candidate_cells=route,
            map_digest=self.job.grid.fingerprint(),
        )
        self._active = goal  # Register before calling an asynchronous backend.
        try:
            self.transport.send_goal(goal)
        except Exception as exc:
            # The transport may have accepted the goal before raising. Never
            # assume it was rejected or launch another goal automatically.
            self._hold('NAV2_SEND_UNCERTAIN')
            try:
                # Best effort only: a ROS2 adapter must queue cancel-on-accept
                # if the action handle has not yet arrived.
                self._request_cancel()
            except JobError:
                pass  # Interlock remains latched; independent stop is needed.
            raise JobError('goal acceptance uncertain; backend reconciliation required') from exc
        return goal

    def _hold(self, reason: str) -> None:
        self._idle_reconciled = False
        self._fault = reason
        if self.job.status()['state'] == 'running':
            self.job.suspend_recovery(reason)

    def _request_cancel(self) -> bool:
        if self._active is None or self._cancel_pending:
            return False
        self._cancel_pending = True
        self._cancel_deadline = self.clock() + self.cancel_timeout_s
        try:
            self.transport.request_cancel(self._active.token)
        except Exception as exc:
            self._hold('NAV2_CANCEL_REQUEST_UNCERTAIN')
            raise JobError('cancel delivery uncertain; physically verify stop') from exc
        # This is a REQUEST only, not terminal confirmation.
        return True

    def pause(self) -> dict:
        status = self.job.pause()
        self._request_cancel()
        return status

    def observe_resources(self, resources: Resources) -> dict:
        status = self.job.observe_resources(resources)
        if status['state'] != 'running':
            self._request_cancel()
        return self.snapshot()['job']

    def pause_for_recovery(self, reason: str = 'NAV2_RECOVERY_REQUIRED') -> dict:
        status = self.job.suspend_recovery(reason)
        self._request_cancel()
        return status

    def cancel_job(self) -> dict:
        status = self.job.cancel()
        self._request_cancel()
        return status

    def resume(self, *, pose_cell: int, resources: Resources) -> dict:
        if self._active is not None or self._cancel_pending or not self._idle_reconciled or self._fault:
            raise JobError('resume blocked until previous Nav2 action is terminal and reconciled')
        return self.job.resume(pose_cell=pose_cell, resources=resources)

    def acknowledge_coverage(
        self,
        *,
        token: str,
        cells: tuple[int, ...],
        producer: str,
        map_digest: str,
    ) -> dict:
        """Only a trusted external CLEANING coverage source may acknowledge cells.

        Reaching a navigation waypoint or accepting a Nav2 goal is *not* cleaning.
        Callers must verify this evidence (a matching string alone is not proof).
        """
        if (self._active is None or token != self._active.token
            or self._cancel_pending or self._fault or not self._idle_reconciled
            or producer != self.coverage_source
            or map_digest != self.job.grid.fingerprint()):
            raise JobError('coverage rejected: stale goal, producer, map or canceled action')
        status = self.job.acknowledge_cleaned(cells)
        if status['state'] == 'completed':
            self._request_cancel()
        return status

    def on_goal_result(self, *, token: str, outcome: str) -> bool:
        """False for a stale result; never let an old result mutate new work."""
        if outcome not in self.TERMINAL_RESULTS:
            raise ValueError('invalid Nav2 terminal result')
        if self._active is None or token != self._active.token:
            return False
        self._active = None
        self._cancel_pending = False
        self._cancel_deadline = None
        if self._fault:
            # Even after observing a terminal response, require explicit
            # backend-idle reconciliation of any uncertain transport outcome.
            return True
        if self.job.status()['state'] == 'running' and outcome != 'succeeded':
            self._hold('NAV2_GOAL_' + outcome.upper())
        return True

    def check_timeout(self) -> bool:
        """Latches a fault when a cancellation has no terminal result in time."""
        if not self._cancel_pending or self._cancel_deadline is None:
            return False
        if self.clock() < self._cancel_deadline:
            return False
        if self._fault != 'NAV2_CANCEL_TIMEOUT':
            self._hold('NAV2_CANCEL_TIMEOUT')
        return True
