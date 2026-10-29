"""Fail-closed ROS-clock age/replay gate for EXPERIMENTAL lab odometry.

Do not treat ROS2 headers as authenticated evidence: trusted publisher identity,
TF frame coherence, robot hardware braking and MCU watchdog are separate.
All times here are integer ROS nanoseconds. The ROS clock may run in sim time;
never compare it to time.monotonic() as if they shared an epoch.
"""
from __future__ import annotations


class OdomStampError(ValueError):
    """Odometry sample was stale, replayed, future dated, or clock-invalid."""


class OdomStampGate:
    """Accept only fresh, strictly advancing ROS-time odometry samples.

    Clock jumps backwards are faults, not automatic resynchronizations. An
    operator must explicitly start a fresh relay after simulator reset.
    """

    def __init__(self, *, max_past_ns: int = 350_000_000,
                 max_future_ns: int = 50_000_000):
        if (type(max_past_ns) is not int or
                not 100_000_000 <= max_past_ns <= 1_000_000_000 or
                type(max_future_ns) is not int or
                not 0 <= max_future_ns <= 100_000_000):
            raise ValueError("invalid bounded odometry timestamp policy")
        self.max_past_ns = max_past_ns
        self.max_future_ns = max_future_ns
        self._last_stamp_ns = None
        self._last_clock_ns = None

    def accept(self, *, ros_now_ns: int, stamp_sec: int,
               stamp_nanosec: int) -> int:
        """Return bounded age in nanoseconds or raise (state unchanged).

        Zero stamps or zero ROS clock are deliberately denied (including
        during sim /clock startup). bool values are not valid integers.
        """
        if (type(ros_now_ns) is not int or ros_now_ns <= 0
                or type(stamp_sec) is not int or stamp_sec < 0
                or type(stamp_nanosec) is not int
                or not 0 <= stamp_nanosec < 1_000_000_000):
            raise OdomStampError("invalid ROS clock or odometry header timestamp")
        stamp_ns = stamp_sec * 1_000_000_000 + stamp_nanosec
        if stamp_ns <= 0:
            raise OdomStampError("zero odometry timestamp not trusted")
        if self._last_clock_ns is not None and ros_now_ns < self._last_clock_ns:
            raise OdomStampError("ROS clock moved backwards")
        if self._last_stamp_ns is not None and stamp_ns <= self._last_stamp_ns:
            raise OdomStampError("duplicate or reordered odometry header")
        age_ns = ros_now_ns - stamp_ns
        if age_ns > self.max_past_ns or age_ns < -self.max_future_ns:
            raise OdomStampError("odometry header outside bounded ROS clock age")
        self._last_clock_ns = ros_now_ns
        self._last_stamp_ns = stamp_ns
        return age_ns
