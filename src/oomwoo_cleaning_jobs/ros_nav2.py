"""Async ROS2 Nav2 ActionClient transport for Phase-1 NavigationInterlock.

No motor command is published. The ROS safety path is owned by the
controller and the MCU. The caller must independently prove backend
idle before rearming after a crash or communication fault.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable

from .nav_interlock import Goal


_GOAL_STATUS = {4: "succeeded", 5: "canceled", 6: "aborted"}


@dataclass
class _Pending:
    token: str
    send_future: Any
    handle: Any = None
    cancel_requested: bool = False
    cancel_future: Any = None
    terminal: bool = False


class AsyncNav2Transport:
    """Bridge async send/accept/cancel/result futures to NavigationInterlock.

    Supports real rclpy.action.ActionClient (created by from_ros2) and fake
    futures for tests. Callbacks must be serialized in one executor thread.
    on_terminal and on_fault are mandatory: neither may assume that a
    cancellation acknowledgement stopped the robot.
    """

    def __init__(
        self,
        action_client: Any,
        goal_builder: Callable[[Goal], Any],
        *,
        on_terminal: Callable[[str, str], None],
        on_fault: Callable[[str, str], None],
        on_accepted: Callable[[str], None] | None = None,
    ):
        if not callable(goal_builder) or not callable(on_terminal) or not callable(on_fault):
            raise ValueError("goal_builder, on_terminal and on_fault required")
        self.client = action_client
        self.goal_builder = goal_builder
        self.on_terminal = on_terminal
        self.on_fault = on_fault
        self.on_accepted = on_accepted
        self._pending: dict[str, _Pending] = {}

    @classmethod
    def from_ros2(
        cls, node: Any, goal_builder: Callable[[Goal], Any], *,
        on_terminal: Callable[[str, str], None],
        on_fault: Callable[[str, str], None],
        on_accepted: Callable[[str], None] | None = None,
        action_name: str = "/navigate_to_pose",
    ) -> "AsyncNav2Transport":
        """Construct actual ROS2 Jazzy NavigateToPose ActionClient."""
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        return cls(
            ActionClient(node, NavigateToPose, action_name),
            goal_builder, on_terminal=on_terminal, on_fault=on_fault,
            on_accepted=on_accepted,
        )

    def wait_for_server(self, timeout_sec: float = 10.0) -> bool:
        return bool(self.client.wait_for_server(timeout_sec=timeout_sec))

    def send_goal(self, goal: Goal) -> None:
        if goal.token in self._pending:
            raise ValueError("duplicate goal token")
        request = self.goal_builder(goal)
        pending = _Pending(goal.token, None)
        self._pending[goal.token] = pending
        try:
            future = self.client.send_goal_async(request)
            pending.send_future = future
            future.add_done_callback(lambda done: self._accepted(goal.token, done))
        except Exception:
            # The server MAY have accepted a partially sent goal. Preserve
            # the token for cancellation / explicit idle reconciliation.
            raise

    def request_cancel(self, token: str) -> None:
        pending = self._pending.get(token)
        if pending is None or pending.terminal:
            return
        pending.cancel_requested = True
        if pending.handle is not None and pending.cancel_future is None:
            self._send_cancel(pending)

    def _accepted(self, token: str, future: Any) -> None:
        p = self._pending.get(token)
        if p is None or p.terminal:
            return
        try:
            handle = future.result()
        except Exception:
            self.on_fault(token, "NAV2_ACCEPTANCE_UNCERTAIN")
            return
        if handle is None:
            self.on_fault(token, "NAV2_ACCEPTANCE_UNCERTAIN")
            return
        if not handle.accepted:
            self._terminal(token, "rejected")
            return
        p.handle = handle
        if self.on_accepted is not None:
            self.on_accepted(token)
        try:
            result_future = handle.get_result_async()
            result_future.add_done_callback(
                lambda done: self._result(token, done))
        except Exception:
            self.on_fault(token, "NAV2_RESULT_FUTURE_UNCERTAIN")
            return
        if p.cancel_requested:
            self._send_cancel(p)

    def _send_cancel(self, pending: _Pending) -> None:
        try:
            fut = pending.handle.cancel_goal_async()
            pending.cancel_future = fut
            fut.add_done_callback(
                lambda done: self._cancel_response(pending.token, done))
        except Exception:
            self.on_fault(pending.token, "NAV2_CANCEL_UNCERTAIN")

    def _cancel_response(self, token: str, future: Any) -> None:
        p = self._pending.get(token)
        if p is None or p.terminal:
            return
        try:
            response = future.result()
        except Exception:
            self.on_fault(token, "NAV2_CANCEL_UNCERTAIN")
            return
        # goals_canceling is an acknowledgement only, not a terminal result.
        if not getattr(response, "goals_canceling", None):
            self.on_fault(token, "NAV2_CANCEL_NOT_ACCEPTED")

    def _result(self, token: str, future: Any) -> None:
        p = self._pending.get(token)
        if p is None or p.terminal:
            return
        try:
            status = int(future.result().status)
        except Exception:
            self.on_fault(token, "NAV2_RESULT_UNCERTAIN")
            return
        outcome = _GOAL_STATUS.get(status)
        if outcome is None:
            self.on_fault(token, "NAV2_RESULT_UNKNOWN_STATUS")
            return
        self._terminal(token, outcome)

    def _terminal(self, token: str, outcome: str) -> None:
        p = self._pending.get(token)
        if p is None or p.terminal:
            return
        p.terminal = True
        del self._pending[token]
        self.on_terminal(token, outcome)

    def outstanding_tokens(self) -> tuple[str, ...]:
        """Diagnostic only: no local tokens does NOT prove backend idle."""
        return tuple(self._pending)


def pose_for_cell(
    goal: Goal, *,
    width: int, height: int, resolution: float,
    origin_x: float, origin_y: float, origin_yaw: float = 0.0,
    frame_id: str = "map", stamp: Any = None,
) -> Any:
    """Build NavigateToPose.Goal with occupancy-grid translation/rotation."""
    import math
    from nav2_msgs.action import NavigateToPose
    if not (type(width) is int and type(height) is int and width > 0 and height > 0
            and type(goal.target_cell) is int and 0 <= goal.target_cell < width * height
            and type(resolution) in (float, int) and math.isfinite(resolution)
            and resolution > 0 and frame_id == "map"):
        raise ValueError("invalid map or target")
    for v in (origin_x, origin_y, origin_yaw):
        if type(v) not in (float, int) or not math.isfinite(v):
            raise ValueError("invalid map origin")
    col, row = goal.target_cell % width, goal.target_cell // width
    x_local, y_local = (col + 0.5) * resolution, (row + 0.5) * resolution
    c, s = math.cos(origin_yaw), math.sin(origin_yaw)
    pose = NavigateToPose.Goal()
    pose.pose.header.frame_id = frame_id
    if stamp is not None:
        pose.pose.header.stamp = stamp
    pose.pose.pose.position.x = origin_x + c * x_local - s * y_local
    pose.pose.pose.position.y = origin_y + s * x_local + c * y_local
    pose.pose.pose.orientation.z = math.sin(origin_yaw / 2)
    pose.pose.pose.orientation.w = math.cos(origin_yaw / 2)
    return pose
