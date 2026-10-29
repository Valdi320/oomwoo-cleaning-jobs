"""Strict frame contract for OOMWOO's experimental, zero-motor ROS2 relay.

OOMWOO's documented TF chain is odom -> base_footprint -> base_link.
These textual fields are NOT authenticated; TF agreement and source provenance
still require independent verification before any hardware deployment.
"""
from __future__ import annotations

ODOM_PARENT = "odom"
ODOM_CHILD = "base_footprint"


class OdomFrameError(ValueError):
    """The Odometry message is expressed in an unexpected coordinate frame."""


def require_odom_frames(frame_id: object, child_frame_id: object) -> None:
    """Require exact, non-aliased OOMWOO odom/body frames, fail-closed.

    ROS2 publishers use header.frame_id for pose and child_frame_id for twist.
    A numerically fresh zero twist in another frame is not motion evidence.
    No whitespace normalization, remapping alias or fallback is permitted here.
    """
    if (type(frame_id) is not str or type(child_frame_id) is not str
            or frame_id != ODOM_PARENT or child_frame_id != ODOM_CHILD):
        raise OdomFrameError(
            "expected odom->base_footprint, got another odometry frame")
