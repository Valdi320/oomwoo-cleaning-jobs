"""Conservative whole-cell proof of a Nav2 keepout mask in an observed costmap.

This is an admission check, not a motor stop.  A lethal value sampled only
at the forbidden mask cell centre does not establish area-wide enforcement.
"""
from __future__ import annotations

import math


class KeepoutRasterProofError(ValueError):
    """The observed costmap does not prove the full forbidden area lethal."""


def _overlaps_square(corners, x, y, size):
    """Positive-area SAT overlap of a rotated rectangle and axis-aligned cell."""
    xs = [p[0] for p in corners]
    ys = [p[1] for p in corners]
    if max(xs) <= x or min(xs) >= x + size or max(ys) <= y or min(ys) >= y + size:
        return False
    # Two additional separating axes: normals of rectangle edges.
    for start, end in ((corners[0], corners[1]), (corners[0], corners[3])):
        nx, ny = -(end[1] - start[1]), end[0] - start[0]
        projections = [nx * px + ny * py for px, py in corners]
        box_center = nx * (x + size / 2) + ny * (y + size / 2)
        box_radius = size / 2 * (abs(nx) + abs(ny))
        if max(projections) <= box_center - box_radius or min(projections) >= box_center + box_radius:
            return False
    return True


def prove_forbidden_area(mask, observed, forbidden, min_lethal_cost, *, max_checks=2_000_000):
    """Require *every* observed cell overlapping each forbidden mask cell lethal.

    Whole-cell geometry in the observed raster's rotated local coordinates.
    An uncovered region or exhausted computation budget fails closed. Inputs
    are previously validated Raster objects; no ROS2 dependencies.
    """
    if not forbidden:
        raise KeepoutRasterProofError("empty forbidden area")
    if type(max_checks) is not int or max_checks < 1:
        raise KeepoutRasterProofError("invalid cell-comparison budget")
    # Exact matching grids are the normal, inexpensive path.
    if (mask.width == observed.width and mask.height == observed.height
        and mask.resolution == observed.resolution
        and mask.origin_x == observed.origin_x and mask.origin_y == observed.origin_y
        and mask.yaw == observed.yaw):
        if any(observed.data[i] < min_lethal_cost for i in forbidden):
            raise KeepoutRasterProofError("forbidden mask cell not lethal")
        return
    cm, sm = math.cos(mask.yaw), math.sin(mask.yaw)
    co, so = math.cos(observed.yaw), math.sin(observed.yaw)
    mr, cr = mask.resolution, observed.resolution
    checked = 0
    for i in forbidden:
        col, row = i % mask.width, i // mask.width
        # The four corners in the observed costmap's own coordinate system.
        corners = []
        for u, v in ((col, row), (col + 1, row),
                     (col + 1, row + 1), (col, row + 1)):
            mx, my = u * mr, v * mr
            dx = mask.origin_x + cm * mx - sm * my - observed.origin_x
            dy = mask.origin_y + sm * mx + cm * my - observed.origin_y
            corners.append((co * dx + so * dy, -so * dx + co * dy))
        xs = [p[0] for p in corners]
        ys = [p[1] for p in corners]
        # Any part of a forbidden square outside observation means no proof.
        if (min(xs) < 0 or min(ys) < 0 or
            max(xs) > observed.width * cr or max(ys) > observed.height * cr):
            raise KeepoutRasterProofError("forbidden area outside observed costmap")
        x0, x1 = max(0, math.floor(min(xs) / cr)), min(observed.width - 1, math.floor(max(xs) / cr))
        y0, y1 = max(0, math.floor(min(ys) / cr)), min(observed.height - 1, math.floor(max(ys) / cr))
        intersecting = 0
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                checked += 1
                if checked > max_checks:
                    raise KeepoutRasterProofError("area verification budget exceeded")
                if _overlaps_square(corners, x * cr, y * cr, cr):
                    intersecting += 1
                    if observed.data[y * observed.width + x] < min_lethal_cost:
                        raise KeepoutRasterProofError("partial forbidden area not lethal")
        if not intersecting:
            raise KeepoutRasterProofError("forbidden area has no observed pixels")
