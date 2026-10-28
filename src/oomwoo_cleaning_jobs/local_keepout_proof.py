"""Area-complete evidence for the observed part of a rolling Nav2 keepout grid.

This is NOT motor protection. Out-of-window forbidden cells are guarded by
the separate GLOBAL keepout proof; every locally observed intersection must
be lethal, including fine/rotated subcells not containing mask centres.
"""
from __future__ import annotations

import math

from .keepout_raster_proof import _overlaps_square
from .map_admission import Raster, AdmissionError


class LocalKeepoutProofError(ValueError):
    """The rolling observation does not prove its visible forbidden area."""


def prove_local_keepout_area(mask: Raster, observed: Raster, forbidden,
                             min_lethal_cost: int = 95, *,
                             max_checks: int = 2_000_000) -> int:
    """Return number of positive-area-observed forbidden intersections.

    Safety contract: every intersecting local-costmap pixel must be lethal,
    and at least one forbidden mask cell CENTRE must be inside the rolling
    window (the original minimum-visibility requirement). Forbidden squares
    OUTSIDE the local window are deliberately not expected to be observed.
    Limit comparisons for predictable failure rather than runaway work.
    """
    if not isinstance(mask, Raster) or not isinstance(observed, Raster):
        raise LocalKeepoutProofError("missing raster")
    if mask.frame != "map" or observed.frame != "map":
        raise LocalKeepoutProofError("incompatible map frame")
    if type(min_lethal_cost) is not int or not 85 <= min_lethal_cost <= 100:
        raise LocalKeepoutProofError("invalid lethal threshold")
    if type(max_checks) is not int or not 1 <= max_checks <= 2_000_000:
        raise LocalKeepoutProofError("invalid comparison limit")
    try:
        indices = tuple(forbidden)
    except (TypeError, ValueError) as exc:
        raise LocalKeepoutProofError("invalid forbidden indices") from exc
    if (not indices or len(indices) > 30_000
        or any(type(i) is not int or i < 0 or i >= len(mask.data)
               or mask.data[i] != 100 for i in indices)):
        raise LocalKeepoutProofError("invalid forbidden region")

    # Aligned and identical grids: exact O(n) equality, no SAT geometry.
    if (mask.width == observed.width and mask.height == observed.height
        and mask.resolution == observed.resolution
        and mask.origin_x == observed.origin_x and mask.origin_y == observed.origin_y
        and mask.yaw == observed.yaw):
        if any(observed.data[i] < min_lethal_cost for i in indices):
            raise LocalKeepoutProofError("visible forbidden cell is not lethal")
        return len(indices)

    co, so = math.cos(observed.yaw), math.sin(observed.yaw)
    cm, sm = math.cos(mask.yaw), math.sin(mask.yaw)
    mr, cr = mask.resolution, observed.resolution
    centre_seen = False
    intersections = 0
    comparisons = 0
    for index in indices:
        # Maintain original admission's meaningful local visibility gate:
        # merely touching the mask with a pixel corner is insufficient.
        try:
            observed.cell_at(*mask.centre(index))
            centre_seen = True
        except AdmissionError:
            pass
        col, row = index % mask.width, index // mask.width
        corners = []
        for u, v in ((col, row), (col + 1, row),
                     (col + 1, row + 1), (col, row + 1)):
            mx, my = u * mr, v * mr
            dx = mask.origin_x + cm * mx - sm * my - observed.origin_x
            dy = mask.origin_y + sm * mx + cm * my - observed.origin_y
            corners.append((co * dx + so * dy, -so * dx + co * dy))
        xs = tuple(p[0] for p in corners)
        ys = tuple(p[1] for p in corners)
        # Pad integer-window calculation for float rounding. Geometry SAT
        # itself still excludes zero-area boundary touches.
        eps = max(1e-12, cr * 1e-10)
        x0 = max(0, math.floor((min(xs) - eps) / cr))
        x1 = min(observed.width - 1, math.ceil((max(xs) + eps) / cr) - 1)
        y0 = max(0, math.floor((min(ys) - eps) / cr))
        y1 = min(observed.height - 1, math.ceil((max(ys) + eps) / cr) - 1)
        if x1 < x0 or y1 < y0:
            continue
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                comparisons += 1
                if comparisons > max_checks:
                    raise LocalKeepoutProofError("local area proof budget exceeded")
                if _overlaps_square(corners, x * cr, y * cr, cr):
                    intersections += 1
                    if observed.data[y * observed.width + x] < min_lethal_cost:
                        raise LocalKeepoutProofError("partially visible forbidden area not lethal")
    if intersections < 1 or not centre_seen:
        raise LocalKeepoutProofError("no meaningful forbidden mask overlap in local window")
    return intersections
