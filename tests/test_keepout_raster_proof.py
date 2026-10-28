"""Host-only synthetic area-overlap regressions for a private software gate."""
import math
from dataclasses import dataclass
import unittest

from oomwoo_cleaning_jobs.keepout_raster_proof import KeepoutRasterProofError, prove_forbidden_area

@dataclass(frozen=True)
class Raster:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    yaw: float
    data: tuple


def scenario(*, global_costs=None, mask_index=5):
    base = Raster(4, 4, .1, 0., 0., 0., (0,) * 16)
    mask = list(base.data)
    mask[mask_index] = 100
    mask = Raster(4, 4, .1, 0., 0., 0., tuple(mask))
    live = Raster(8, 8, .05, 0., 0., 0., tuple(global_costs if global_costs is not None else [100] * 64))
    return mask, live, [mask_index]


class AreaProofTests(unittest.TestCase):
    def test_aligned_same_geometry(self):
        m, c, cells = scenario()
        self.assertIsNone(prove_forbidden_area(m, m, cells, 95))
        with self.assertRaises(KeepoutRasterProofError):
            prove_forbidden_area(m, Raster(m.width, m.height, m.resolution,
                                         m.origin_x, m.origin_y, m.yaw, (0,) * 16), cells, 95)

    def test_single_lethal_centre_is_not_full_area_proof(self):
        # Forbidden cell index 5 covers x,y in [0.1, 0.2]. Original
        # centre-only code tests exactly one 0.05m live pixel at (0.15,0.15).
        costs = [0] * 64
        costs[3 * 8 + 3] = 100
        mask, live, cells = scenario(global_costs=costs)
        with self.assertRaises(KeepoutRasterProofError):
            prove_forbidden_area(mask, live, cells, 95)

    def test_one_of_four_subcells_clear_refuses(self):
        costs = [100] * 64
        costs[2 * 8 + 2] = 0
        mask, live, cells = scenario(global_costs=costs)
        with self.assertRaises(KeepoutRasterProofError):
            prove_forbidden_area(mask, live, cells, 95)

    def test_all_four_subcells_lethal_passes(self):
        costs = [0] * 64
        for y in (2,3):
            for x in (2,3):
                costs[y * 8 + x] = 100
        mask, live, cells = scenario(global_costs=costs)
        self.assertIsNone(prove_forbidden_area(mask, live, cells, 95))

    def test_shifted_observation_partial_overlap_not_proof(self):
        mask, live, cells = scenario(mask_index=0)
        shifted = Raster(live.width, live.height, live.resolution, .025, 0., 0., live.data)
        with self.assertRaises(KeepoutRasterProofError):
            prove_forbidden_area(mask, shifted, cells, 95)

    def test_rotated_local_grid_all_lethal_passes(self):
        mask, _, cells = scenario()
        obs = Raster(40, 40, .025, -.5, -.5, math.pi / 9, (100,) * 1600)
        self.assertIsNone(prove_forbidden_area(mask, obs, cells, 95))

    def test_rotated_local_grid_underproven_fails(self):
        mask, _, cells = scenario()
        obs = Raster(40, 40, .025, -.5, -.5, math.pi / 9, (0,) * 1600)
        with self.assertRaises(KeepoutRasterProofError):
            prove_forbidden_area(mask, obs, cells, 95)

    def test_explicit_budget_fails_closed(self):
        mask, live, cells = scenario()
        with self.assertRaisesRegex(KeepoutRasterProofError, "budget"):
            prove_forbidden_area(mask, live, cells, 95, max_checks=1)

    def test_empty_area_never_succeeds(self):
        mask, live, _ = scenario()
        with self.assertRaises(KeepoutRasterProofError):
            prove_forbidden_area(mask, live, [], 95)

if __name__ == "__main__":
    unittest.main()
