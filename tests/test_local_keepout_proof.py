"""Exact rolling-costmap keepout overlap: synthetic maps only."""
import math
import unittest

from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.local_keepout_proof import (
    LocalKeepoutProofError, prove_local_keepout_area,
)


class LocalKeepoutAreaProofTests(unittest.TestCase):
    def setUp(self):
        cells = [0] * 16
        cells[5] = 100
        self.mask = Raster(4, 4, .1, 0., 0., 0., tuple(cells))
        self.indices = (5,)

    def fine(self, lethal=()):
        pixels = [0] * 64
        for index in lethal:
            pixels[index] = 100
        return Raster(8, 8, .05, 0., 0., 0., tuple(pixels))

    def check(self, observed, **kw):
        return prove_local_keepout_area(self.mask, observed, self.indices, **kw)

    def test_negative_control_centre_only_lethal_misses_other_subcells(self):
        bad = self.fine((27,))
        self.assertEqual(bad.data[bad.cell_at(*self.mask.centre(5))], 100)
        with self.assertRaisesRegex(LocalKeepoutProofError, "not lethal"):
            self.check(bad)

    def test_all_four_finer_subcells_lethal(self):
        fine = self.fine((18, 19, 26, 27))
        self.assertEqual(self.check(fine), 4)

    def test_unknown_subcell_rejected(self):
        pixels = list(self.fine((18, 19, 26, 27)).data)
        pixels[18] = -1
        with self.assertRaises(LocalKeepoutProofError):
            self.check(Raster(8, 8, .05, 0., 0., 0., tuple(pixels)))

    def test_identical_grid_fast_path(self):
        self.assertEqual(self.check(self.mask), 1)
        bad = Raster(4, 4, .1, 0., 0., 0., (0,) * 16)
        with self.assertRaises(LocalKeepoutProofError):
            self.check(bad)

    def test_no_local_overlap_fail_closed(self):
        distant = Raster(4, 4, .05, 9., 9., 0., (100,) * 16)
        with self.assertRaisesRegex(LocalKeepoutProofError, "no meaningful"):
            self.check(distant)

    def test_partial_border_cell_with_a_visible_centre_must_be_lethal(self):
        # .12..22 observes the (.1,.2) forbidden square in four pixels;
        # the centre (.15,.15) lies in the first observed pixel only.
        local = Raster(2, 2, .05, .12, .12, 0., (100, 0, 0, 0))
        with self.assertRaisesRegex(LocalKeepoutProofError, "not lethal"):
            self.check(local)
        full = Raster(2, 2, .05, .12, .12, 0., (100,) * 4)
        self.assertEqual(self.check(full), 4)

    def test_rotated_local_map_proves_all_intersections(self):
        local = Raster(30, 30, .05, -.5, -.5, math.pi / 6, (100,) * 900)
        self.assertGreater(self.check(local), 1)
        data = list(local.data)
        data[local.cell_at(*self.mask.centre(5))] = 0
        with self.assertRaisesRegex(LocalKeepoutProofError, "not lethal"):
            self.check(Raster(30, 30, .05, -.5, -.5, math.pi / 6, tuple(data)))

    def test_bounded_scan_rejects_unfinished_proof(self):
        with self.assertRaisesRegex(LocalKeepoutProofError, "budget"):
            self.check(self.fine((18, 19, 26, 27)), max_checks=1)

    def test_corner_sliver_without_mask_centre_not_enough(self):
        local = Raster(1, 1, .04, .1, .1, 0., (100,))
        with self.assertRaisesRegex(LocalKeepoutProofError, "no meaningful"):
            self.check(local)

    def test_invalid_input_or_threshold_rejected(self):
        with self.assertRaises(LocalKeepoutProofError):
            self.check(self.mask, min_lethal_cost=True)
        with self.assertRaises(LocalKeepoutProofError):
            self.check(self.mask, max_checks=0)
        with self.assertRaises(LocalKeepoutProofError):
            prove_local_keepout_area(self.mask, self.mask, (4,))


if __name__ == "__main__":
    unittest.main()
