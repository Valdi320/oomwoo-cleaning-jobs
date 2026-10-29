"""Reference parity for sparse/dense Chebyshev clearance (pure Python)."""
import random
import unittest
from unittest.mock import patch
from array import array
from oomwoo_cleaning_jobs.grid import Grid


def reference_blocked(grid):
    blocked = {i for i, v in enumerate(grid.occupancy) if v != 0} | set(grid.keepout)
    r = grid.clearance_cells
    if r:
        enlarged = set(blocked)
        for idx in blocked:
            x, y = idx % grid.width, idx // grid.width
            for yy in range(max(0, y-r), min(grid.height, y+r+1)):
                for xx in range(max(0, x-r), min(grid.width, x+r+1)):
                    enlarged.add(yy * grid.width + xx)
        blocked = enlarged
        for y in range(grid.height):
            for x in range(grid.width):
                if x < r or y < r or x >= grid.width-r or y >= grid.height-r:
                    blocked.add(y * grid.width + x)
    return frozenset(blocked)


class GridInflationResourceTests(unittest.TestCase):
    def test_seeded_exact_parity_300_maps(self):
        rng = random.Random(20261029)
        for case in range(300):
            w, h = rng.randint(1, 70), rng.randint(1, 70)
            r = rng.randint(0, 12)
            threshold = rng.choice((0., .001, .02, .10, .35, .75, 1.))
            occupancy = tuple(
                rng.choice((-1, 35, 100)) if rng.random() < threshold else 0
                for _ in range(w * h)
            )
            keepout = frozenset(i for i in range(w*h) if rng.random() < .013)
            grid = Grid(w, h, occupancy, keepout=keepout, clearance_cells=r)
            with self.subTest(case=case, width=w, height=h, radius=r):
                expected = reference_blocked(grid)
                self.assertEqual(grid.blocked, expected)
                self.assertEqual(grid.traversable, frozenset(range(w*h)) - expected)

    def test_dense_difference_branch_exact(self):
        w, h, r = 144, 120, 7
        occ = tuple(100 if (x * 13 + y * 19) % 9 < 3 else 0
                    for y in range(h) for x in range(w))
        grid = Grid(w, h, occ, keepout=frozenset((100, 1200, 12800)),
                    clearance_cells=r)
        with patch('oomwoo_cleaning_jobs.grid.array', wraps=array) as alloc:
            self.assertEqual(grid.blocked, reference_blocked(grid))
            self.assertGreater(alloc.call_count, 0)

    def test_sparse_skips_dense_array(self):
        grid = Grid(500, 350, tuple(0 for _ in range(175000)),
                    keepout=frozenset((500*50+50,)), clearance_cells=3)
        with patch('oomwoo_cleaning_jobs.grid.array',
                   side_effect=AssertionError('unexpected dense buffer')):
            self.assertEqual(grid.blocked, reference_blocked(grid))

    def test_all_blocked_by_boundary_without_buffer(self):
        for w, h, r in ((1, 15, 1), (30, 2, 1), (8, 10, 4), (3, 3, 100)):
            grid = Grid(w, h, (0,) * (w * h), clearance_cells=r)
            with patch('oomwoo_cleaning_jobs.grid.array',
                       side_effect=AssertionError('unexpected dense buffer')):
                self.assertEqual(grid.blocked, frozenset(range(w * h)))
                self.assertEqual(grid.blocked, reference_blocked(grid))

    def test_obstacle_kept_out_at_radius_and_boundary(self):
        w, h, r = 25, 17, 4
        obstacle = 8*w+12
        grid = Grid(w, h, (0,) * (w*h),
                    keepout=frozenset((obstacle,)), clearance_cells=r)
        self.assertEqual(grid.blocked, reference_blocked(grid))
        for dy in range(-r, r+1):
            for dx in range(-r, r+1):
                self.assertIn(obstacle+dy*w+dx, grid.blocked)


if __name__ == '__main__':
    unittest.main()
