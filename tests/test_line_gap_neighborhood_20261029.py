"""Exact reference parity for bounded line-through dirty corridor and route gate."""
from __future__ import annotations
import hashlib
import math
import random
import unittest

from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.line_gap_route import (
    _route_near_missing, _sample_dirty_cells, select_line_through_route,
)
from oomwoo_cleaning_jobs.sweep_target import NoSafeSweep


class LineGapNeighborhoodTests(unittest.TestCase):
    def test_sampled_disk_exactly_matches_original_full_scan(self):
        rng = random.Random(202610290444)
        w, h = 29, 24
        population = tuple(range(w * h))
        checks = 0
        for resolution in (.001, .02, .035, .05, .1, .3, 2.0):
            for _ in range(75):
                missing = frozenset(rng.sample(population, rng.randrange(0, 240)))
                x = rng.uniform(-.5, w-.5)
                y = rng.uniform(-.5, h-.5)
                radius = rng.uniform(.05, .18)
                expected = {
                    i for i in missing
                    if math.hypot((i % w-x)*resolution,
                                  (i // w-y)*resolution) <= radius
                }
                self.assertEqual(
                    _sample_dirty_cells(
                        x, y, missing, width=w, height=h,
                        resolution_m=resolution, sweep_radius_m=radius
                    ),
                    expected,
                    f"reference mismatch at resolution {resolution}"
                )
                checks += 1
        self.assertEqual(checks, 525)

    def test_exact_inclusive_distance_and_map_edges(self):
        w,h=20,19
        missing=frozenset((0,1,20,21,10+10*w,10+12*w))
        for x,y,r in ((0,0,.1),(10.,10.,.1),(10.3,11.6,.11),
                      (18.9,18.9,.16),(3.2,2.8,.18)):
            actual=_sample_dirty_cells(x,y,missing,width=w,height=h,
                   resolution_m=.05,sweep_radius_m=r)
            expected={i for i in missing if
                      math.hypot((i%w-x)*.05,(i//w-y)*.05)<=r}
            self.assertEqual(actual,expected)

    def test_route_gate_matches_original_pairwise_manhattan(self):
        rng=random.Random(555771)
        w,h=37,31
        cells=tuple(range(w*h))
        for _ in range(300):
            route=tuple(rng.sample(cells,rng.randint(0,60)))
            missing=frozenset(rng.sample(cells,rng.randint(0,100)))
            old=any(abs(i%w-j%w)+abs(i//w-j//w)<=3
                    for i in route for j in missing)
            self.assertEqual(_route_near_missing(route,missing,w,h),old)

    def test_immutable_small_map_snapshot_cached_but_large_map_bounded(self):
        grid = Grid(24, 24, (0,) * (24 * 24), resolution_m=.05,
                    keepout=frozenset((24 * 7 + 8,)), clearance_cells=1)
        first = grid.traversable
        self.assertIs(first, grid.traversable)
        self.assertNotIn(24 * 7 + 8, first)
        self.assertNotIn(24 * 7 + 9, first)
        with self.assertRaises(Exception):
            grid.keepout = frozenset()
        # This larger synthetic grid does not retain a safe-cell cache.
        large = Grid(260, 260, (0,) * (260 * 260), resolution_m=.05)
        self.assertEqual(large.traversable, large.traversable)
        self.assertNotIn("_traversable_cache", large.__dict__)

    def test_measured_baseline_route_and_admission_contract(self):
        w,h=120,90
        grid=Grid(w,h,(0,)*(w*h),resolution_m=.05,clearance_cells=3)
        start=70*w+80
        missing=frozenset(y*w+x for y in range(30,60) for x in range(45,66))
        calls=[0]
        def admit(route):
            calls[0]+=1
            return True
        route=select_line_through_route(grid,start,missing,admit=admit)
        self.assertEqual(calls[0],10)
        self.assertEqual(len(route),42)
        self.assertEqual(hashlib.sha256(
            ",".join(map(str,route)).encode()).hexdigest(),
            "65037a5b74e8105fbbf8f5612ea5dadb09248f19319dcd486ff9253ae1f4de83")
        with self.assertRaises(NoSafeSweep):
            select_line_through_route(grid,start,missing,admit=lambda _:False)


if __name__=="__main__":
    unittest.main()
