"""Tests for bounded through-and-beyond targets (never mistaken for cleaning)."""
import unittest

from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.sweep_target import select_sweep_route, NoSafeSweep


class SweepTargetTests(unittest.TestCase):
    def setUp(self):
        self.grid=Grid(40,20,tuple([0]*800),resolution_m=.05,clearance_cells=0)
        self.pose=10*40+5
        self.remaining=frozenset((10*40+22,9*40+22,11*40+22))

    def test_waypoint_goes_beyond_missing_cell(self):
        route=select_sweep_route(self.grid,self.pose,self.remaining,admit=lambda r:True)
        endpoint=route[-1]
        self.assertGreater(endpoint%40,22)
        self.assertTrue(all(i in self.grid.traversable for i in route))
        self.assertGreater(len(route),1)

    def test_previous_target_not_repeated(self):
        first=select_sweep_route(self.grid,self.pose,self.remaining,admit=lambda r:True)
        second=select_sweep_route(self.grid,self.pose,self.remaining,
                                  previously_tried=frozenset((first[-1],)),
                                  admit=lambda r:True)
        self.assertNotEqual(first[-1],second[-1])

    def test_dynamic_costmap_can_reject_every_route(self):
        with self.assertRaises(NoSafeSweep):
            select_sweep_route(self.grid,self.pose,self.remaining,admit=lambda r:False)

    def test_costmap_exception_fails_closed(self):
        def broken(route):
            raise OSError("costmap unavailable")
        with self.assertRaises(NoSafeSweep):
            select_sweep_route(self.grid,self.pose,self.remaining,admit=broken)

    def test_no_reachable_safe_endpoint(self):
        occupancy=[0]*800
        for y in range(20):
            occupancy[y*40+24]=100
        grid=Grid(40,20,tuple(occupancy),resolution_m=.05)
        with self.assertRaises(NoSafeSweep):
            select_sweep_route(grid,self.pose,
                 frozenset((10*40+23,)),admit=lambda r:False)

    def test_zero_target_refused(self):
        with self.assertRaises(NoSafeSweep):
            select_sweep_route(self.grid,self.pose,frozenset(),admit=lambda r:True)

    def test_unsafe_target_refused(self):
        with self.assertRaises(NoSafeSweep):
            select_sweep_route(self.grid,self.pose,frozenset((-1,)),admit=lambda r:True)

    def test_overshoot_bounds_checked(self):
        for x in (-1,.02,.8,float("inf"),float("nan")):
            with self.subTest(x=x),self.assertRaises(ValueError):
                select_sweep_route(self.grid,self.pose,self.remaining,
                                   admit=lambda r:True,overshoot_m=x)

    def test_cross_map_boundaries_not_bypassed(self):
        grid=Grid(5,5,tuple([0]*25),resolution_m=.05)
        with self.assertRaises(NoSafeSweep):
            select_sweep_route(grid,12,frozenset((12,)),
                               admit=lambda r:False)

    def test_does_not_credit_or_modify_grid(self):
        before=self.grid.fingerprint()
        _=select_sweep_route(self.grid,self.pose,self.remaining,
                             admit=lambda r:True)
        self.assertEqual(self.grid.fingerprint(),before)
        self.assertEqual(len(self.remaining),3)


if __name__=="__main__":
    unittest.main()
