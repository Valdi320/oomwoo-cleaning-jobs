"""Source-plan provenance / obstacle-bound waypoint admission regressions."""
from __future__ import annotations
import math
import unittest

from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.upstream_sweep_adapter import (
    SafeSweepPlan,SweepAdaptationError,adapt_upstream_plan)


class UpstreamWaypointAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.free=tuple([0]*400)
        self.map=Raster(20,20,.1,-1.,-1.,0.,self.free)
        self.grid=Grid(20,20,self.free,resolution_m=.1,clearance_cells=2)

    def test_waypoints_near_wall_shift_inward(self):
        wp=((-0.85,0.),(.05,0.),(.55,0.))
        plan=adapt_upstream_plan(self.grid,self.map,wp,max_shift_m=.27)
        self.assertEqual(plan.originally_requested,3)
        self.assertEqual(plan.rejected,())
        self.assertGreater(plan.adjusted,0)
        self.assertTrue(all(i in self.grid.traversable for i in plan.selected))
        self.assertEqual(plan.require_complete(),plan)

    def test_unreachable_waypoint_fails_strict_completion(self):
        wp=((10.,10.),(.05,.05))
        p=adapt_upstream_plan(self.grid,self.map,wp,max_shift_m=.3)
        self.assertEqual(p.rejected,(0,))
        with self.assertRaises(SweepAdaptationError):
            p.require_complete()

    def test_wall_is_never_added_to_safe_output(self):
        d=list(self.free)
        for r in range(20):d[r*20+10]=100
        raster=Raster(20,20,.1,-1.,-1.,0.,tuple(d))
        grid=Grid(20,20,tuple(d),resolution_m=.1,clearance_cells=2)
        poses=((-.1,.0),(-.15,.4))
        plan=adapt_upstream_plan(grid,raster,poses,max_shift_m=.3)
        self.assertTrue(all(i in grid.traversable for i in plan.selected))

    def test_keepout_zone_is_never_accepted(self):
        mid=frozenset(i for i in range(400)
                      if 8<=i%20<=12 and 8<=i//20<=12)
        g=Grid(20,20,self.free,resolution_m=.1,clearance_cells=1,
               keepout=mid)
        p=adapt_upstream_plan(g,self.map,((.0,.0),(.55,.55)),max_shift_m=.32)
        self.assertTrue(all(i not in g.keepout for i in p.selected))

    def test_rotated_map_frame_transformation(self):
        origin=Raster(20,20,.1,2.,3.,math.pi/3,self.free)
        idx=9*20+10
        x,y=origin.centre(idx)
        p=adapt_upstream_plan(self.grid,origin,((x,y),))
        self.assertIn(p.selected[0],self.grid.traversable)
        self.assertLess(p.maximum_adjustment_m,.075)

    def test_nonfinite_or_incomplete_rejected(self):
        with self.assertRaises(SweepAdaptationError):
            adapt_upstream_plan(self.grid,self.map,((math.nan,0.),))
        with self.assertRaises(SweepAdaptationError):
            adapt_upstream_plan(self.grid,self.map,())

    def test_detour_does_not_claim_cleaning_progress(self):
        p=adapt_upstream_plan(self.grid,self.map,((-.85,0.),(.05,0.)),
                              max_shift_m=.3)
        self.assertFalse(hasattr(p,"cleaned_cells"))
        self.assertTrue(p.plan_digest)
        self.assertEqual(len(p.plan_digest),64)

    def test_repeated_waypoints_coalesced(self):
        x,y=self.map.centre(9*20+8)
        p=adapt_upstream_plan(self.grid,self.map,((x,y),(x,y),(x,y)))
        self.assertEqual(len(p.selected),1)

    def test_stable_plan_digest_and_radius_changes(self):
        poses=((-.7,0.),(.3,.4))
        a=adapt_upstream_plan(self.grid,self.map,poses,max_shift_m=.2)
        b=adapt_upstream_plan(self.grid,self.map,poses,max_shift_m=.2)
        self.assertEqual(a.plan_digest,b.plan_digest)
        c=adapt_upstream_plan(self.grid,self.map,poses,max_shift_m=.3)
        self.assertNotEqual(a.plan_digest,c.plan_digest)

    def test_wrong_underlying_occupancy_forbidden(self):
        bad=Raster(20,20,.1,-1.,-1.,0.,(100,)+self.free[1:])
        with self.assertRaises(SweepAdaptationError):
            adapt_upstream_plan(self.grid,bad,((0.,0.),))

    def test_candidate_index_bounds_check(self):
        p=adapt_upstream_plan(self.grid,self.map,((0.,0.),))
        with self.assertRaises(SweepAdaptationError):
            p.candidate_at(-1)
        with self.assertRaises(SweepAdaptationError):
            p.candidate_at(99999)

if __name__=="__main__":
    unittest.main()
