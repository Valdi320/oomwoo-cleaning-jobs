"""Pure-Python failure tests for actual *post-filter* global costmap admission."""
import unittest

from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.nav_keepout_admission import (
    VerifiedGlobalKeepout, KeepoutNotEnforced)


class VerifiedGlobalKeepoutTests(unittest.TestCase):
    def setUp(self):
        self.width=10
        self.base=Raster(10,10,.1,-.5,-.5,0.,(0,)*100)
        mask=[0]*100
        for i in (44,45,54,55):mask[i]=100
        self.mask=Raster(10,10,.1,-.5,-.5,0.,tuple(mask))
        self.working=Raster(10,10,.1,-.5,-.5,0.,tuple(mask))

    def test_actual_filtered_costmap_required(self):
        with self.assertRaises(KeepoutNotEnforced):
            VerifiedGlobalKeepout(self.base,self.mask,self.base)
        ok=VerifiedGlobalKeepout(self.base,self.mask,self.working)
        self.assertEqual(ok.forbidden_count,4)

    def test_reject_forbidden_goal_even_if_nav2_returns_partial_path(self):
        guard=VerifiedGlobalKeepout(self.base,self.mask,self.working)
        for x in (44,45,54,55):
            self.assertFalse(guard.accept_target(x))
        self.assertTrue(guard.accept_target(0))
        self.assertFalse(guard.accept_route((0,44,55)))
        self.assertTrue(guard.accept_route((0,1,2)))

    def test_one_missing_lethal_cell_fails_entire_filter(self):
        d=list(self.working.data);d[55]=0
        dirty=Raster(10,10,.1,-.5,-.5,0.,tuple(d))
        with self.assertRaises(KeepoutNotEnforced):
            VerifiedGlobalKeepout(self.base,self.mask,dirty)

    def test_wrong_origin_never_authorized(self):
        shift=Raster(10,10,.1,-.4,-.5,0.,self.mask.data)
        with self.assertRaises(KeepoutNotEnforced):
            VerifiedGlobalKeepout(self.base,shift,self.working)

    def test_global_costmap_does_not_cover_all_prohibited_cells(self):
        cut=Raster(3,3,.1,-.5,-.5,0.,(100,)*9)
        with self.assertRaises(KeepoutNotEnforced):
            VerifiedGlobalKeepout(self.base,self.mask,cut)

    def test_mask_not_binary_never_accepted(self):
        d=list(self.mask.data);d[22]=50
        semi=Raster(10,10,.1,-.5,-.5,0.,tuple(d))
        with self.assertRaises(KeepoutNotEnforced):
            VerifiedGlobalKeepout(self.base,semi,self.working)

    def test_empty_mask_not_proven(self):
        with self.assertRaises(KeepoutNotEnforced):
            VerifiedGlobalKeepout(self.base,self.base,self.base)

    def test_invalid_goal_index_rejected(self):
        guard=VerifiedGlobalKeepout(self.base,self.mask,self.working)
        for x in (-1,100,None,True,0.0):
            with self.subTest(x=x):
                self.assertFalse(guard.accept_target(x))

    def test_inflated_filter_threshold_does_not_turn_low_cost_into_lethal(self):
        costs=list(self.working.data)
        costs[55]=90
        under=Raster(10,10,.1,-.5,-.5,0.,tuple(costs))
        with self.assertRaises(KeepoutNotEnforced):
            VerifiedGlobalKeepout(self.base,self.mask,under)

    def test_mask_budget_prevents_large_raster_scan(self):
        with self.assertRaises(KeepoutNotEnforced):
            VerifiedGlobalKeepout(self.base,self.mask,self.working,
                                  max_covered_cells=3)


    def test_subcell_only_centre_lethal_is_not_enforced(self):
        base=Raster(4,4,.1,0.,0.,0.,(0,)*16)
        mask=[0]*16;mask[5]=100
        mask=Raster(4,4,.1,0.,0.,0.,tuple(mask))
        costs=[0]*64;costs[3*8+3]=100
        observed=Raster(8,8,.05,0.,0.,0.,tuple(costs))
        with self.assertRaises(KeepoutNotEnforced):
            VerifiedGlobalKeepout(base,mask,observed)

    def test_four_fine_resolution_subcells_required_and_sufficient(self):
        base=Raster(4,4,.1,0.,0.,0.,(0,)*16)
        mask=[0]*16;mask[5]=100
        mask=Raster(4,4,.1,0.,0.,0.,tuple(mask))
        costs=[0]*64
        for row in (2,3):
            for col in (2,3): costs[row*8+col]=100
        observed=Raster(8,8,.05,0.,0.,0.,tuple(costs))
        self.assertEqual(VerifiedGlobalKeepout(base,mask,observed).forbidden_count,1)
        costs[2*8+2]=-1
        with self.assertRaises(KeepoutNotEnforced):
            VerifiedGlobalKeepout(base,mask,Raster(8,8,.05,0.,0.,0.,tuple(costs)))

    def test_unobserved_fraction_of_forbidden_square_rejected(self):
        base=Raster(4,4,.1,0.,0.,0.,(0,)*16)
        mask=[100]+[0]*15
        mask=Raster(4,4,.1,0.,0.,0.,tuple(mask))
        observed=Raster(8,8,.05,.025,0.,0.,(100,)*64)
        with self.assertRaises(KeepoutNotEnforced):
            VerifiedGlobalKeepout(base,mask,observed)

    def test_rotated_full_observation_still_passes(self):
        import math
        base=Raster(4,4,.1,0.,0.,0.,(0,)*16)
        mask=[0]*16;mask[5]=100
        mask=Raster(4,4,.1,0.,0.,0.,tuple(mask))
        observed=Raster(40,40,.025,-.5,-.5,math.pi/9,(100,)*1600)
        self.assertEqual(VerifiedGlobalKeepout(base,mask,observed).forbidden_count,1)


if __name__=="__main__":
    unittest.main()
