"""Independent pure-Python tests for dynamic occupancy costmap admission."""
import math
import unittest

from oomwoo_cleaning_jobs.map_admission import Raster, CostmapChecker, AdmissionError


class RasterChecks(unittest.TestCase):
    def setUp(self):
        self.base=Raster(24,24,0.1,-1.2,-1.2,0.0,tuple([0]*576))

    def test_centre_and_lookup(self):
        self.assertEqual(self.base.cell_at(*self.base.centre(290)),290)

    def test_ninety_degree_origin(self):
        map=Raster(3,4,.5,2.,-1.,math.pi/2,tuple([0]*12))
        for c in range(12):
            self.assertEqual(map.cell_at(*map.centre(c)),c)

    def test_outside_map_rejected(self):
        for xy in ((50,50),(-50,0),(math.nan,0)):
            with self.subTest(xy=xy),self.assertRaises(AdmissionError):
                self.base.cell_at(*xy)

    def test_invalid_raster_rejected(self):
        with self.assertRaises(AdmissionError):
            Raster(4,2,.05,0,0,0,(0,0))
        with self.assertRaises(AdmissionError):
            Raster(4,2,.05,0,0,0,(0,)*8,frame="odom")
        with self.assertRaises(AdmissionError):
            Raster(4,2,0,0,0,0,(0,)*8)
        with self.assertRaises(AdmissionError):
            Raster(4,2,.05,0,0,0,(-3,)*8)

    def test_safe_route_passes(self):
        costs=Raster(24,24,.1,-1.2,-1.2,0,tuple([0]*576))
        route=tuple(12*24+x for x in range(7,17))
        self.assertTrue(CostmapChecker(self.base,costs,footprint_radius_m=.21).route_admitted(route))

    def test_unknown_point_in_footprint_blocks(self):
        occupancy=[0]*576
        route=tuple(12*24+x for x in range(7,17))
        occupancy[10*24+12]=-1
        live=Raster(24,24,.1,-1.2,-1.2,0,tuple(occupancy))
        self.assertFalse(CostmapChecker(self.base,live,footprint_radius_m=.21).route_admitted(route))

    def test_dynamic_inflation_blocks(self):
        a=[0]*576
        a[12*24+12]=50
        live=Raster(24,24,.1,-1.2,-1.2,0,tuple(a))
        self.assertFalse(CostmapChecker(self.base,live,footprint_radius_m=.21).route_admitted((12*24+12,)))

    def test_unknown_start_cell_fails(self):
        a=[0]*576
        a[12*24+12]=-1
        m=Raster(24,24,.1,-1.2,-1.2,0,tuple(a))
        self.assertFalse(CostmapChecker(m,self.base).route_admitted((12*24+12,)))

    def test_nonadjacent_jump_blocked(self):
        self.assertFalse(CostmapChecker(self.base,self.base).route_admitted((288,292)))

    def test_map_origin_mismatch_blocks_when_costmap_no_overlap(self):
        bad=Raster(24,24,.1,10.,10.,0,tuple([0]*576))
        self.assertFalse(CostmapChecker(self.base,bad).route_admitted((288,)))

    def test_unsafe_radius_or_cost_limit_rejected(self):
        with self.assertRaises(AdmissionError):
            CostmapChecker(self.base,self.base,footprint_radius_m=.001)
        with self.assertRaises(AdmissionError):
            CostmapChecker(self.base,self.base,max_cost=101)

    def test_empty_route_refused(self):
        self.assertFalse(CostmapChecker(self.base,self.base).route_admitted(()))


if __name__=="__main__":
    unittest.main()
