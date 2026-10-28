"""Continuous 2D swept-circle costmap tests; host-only, not a motor interlock."""
import math
import unittest
from oomwoo_cleaning_jobs.map_admission import Raster,CostmapChecker

class SweptFootprintTests(unittest.TestCase):
    def setUp(self):
        self.static=Raster(20,20,.1,-1.,-1.,0.,(0,)*400)
        self.cell=210 # centre (.05,.05)
    def live(self,location=(.14,.19),value=100,yaw=0.):
        origin=(.75,-.75) if yaw else (-.75,-.75)
        clean=Raster(150,150,.01,*origin,yaw,(0,)*22500)
        data=list(clean.data)
        data[clean.cell_at(*location)]=value
        return Raster(150,150,.01,*origin,yaw,tuple(data))
    def test_lethal_pixel_between_eight_probes_rejected(self):
        self.assertFalse(CostmapChecker(self.static,self.live()).route_admitted((self.cell,)))
    def test_unknown_pixel_between_eight_probes_rejected(self):
        self.assertFalse(CostmapChecker(self.static,self.live(value=-1)).route_admitted((self.cell,)))
    def test_rotated_costmap_same_lethal_pixel_rejected(self):
        self.assertFalse(CostmapChecker(self.static,self.live(yaw=math.pi/2)).route_admitted((self.cell,)))
    def test_lethal_pixel_between_route_waypoints_rejected(self):
        big=Raster(8,8,.6,-2.4,-2.4,0.,(0,)*64)
        a=35 # (col 3,row 4), centre (-.3,+.3)
        cost=Raster(180,180,.01,-.9,-.9,0.,(0,)*32400)
        data=list(cost.data);data[cost.cell_at(0.,.3)]=100
        cost=Raster(180,180,.01,-.9,-.9,0.,tuple(data))
        self.assertFalse(CostmapChecker(big,cost).route_admitted((a,a+1)))
    def test_clean_adjacent_route_remains_admitted(self):
        safe=Raster(100,100,.02,-1.,-1.,0.,(0,)*10000)
        self.assertTrue(CostmapChecker(self.static,safe).route_admitted((210,211,212)))
    def test_pixel_outside_footprint_keeps_path_clear(self):
        self.assertTrue(CostmapChecker(self.static,self.live(location=(.35,.05))).route_admitted((self.cell,)))
    def test_missing_rolling_costmap_boundary_refused(self):
        cropped=Raster(20,20,.02,-.1,-.1,0.,(0,)*400)
        self.assertFalse(CostmapChecker(self.static,cropped).route_admitted((self.cell,)))
    def test_diagonal_jump_still_refused(self):
        wide=Raster(100,100,.02,-1.,-1.,0.,(0,)*10000)
        self.assertFalse(CostmapChecker(self.static,wide).route_admitted((210,231)))
    def test_nonlethal_pixel_does_not_block(self):
        self.assertTrue(CostmapChecker(self.static,self.live(value=20)).route_admitted((self.cell,)))

if __name__=="__main__":unittest.main()
