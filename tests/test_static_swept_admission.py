"""Synthetic two-map route preflight tests. No Gazebo or hardware."""
import math
import unittest
from oomwoo_cleaning_jobs.map_admission import Raster, CostmapChecker

START = 210  # x=.05, y=.05 in static grid

def static(hazards=(), yaw=0.):
    data = [0]*400
    for index, value in hazards:
        data[index] = value
    return Raster(20,20,.1,-1.,-1.,yaw,tuple(data))

def live_clear():
    return Raster(200,200,.02,-2.,-2.,0.,(0,)*40000)

class StaticSweptTests(unittest.TestCase):
    def checker(self, hazards=(), yaw=0.):
        return CostmapChecker(static(hazards,yaw), live_clear())

    def test_lethal_next_to_waypoint_with_blind_live(self):
        self.assertFalse(self.checker(((211,100),)).route_admitted((START,)))

    def test_unknown_next_to_waypoint_with_blind_live(self):
        self.assertFalse(self.checker(((211,-1),)).route_admitted((START,)))

    def test_nonzero_static_below_live_threshold(self):
        self.assertFalse(self.checker(((211,20),)).route_admitted((START,)))

    def test_lethal_next_to_continuous_segment(self):
        self.assertFalse(self.checker(((190,100),)).route_admitted((START,START+1)))

    def test_rotated_static_hazard(self):
        self.assertFalse(self.checker(((211,100),),math.pi/2).route_admitted((START,)))

    def test_static_map_edge_rejects_disk_outside_known_floor(self):
        self.assertFalse(self.checker().route_admitted((0,)))

    def test_both_maps_free_preserve_route(self):
        self.assertTrue(self.checker().route_admitted((START,START+1)))

    def test_static_bbox_hazard_outside_capsule_not_false_positive(self):
        self.assertTrue(self.checker(((252,100),)).route_admitted((START,)))

if __name__=="__main__":
    unittest.main()
