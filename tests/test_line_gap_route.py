"""Corner-gap navigation proposals crossing missing 2x2 patch.

Pure geometry, no guaranteed Nav2 path and NO fabricated cleaning credit.
"""
import unittest
from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.line_gap_route import select_line_through_route
from oomwoo_cleaning_jobs.sweep_target import NoSafeSweep

class LineThroughTests(unittest.TestCase):
    def setUp(self):
        self.grid=Grid(100,80,(0,)*8000,resolution_m=.05,clearance_cells=5)
        self.pose=42*100+50
        self.missing=frozenset((40*100+42,40*100+43,
                                41*100+42,41*100+43))

    def test_endpoint_beyond_corner_cluster_and_not_already_tried(self):
        route=select_line_through_route(self.grid,self.pose,self.missing,
                admit=lambda path:True)
        self.assertEqual(route[0],self.pose)
        self.assertIn(route[-1],self.grid.traversable)
        self.assertGreater(len(route),10)
        end_x,end_y=self.grid.xy(route[-1])
        self.assertLess(end_x,42)
        alternative=select_line_through_route(self.grid,self.pose,self.missing,
             admit=lambda path:True,previously_tried=frozenset((route[-1],)))
        self.assertNotEqual(alternative[-1],route[-1])

    def test_blocked_costmap_never_forces_a_waypoint(self):
        with self.assertRaises(NoSafeSweep):
            select_line_through_route(self.grid,self.pose,self.missing,
                                      admit=lambda path:False)

    def test_outside_grid_unknown_or_keepout_never_accepted(self):
        blocked=Grid(100,80,(0,)*8000,
                     resolution_m=.05,clearance_cells=5,
                     keepout=frozenset((31*100+42,)))
        route=select_line_through_route(blocked,self.pose,self.missing,
                                       admit=lambda path:True)
        self.assertTrue(all(i in blocked.traversable for i in route))

    def test_empty_or_invalid_goal_refused(self):
        with self.assertRaises(NoSafeSweep):
            select_line_through_route(self.grid,self.pose,frozenset(),
                                      admit=lambda _:True)
        with self.assertRaises(NoSafeSweep):
            select_line_through_route(self.grid,-1,self.missing,
                                      admit=lambda _:True)

if __name__=="__main__":
    unittest.main()
