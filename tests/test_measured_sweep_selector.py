"""Independent swept-disk measured selection tests, not Nav2 motor tests."""
import unittest

from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.upstream_sweep_adapter import adapt_upstream_plan,SweepAdaptationError
from oomwoo_cleaning_jobs.measured_sweep_selector import MeasuredSweepSelector


class MeasuredWaypointTests(unittest.TestCase):
    def setUp(self):
        a=(0,)*30*20
        self.raster=Raster(30,20,.1,-1.5,-1.,0.,a)
        self.grid=Grid(30,20,a,resolution_m=.1,clearance_cells=1)
        cells=(10*30+8,10*30+13,10*30+18)
        self.waypoints=tuple(self.raster.centre(c) for c in cells)
        self.plan=adapt_upstream_plan(self.grid,self.raster,self.waypoints)
        self.selector=MeasuredSweepSelector(self.grid,self.raster,self.plan)
        self.scope=self.grid.traversable

    def mark_disk(self,cell):
        x,y=self.raster.centre(cell)
        return tuple(100 if __import__("math").hypot(wx-x,wy-y)<=.16 else 0
                     for i in range(self.grid.width*self.grid.height)
                     for wx,wy in (self.raster.centre(i),))

    def test_first_uncovered_waypoint_selected(self):
        actual=Raster(30,20,.1,-1.5,-1.,0.,self.raster.data)
        q=self.selector.select(actual,scope=self.scope)
        self.assertEqual(q.index,0)
        self.assertGreater(q.measured_dirty_fraction,.9)

    def test_new_independent_coverage_skips_first(self):
        data=self.mark_disk(self.plan.selected[0])
        meter=Raster(30,20,.1,-1.5,-1.,0.,data)
        q=self.selector.select(meter,scope=self.scope)
        self.assertEqual(q.index,1)
        self.assertEqual(q.skipped_fully_covered,1)

    def test_multiple_marked_disks_select_last(self):
        masked=[0]*600
        for i in self.plan.selected[:2]:
            cover=self.mark_disk(i)
            masked=[max(x,y) for x,y in zip(masked,cover)]
        met=Raster(30,20,.1,-1.5,-1.,0.,tuple(masked))
        q=self.selector.select(met,scope=self.scope)
        self.assertEqual(q.index,2)
        self.assertEqual(q.skipped_fully_covered,2)

    def test_navigation_completion_does_not_change_selection(self):
        met=Raster(30,20,.1,-1.5,-1.,0.,self.raster.data)
        q0=self.selector.select(met,scope=self.scope)
        # No Nav2 success data is even accepted as an input.
        q1=self.selector.select(met,scope=self.scope)
        self.assertEqual(q0,q1)

    def test_room_scope_rejects_other_room_targets(self):
        scope=frozenset({self.plan.selected[2]})
        met=Raster(30,20,.1,-1.5,-1.,0.,self.raster.data)
        choice=self.selector.select(met,scope=scope)
        self.assertEqual(choice.index,2)
        self.assertEqual(choice.skipped_outside_scope,2)

    def test_explicitly_excluded_waypoint_skipped(self):
        met=Raster(30,20,.1,-1.5,-1.,0.,self.raster.data)
        choice=self.selector.select(met,scope=self.scope,
                        excluded=frozenset({self.plan.selected[0]}))
        self.assertEqual(choice.index,1)

    def test_full_measured_disk_coverage_returns_none(self):
        data=[100]*600
        meter=Raster(30,20,.1,-1.5,-1.,0.,tuple(data))
        self.assertIsNone(self.selector.select(meter,scope=self.scope))

    def test_unresolved_original_waypoints_not_claimed_completed(self):
        record=adapt_upstream_plan(self.grid,self.raster,
                   ((999.,999.),)+self.waypoints)
        self.assertEqual(record.rejected,(0,))
        with self.assertRaises(SweepAdaptationError):
            record.require_complete()

    def test_unknown_meter_cells_refused(self):
        data=list(self.raster.data);data[50]=-1
        m=Raster(30,20,.1,-1.5,-1.,0.,tuple(data))
        with self.assertRaises(SweepAdaptationError):
            self.selector.select(m,scope=self.scope)

    def test_source_map_rotation_mismatch_refused(self):
        wrong=Raster(30,20,.1,-1.5,-1.,.2,self.raster.data)
        with self.assertRaises(SweepAdaptationError):
            self.selector.select(wrong,scope=self.scope)

    def test_invalid_scope_refused(self):
        meter=Raster(30,20,.1,-1.5,-1.,0.,self.raster.data)
        for scope in (frozenset(),frozenset((-1,)),{1,2,3}):
            with self.subTest(scope=scope),self.assertRaises(SweepAdaptationError):
                self.selector.select(meter,scope=scope)

if __name__=="__main__":
    unittest.main()
