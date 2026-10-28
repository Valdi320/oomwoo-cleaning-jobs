"""Prevent a per-waypoint/per-cell full-grid scan in measured selection."""
import unittest
from unittest.mock import PropertyMock,patch
from oomwoo_cleaning_jobs.grid import Grid
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.measured_sweep_selector import MeasuredSweepSelector
from oomwoo_cleaning_jobs.upstream_sweep_adapter import adapt_upstream_plan

class SelectorResourceTests(unittest.TestCase):
    def setUp(self):
        pixels=(0,)*600
        self.grid=Grid(30,20,pixels,resolution_m=.1,clearance_cells=1)
        self.map=Raster(30,20,.1,-1.5,-1.,0.,pixels)
        self.cells=(10*30+8,10*30+13,10*30+18)
        self.plan=adapt_upstream_plan(self.grid,self.map,tuple(self.map.centre(i) for i in self.cells))
        self.selector=MeasuredSweepSelector(self.grid,self.map,self.plan)
        self.safe=self.grid.traversable

    def test_all_covered_scans_grid_traversability_once(self):
        covered=Raster(30,20,.1,-1.5,-1.,0.,(100,)*600)
        with patch.object(Grid,"traversable",new_callable=PropertyMock) as prop:
            prop.return_value=self.safe
            self.assertIsNone(self.selector.select(covered,scope=self.safe))
            self.assertEqual(prop.call_count,1,"grid traversal was rebuilt for every sampled cell")

    def test_scoped_and_excluded_result_unchanged(self):
        meter=Raster(30,20,.1,-1.5,-1.,0.,self.map.data)
        picked=self.selector.select(meter,scope=self.safe,excluded=frozenset((self.plan.selected[0],)))
        self.assertEqual((picked.index,picked.target_cell),(1,self.plan.selected[1]))

    def test_invalid_scope_still_fails_closed(self):
        meter=Raster(30,20,.1,-1.5,-1.,0.,self.map.data)
        with self.assertRaises(ValueError):
            self.selector.select(meter,scope=frozenset({-1}))

if __name__=="__main__":unittest.main()
