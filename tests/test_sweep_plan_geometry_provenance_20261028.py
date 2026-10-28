"""Regression: an adapted sweep plan MUST NOT cross map-frame revisions."""
from dataclasses import replace
from pathlib import Path
import math
import tempfile
import unittest
from oomwoo_cleaning_jobs.grid import Grid
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.measured_sweep_selector import MeasuredSweepSelector
from oomwoo_cleaning_jobs.sweep_campaign import SweepCampaign, SweepStateError
from oomwoo_cleaning_jobs.upstream_sweep_adapter import (
    adapt_upstream_plan, SweepAdaptationError,
)


class SweepFrameBindingTests(unittest.TestCase):
    def setUp(self):
        self.pixels=(0,)*400
        self.grid=Grid(20,20,self.pixels,resolution_m=.1,clearance_cells=1)
        self.map=Raster(20,20,.1,-1.,-1.,0.,self.pixels)
        self.plan=adapt_upstream_plan(
            self.grid,self.map,(self.map.centre(9*20+8),)
        )
        self.scope=self.grid.traversable

    def test_same_grid_different_translation_rejected_before_selector(self):
        moved=replace(self.map,origin_x=self.map.origin_x+.2)
        with self.assertRaises(SweepAdaptationError):
            MeasuredSweepSelector(self.grid,moved,self.plan)

    def test_same_grid_different_yaw_rejected_before_selector(self):
        rotated=replace(self.map,yaw=math.pi/2)
        with self.assertRaises(SweepAdaptationError):
            MeasuredSweepSelector(self.grid,rotated,self.plan)

    def test_persistent_campaign_rejects_foreign_frame_before_file_write(self):
        moved=replace(self.map,origin_y=self.map.origin_y-.4)
        with tempfile.TemporaryDirectory() as root:
            path=Path(root)/"session.json"
            with self.assertRaises((SweepStateError,SweepAdaptationError)):
                SweepCampaign(self.grid,moved,self.plan,path,
                              scope=self.scope,session_id="geometry")
            self.assertFalse(path.exists())

    def test_fresh_adaptation_still_works_and_is_bound_to_new_map(self):
        shifted=replace(self.map,origin_x=.3,yaw=.11)
        plan=adapt_upstream_plan(self.grid,shifted,
                                 (shifted.centre(9*20+8),))
        self.assertIsNotNone(MeasuredSweepSelector(self.grid,shifted,plan))
        self.assertIsNotNone(MeasuredSweepSelector(self.grid,self.map,self.plan))

    def test_forged_frame_digest_rejected(self):
        forged=replace(self.plan,raster_geometry_digest="0"*64)
        with self.assertRaises(SweepAdaptationError):
            MeasuredSweepSelector(self.grid,self.map,forged)

    def test_identical_map_restart_digest_stable(self):
        copy=replace(self.map)
        replan=adapt_upstream_plan(self.grid,copy,
                                   (copy.centre(9*20+8),))
        self.assertEqual(self.plan.plan_digest,replan.plan_digest)
        self.assertIsNotNone(MeasuredSweepSelector(self.grid,copy,replan))


if __name__=="__main__":unittest.main()
