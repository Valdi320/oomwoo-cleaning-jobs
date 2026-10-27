"""Regression tests: OOMWOO real coverage_meter 0/100 OccupancyGrid protocol.

Do not credit arbitrary navigation success, stale grids, preexisting floor,
or increments while a job is paused.
"""
from __future__ import annotations
from dataclasses import replace
import math
import unittest

from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.sim_coverage import SimCoverageBridge, CoverageError


class MeterBridgeTests(unittest.TestCase):
    def setUp(self):
        self.raw=tuple([0]*100)
        self.static=Raster(10,10,.1,-.5,-.5,0.,self.raw)
        self.grid=Grid(10,10,self.raw,resolution_m=.1,clearance_cells=1)
        self.scope=self.grid.traversable
        self.bridge=SimCoverageBridge(self.static,self.grid,max_age_s=2.)
        self.t=10.0

    def raster(self,cells=(),**kw):
        values=[0]*100
        for i in cells:values[i]=100
        v=dict(width=10,height=10,resolution=.1,
               origin_x=-.5,origin_y=-.5,yaw=0.,data=tuple(values))
        v.update(kw)
        return Raster(**v)

    def consume(self,cells=(),*,active=True,accepted=True,unstable=False,
                dt=1.0,age=0.0,**kw):
        self.t+=dt
        return self.bridge.consume(self.raster(cells,**kw),stamp_s=self.t,
                                    current_sim_s=self.t+age,
                                    simulation_unstable=unstable,
                                    active=active,nav_goal_accepted=accepted,
                                    allowed_targets=self.scope)

    def test_initial_measurement_is_baseline_not_credit(self):
        first=self.consume((22,23))
        self.assertEqual(first.state,"baseline")
        self.assertEqual(first.new_cells,())
        self.assertTrue(self.bridge.ready)

    def test_only_new_covered_cells_can_be_acknowledged(self):
        self.consume((22,))
        result=self.consume((22,23,24))
        self.assertEqual(result.new_cells,(23,24))
        self.assertEqual(self.consume((22,23,24)).new_cells,())
        self.assertEqual(self.bridge.credited,2)

    def test_only_job_scope_eligible(self):
        self.scope=frozenset({22})
        self.consume((23,))
        self.assertEqual(self.consume((22,23,24)).new_cells,(22,))
        self.assertEqual(self.bridge.credited,1)

    def test_rejects_outside_target_even_if_marked_ground_truth(self):
        self.scope=frozenset({22})
        self.consume(())
        self.assertEqual(self.consume((11,22,24,88)).new_cells,(22,))

    def test_clearance_fringe_may_be_swept_but_not_credited(self):
        self.consume(())
        data=self.consume((0,1,2,22))
        self.assertEqual(data.new_cells,(22,))
        self.assertEqual(data.unscoped_cells,3)

    def test_preexisting_before_start_not_counted(self):
        first=self.consume((22,23),active=False)
        self.assertEqual(first.state,"baseline")
        self.assertEqual(self.consume((22,23),active=True).new_cells,())

    def test_paused_growth_becomes_baseline_not_credit_after_resume(self):
        self.consume(())
        self.assertEqual(self.consume((22,),active=False).state,"not_active")
        self.assertEqual(self.consume((22,),active=True).new_cells,())
        self.assertEqual(self.consume((22,23),active=True).new_cells,(23,))

    def test_needs_real_nav2_goal_accepted_not_merely_sent(self):
        self.consume(())
        self.consume((22,),active=True,accepted=False)
        self.assertEqual(self.consume((22,),active=True,accepted=True).new_cells,())

    def test_explicit_unstable_freezes_measurement(self):
        self.consume(())
        with self.assertRaises(CoverageError):
            self.consume((22,),unstable=True)
        self.assertEqual(self.bridge.credited,0)

    def test_missing_boolean_stability_rejected(self):
        self.consume(())
        with self.assertRaises(CoverageError):
            self.bridge.consume(self.raster((22,)),stamp_s=11.,current_sim_s=11.,
              simulation_unstable=None,active=True,nav_goal_accepted=True,
              allowed_targets=self.scope)

    def test_stale_measurement_cannot_credit(self):
        self.consume(())
        with self.assertRaises(CoverageError):
            self.consume((22,),age=5.)
        self.assertEqual(self.bridge.credited,0)

    def test_future_dated_measurement_cannot_credit(self):
        self.consume(())
        with self.assertRaises(CoverageError):
            self.consume((22,),age=-5.)

    def test_out_of_order_timestamp_fails(self):
        self.consume(())
        with self.assertRaises(CoverageError):
            self.consume((22,),dt=-1.0)

    def test_equal_timestamp_changed_data_fails(self):
        self.consume(())
        with self.assertRaises(CoverageError):
            self.consume((22,),dt=0)

    def test_equal_stamp_same_map_ignored(self):
        self.consume((22,))
        self.assertEqual(self.consume((22,),dt=0).new_cells,())

    def test_coverage_bit_reversal_is_meter_reset(self):
        self.consume((22,23))
        with self.assertRaises(CoverageError):
            self.consume((22,))

    def test_invalid_marker_value_fails(self):
        self.consume(())
        with self.assertRaises(CoverageError):
            self.consume((22,),data=tuple([0]*22+[50]+[0]*77))

    def test_nonfree_map_cell_marked_fails(self):
        raw=list(self.raw);raw[33]=100
        obstacle_map=Raster(10,10,.1,-.5,-.5,0.,tuple(raw))
        grid=Grid(10,10,tuple(raw),resolution_m=.1,clearance_cells=1)
        bridge=SimCoverageBridge(obstacle_map,grid)
        with self.assertRaises(CoverageError):
            bridge.consume(self.raster((33,)),stamp_s=1.,current_sim_s=1.,
               simulation_unstable=False,active=False,nav_goal_accepted=False,
               allowed_targets=grid.traversable)

    def test_map_origin_mismatch(self):
        self.consume(())
        with self.assertRaises(CoverageError):
            self.consume((22,),origin_x=.7)

    def test_map_rotation_mismatch(self):
        self.consume(())
        with self.assertRaises(CoverageError):
            self.consume((22,),yaw=math.pi/2)

    def test_invalid_target_scope_rejected(self):
        self.consume(())
        with self.assertRaises(CoverageError):
            self.bridge.consume(self.raster((22,)),stamp_s=12.,current_sim_s=12.,
                simulation_unstable=False,active=True,nav_goal_accepted=True,
                allowed_targets=frozenset({0,22}))

    def test_repeated_new_coverage_not_double_counted_after_pause(self):
        self.consume(())
        self.assertEqual(self.consume((22,23)).new_cells,(22,23))
        self.consume((22,23,24),active=False)
        self.assertEqual(self.consume((22,23,24)).new_cells,())
        self.assertEqual(self.consume((22,23,24,25)).new_cells,(25,))
        self.assertEqual(self.bridge.credited,3)


if __name__=="__main__":
    unittest.main()
