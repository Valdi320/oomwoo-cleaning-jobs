"""End-to-end synthetic (NOT Gazebo) three-room multi-service tests."""
import unittest
from oomwoo_cleaning_jobs.synthetic_home import (
    run_home,make_home,ScenarioFailure,DiskMeter)
from oomwoo_cleaning_jobs.multiroom_program import RoomProgramError
from oomwoo_cleaning_jobs.sim_coverage import SimCoverageBridge


class IntegratedSyntheticHomeTests(unittest.TestCase):
    def test_all_three_rooms_with_three_services_and_crash(self):
        r=run_home(seed=0)
        self.assertTrue(r.completed)
        self.assertEqual(r.covered,r.eligible)
        self.assertEqual(r.dock_cycles,3)
        self.assertEqual(r.crash_restarts,1)
        self.assertEqual(r.keepout_entries,0)
        self.assertGreater(r.goals,50)

    def test_only_cocina_never_expands_into_other_rooms(self):
        r=run_home(seed=2,selected_rooms=("Cocina",),
                   service_interruptions=False,crash_once=False)
        self.assertTrue(r.completed)
        self.assertEqual(r.rooms,("Cocina",))
        self.assertEqual(r.covered,r.eligible)
        self.assertEqual(r.dock_cycles,0)
        self.assertEqual(r.crash_restarts,0)
        self.assertLess(r.eligible,300)

    def test_empty_selection_must_not_default_to_whole_house(self):
        with self.assertRaises(RoomProgramError):
            run_home(seed=0,selected_rooms=(),max_goals=1)

    def test_missing_meter_progress_does_not_appear_spontaneously(self):
        grid,raster,plan=make_home(0)
        meter=DiskMeter(raster)
        bridge=SimCoverageBridge(raster,grid)
        first=meter.scan()
        data=bridge.consume(first,stamp_s=meter.time,
             current_sim_s=meter.time,simulation_unstable=False,
             active=False,nav_goal_accepted=False,
             allowed_targets=grid.traversable)
        self.assertEqual(data.new_cells,())
        unchanged=meter.scan()
        delta=bridge.consume(unchanged,stamp_s=meter.time,
             current_sim_s=meter.time,simulation_unstable=False,
             active=True,nav_goal_accepted=True,
             allowed_targets=grid.traversable)
        self.assertEqual(delta.new_cells,())

    def test_explicit_bounded_goal_budget(self):
        with self.assertRaises(ScenarioFailure):
            run_home(seed=0,max_goals=2)

if __name__=="__main__":
    unittest.main()
