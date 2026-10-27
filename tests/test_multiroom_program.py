"""Full digital room scheduling / service interruptions / scope provenance tests.

These use SYNTHETIC externally acknowledged floor cells, NOT real Gazebo
movement or real battery/vacuum/station hardware.
"""
from pathlib import Path
import tempfile
import unittest

from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.upstream_sweep_adapter import adapt_upstream_plan
from oomwoo_cleaning_jobs.multiroom_program import MultiRoomProgram,RoomProgramError
from oomwoo_cleaning_jobs.dock_cycle import DockCycleError
from oomwoo_cleaning_jobs.sweep_campaign import SweepStateError


class WholeHomeProgramTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        w,h=26,15
        occ=[0]*(w*h)
        keepout=frozenset((6*w+12,7*w+12))
        rooms={
           "Salón":tuple(i for i in range(w*h)
                          if i%w<=12 and i not in keepout),
           "Cocina":tuple(i for i in range(w*h)
                           if i%w>12 and i not in keepout)}
        self.grid=Grid(w,h,tuple(occ),keepout=keepout,
                       rooms=rooms,resolution_m=.1,clearance_cells=0)
        # Grid.rooms must include only safe indices in these tests.
        self.rooms={n:frozenset(i for i in indices if i in self.grid.traversable)
                    for n,indices in rooms.items()}
        self.grid=Grid(w,h,tuple(occ),keepout=keepout,
                       rooms={n:tuple(sorted(v)) for n,v in self.rooms.items()},
                       resolution_m=.1)
        self.map=Raster(w,h,.1,-1.3,-.75,0.,tuple(occ))
        points=tuple(self.map.centre(i) for i in
            (4*w+4,4*w+10,4*w+18,4*w+22,9*w+20))
        self.plan=adapt_upstream_plan(self.grid,self.map,points)
        self.home=Path(self.tmp.name)/"home"
        self.p=MultiRoomProgram(self.grid,self.map,self.plan,self.home,
             program_id="apt_demo",selected_rooms=("Salón","Cocina"))
        self.meter=Raster(w,h,.1,-1.3,-.75,0.,tuple(occ))

    def test_explicit_room_order_and_scope(self):
        summary=self.p.progress()
        self.assertEqual(tuple(x.name for x in summary),("Salón","Cocina"))
        self.assertEqual(self.p.active_room(),"Salón")
        self.assertEqual(sum(x.total for x in summary),len(self.p.scope))
        self.assertNotIn(6*26+12,self.p.scope)

    def test_only_first_room_goal_scheduled_until_verified(self):
        self.p.campaign.start()
        chosen=self.p.choose_next(self.meter)
        self.assertEqual(chosen[0],"Salón")
        self.assertIn(chosen[1].target_cell,self.rooms["Salón"])
        self.p.measured_delta(tuple(sorted(self.rooms["Salón"])))
        self.assertEqual(self.p.active_room(),"Cocina")
        next_room=self.p.choose_next(self.meter)
        self.assertEqual(next_room[0],"Cocina")

    def test_complete_two_rooms_only_with_measured_acknowledgements(self):
        self.p.campaign.start()
        self.p.measured_delta(tuple(sorted(self.rooms["Salón"])))
        self.p.measured_delta(tuple(sorted(self.rooms["Cocina"])))
        self.assertTrue(all(x.verified_complete for x in self.p.progress()))
        st=self.p.finish_if_measured(self.meter)
        self.assertEqual(st.state,"completed")
        self.assertEqual(st.measured_fraction,1.0)

    def test_one_room_selected_never_credits_other(self):
        single=MultiRoomProgram(self.grid,self.map,self.plan,self.home/"only",
             program_id="only_salon",selected_rooms=("Salón",))
        single.campaign.start()
        with self.assertRaises(SweepStateError):
            single.measured_delta((next(iter(self.rooms["Cocina"])),))
        self.assertEqual(single.campaign.summary().verified_covered_cells,0)

    def test_station_recharge_empty_mop_and_resume_two_room_job(self):
        self.p.campaign.start()
        self.p.measured_delta(tuple(sorted(self.rooms["Salón"])))
        self.p.station_interrupt(battery_low=True,bin_full=True,
                                 mop_dirty=True,goal_terminal=True)
        self.assertEqual(self.p.campaign.summary().state,"paused")
        with self.assertRaises(RoomProgramError):
            self.p.choose_next(self.meter)
        dock=self.p.dock
        self.assertEqual(set(dock.state["needed"]),{"recharge","empty_bin","wash_mop"})
        dock.confirm_dock_arrival(docked=True,nav2_terminal=True,
                                  station_connected=True)
        dock.start_service(station_ack_nonce=dock.state["nonce"],station_ready=True)
        for observation in range(1,3):
            dock.observe(battery_fraction=.93,bin_full=False,mop_dirty=False,
              station_completed=tuple(dock.state["needed"]),
              station_ack_nonce=dock.state["nonce"],docked=True,
                observation_seq=observation)
        self.p.continue_after_service(nav2_idle_verified=True,
                                      fresh_pose=True,resources_healthy=True)
        self.assertEqual(self.p.campaign.summary().service_cycles,1)
        self.assertEqual(self.p.active_room(),"Cocina")
        self.p.measured_delta(tuple(sorted(self.rooms["Cocina"])))
        self.assertEqual(self.p.finish_if_measured(self.meter).state,"completed")

    def test_service_cannot_start_with_active_nav2_goal(self):
        self.p.campaign.start()
        with self.assertRaises(RoomProgramError):
            self.p.station_interrupt(battery_low=True,bin_full=False,
                 mop_dirty=False,goal_terminal=False)
        self.assertEqual(self.p.campaign.summary().state,"running")

    def test_service_missing_telemetry_denies_resume(self):
        self.p.campaign.start()
        self.p.station_interrupt(battery_low=True,bin_full=False,
                                 mop_dirty=False,goal_terminal=True)
        with self.assertRaises(DockCycleError):
            self.p.continue_after_service(nav2_idle_verified=True,
                                          fresh_pose=True,resources_healthy=True)
        self.assertEqual(self.p.campaign.summary().state,"paused")

    def test_persisted_resume_and_room_selection(self):
        self.p.campaign.start()
        self.p.measured_delta(tuple(sorted(self.rooms["Salón"])))
        second=MultiRoomProgram(self.grid,self.map,self.plan,self.home,
           program_id="apt_demo",selected_rooms=("Salón","Cocina"))
        self.assertEqual(second.campaign.summary().state,"paused_recovery")
        self.assertEqual(second.active_room(),"Cocina")
        second.campaign.resumed_after_verified_idle(idle_verified=True)
        second.measured_delta(tuple(sorted(self.rooms["Cocina"])))
        self.assertEqual(second.finish_if_measured(self.meter).state,"completed")

    def test_room_order_change_refused_after_checkpoint(self):
        with self.assertRaises(RoomProgramError):
            MultiRoomProgram(self.grid,self.map,self.plan,self.home,
                program_id="apt_demo",selected_rooms=("Cocina","Salón"))

    def test_changed_map_or_keepout_refused_after_checkpoint(self):
        new=Grid(self.grid.width,self.grid.height,self.grid.occupancy,
                 keepout=frozenset(set(self.grid.keepout)|{0}),
                 rooms=self.grid.rooms,resolution_m=.1)
        nextplan=adapt_upstream_plan(new,self.map,
                             (self.map.centre(4*26+4),))
        with self.assertRaises(RoomProgramError):
            MultiRoomProgram(new,self.map,nextplan,self.home,
                 program_id="apt_demo",selected_rooms=("Salón","Cocina"))

    def test_gap_audit_when_measured_room_remains_partial(self):
        self.p.campaign.start()
        self.p.measured_delta((4*26+4,))
        audit=self.p.audit_room("Salón",pose_cell=4*26+4)
        self.assertTrue(audit.require_manual_review)
        self.assertGreater(audit.missing_cells,0)
        self.assertFalse(audit.verified_complete)

    def test_missing_proposal_in_room_never_auto_promotes_next_room(self):
        self.p.campaign.start()
        dirty=Raster(self.map.width,self.map.height,self.map.resolution,
                self.map.origin_x,self.map.origin_y,self.map.yaw,
                (100,)*len(self.map.data))
        # External raw meter may say covered; until validated + committed
        # into the campaign's durable credit ledger, no room is done.
        self.assertEqual(self.p.active_room(),"Salón")
        with self.assertRaises(RoomProgramError):
            self.p.choose_next(dirty)
        self.assertFalse(any(x.verified_complete for x in self.p.progress()))

if __name__=="__main__":
    unittest.main()
