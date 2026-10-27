"""One authoritative room campaign, fsynced external meter and source sweep.

All meter events here are EXPLICIT SYNTHETIC fixtures. Actual Nav2/Gazebo
acceptance is a separate integration test. Never infer coverage from terminal
Nav2 responses.
"""
from pathlib import Path
import json,tempfile,unittest

from oomwoo_cleaning_jobs import Grid,CleaningJobs,Resources
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.room_bridge import (
    PersistentRoomBridge,imported_offline_plan,RoomBridgeError)
from oomwoo_cleaning_jobs.upstream_sweep_adapter import adapt_upstream_plan


class PersistedRoomBridgeTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root=Path(self.tmp.name)
        width,height=28,20
        occupancy=(0,)*(width*height)
        self.base=Grid(width,height,occupancy,resolution_m=.05,
                       clearance_cells=4)
        room=tuple(sorted(i for i in self.base.traversable
                if 10<=i%width<=17 and 7<=i//width<=11))
        other=tuple(sorted(self.base.traversable-frozenset(room)))
        self.grid=Grid(width,height,occupancy,resolution_m=.05,
                 clearance_cells=4,rooms={"Zona digital":room,"Resto":other})
        self.map=Raster(width,height,.05,-.7,-.5,0.,occupancy)
        centers=(8*width+12,10*width+14,8*width+16)
        self.plan=adapt_upstream_plan(self.grid,self.map,
                        tuple(self.map.centre(i) for i in centers))
        self.root=root
        self.room=room
        self.job=CleaningJobs(self.grid,root/"job.json")
        self.job.start("room_trial",mode="rooms",
                       rooms=("Zona digital",),start_cell=8*width+8,
                       resources=Resources(.95))
        self.bridge=PersistentRoomBridge(self.grid,self.map,self.plan,
                         root/"campaign",job_id="room_trial",
                         rooms=("Zona digital",))
        self.bridge.start()
        self.blank=Raster(width,height,.05,-.7,-.5,0.,occupancy)

    def test_upstream_waypoint_is_selected_with_exclusive_action_token(self):
        choice=self.bridge.choose(self.blank,8*28+8,lambda _:True)
        self.assertEqual(choice.kind,"upstream")
        self.assertEqual(choice.room,"Zona digital")
        self.bridge.after_dispatch(choice,"route-1")
        with self.assertRaises(RoomBridgeError):
            self.bridge.choose(self.blank,8*28+8,lambda _:True)
        self.bridge.after_terminal("route-1","succeeded")
        self.assertEqual(self.bridge.program.campaign.summary().state,"running")
        self.assertEqual(self.bridge.program.campaign.summary().verified_covered_cells,0)

    def test_measurements_written_before_checkpoint_projection(self):
        cells=tuple(self.room[:4])
        self.bridge.accept_meter_delta(frame_id="meter_1_0",cells=cells,
                           job=self.job,verified_active_goal=True)
        self.assertEqual(self.bridge.wal.pending(),("meter_1_0",))
        self.job.acknowledge_cleaned(cells)
        self.bridge.finish_meter_projection("meter_1_0",self.job)
        self.assertEqual(self.bridge.wal.pending(),())
        self.assertEqual(self.job.status()["cleaned_cells"],4)
        self.assertEqual(self.bridge.status()["verified_cells"],4)

    def test_unaccepted_navigation_never_credits(self):
        with self.assertRaises(RoomBridgeError):
            self.bridge.accept_meter_delta(frame_id="meter_1_0",
                 cells=(self.room[0],),job=self.job,verified_active_goal=False)
        self.assertEqual(self.job.status()["cleaned_cells"],0)

    def test_outside_room_cell_is_rejected(self):
        wrong=next(iter(self.grid.rooms["Resto"]))
        with self.assertRaises(RoomBridgeError):
            self.bridge.accept_meter_delta(frame_id="meter_1_0",
                 cells=(wrong,),job=self.job,verified_active_goal=True)

    def test_external_100_percent_then_terminal_cancel_is_real_completion(self):
        first=self.bridge.choose(self.blank,8*28+8,lambda _:True)
        self.bridge.after_dispatch(first,"route-complete")
        cells=tuple(self.room)
        self.bridge.accept_meter_delta(frame_id="meter_full",cells=cells,
                 job=self.job,verified_active_goal=True)
        self.job.acknowledge_cleaned(cells)
        self.bridge.finish_meter_projection("meter_full",self.job)
        self.assertEqual(self.job.status()["state"],"completed")
        self.bridge.after_terminal("route-complete","canceled")
        self.assertEqual(self.bridge.status()["state"],"completed")
        self.assertEqual(self.bridge.status()["coverage_fraction"],1.)
        self.assertEqual(self.bridge.status()["pending_coverage_events"],0)

    def test_measured_repair_completes_after_nav2_terminal_cancel(self):
        from oomwoo_cleaning_jobs.room_bridge import CampaignRoute
        route=(8*28+8,8*28+9,8*28+10)
        choice=CampaignRoute(route,"repair",None,"Zona digital",route[-1])
        self.bridge.after_dispatch(choice,"repair-canceled-after-full-meter")
        cells=tuple(self.room)
        self.bridge.accept_meter_delta(frame_id="real_meter_full_repair",
            cells=cells,job=self.job,verified_active_goal=True)
        self.job.acknowledge_cleaned(cells)
        self.bridge.finish_meter_projection("real_meter_full_repair",self.job)
        self.assertEqual(self.job.status()["state"],"completed")
        self.bridge.after_terminal("repair-canceled-after-full-meter","canceled")
        self.assertEqual(self.bridge.status()["state"],"completed")
        self.assertEqual(self.bridge.status()["verified_cells"],len(self.room))

    def test_restart_blocks_moving_with_unresolved_route(self):
        chosen=self.bridge.choose(self.blank,8*28+8,lambda _:True)
        self.bridge.after_dispatch(chosen,"crash-goal")
        again=PersistentRoomBridge(self.grid,self.map,self.plan,
                   self.root/"campaign",job_id="room_trial",rooms=("Zona digital",))
        self.assertEqual(again.status()["state"],"paused_recovery")

    def test_source_map_digest_not_arbitrary_json(self):
        file=self.root/"offline.json"
        doc={"never_spun_planner":True,"no_nav2_goal_sent":True,
             "frame":"map","source_map_size":[28,20],
             "resolution":.05,"source_map_digest":self.base.fingerprint(),
             "target_poses":[list(self.map.centre(i))
                              for i in (8*28+12,10*28+14)]}
        file.write_text(json.dumps(doc))
        safe=imported_offline_plan(self.grid,self.map,file)
        self.assertEqual(safe.originally_requested,2)
        doc["source_map_digest"]="f"*64
        file.write_text(json.dumps(doc))
        with self.assertRaises(RoomBridgeError):
            imported_offline_plan(self.grid,self.map,file)

if __name__=="__main__":
    unittest.main()
