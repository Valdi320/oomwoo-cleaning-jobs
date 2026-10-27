"""Integrated independent measured-coverage WAL + multi-room + station tests.

The coverage events here are synthetic, not real Nav2/Gazebo. Every
pre-acknowledged receipt is fsync'd, replayable and checked against
BOTH the job-state file and route campaign, without fake cleaning.
"""
from __future__ import annotations
from pathlib import Path
import tempfile
import unittest

from oomwoo_cleaning_jobs import Grid,CleaningJobs,Resources
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.upstream_sweep_adapter import adapt_upstream_plan
from oomwoo_cleaning_jobs.multiroom_program import MultiRoomProgram
from oomwoo_cleaning_jobs.coverage_wal import CoverageWAL,CoverageLedgerError
from oomwoo_cleaning_jobs.sweep_campaign import SweepStateError


class IntegratedMultiLedgerTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        root=Path(self.tmp.name)
        width,height=12,8
        free=(0,)*(width*height)
        keepout=frozenset((3*width+5,4*width+5))
        rooms={"Salón":tuple(i for i in range(len(free))
                             if i%width<6 and i not in keepout),
               "Cocina":tuple(i for i in range(len(free))
                              if i%width>=6 and i not in keepout)}
        self.grid=Grid(width,height,free,keepout=keepout,rooms=rooms,
                       resolution_m=.1)
        self.raster=Raster(width,height,.1,-.6,-.4,0.,free)
        poses=tuple(self.raster.centre(i)
                    for i in (2*width+2,4*width+2,4*width+9))
        self.plan=adapt_upstream_plan(self.grid,self.raster,poses)
        self.root=root
        self.program=MultiRoomProgram(self.grid,self.raster,self.plan,
                                      root/"program",program_id="integrated",
                                      selected_rooms=("Salón","Cocina"))
        self.job=CleaningJobs(self.grid,root/"job.json")
        self.job.start("integrated",mode="rooms",
                       rooms=("Salón","Cocina"),start_cell=1,
                       resources=Resources(.9))
        self.program.campaign.start()
        self.wal=CoverageWAL(root/"coverage.wal",grid=self.grid,
                             scope=self.program.scope,
                             producer="verified_sim_meter",job_id="integrated")

    def project(self,event,cells,**failure):
        self.wal.stage(event,tuple(sorted(cells)),externally_validated=True)
        return self.wal.project(event,self.job,self.program.campaign,**failure)

    def test_one_job_two_rooms_three_dock_services_and_no_double_credit(self):
        salon=tuple(sorted(self.grid.rooms["Salón"]))
        cocina=tuple(sorted(self.grid.rooms["Cocina"]))
        self.project("room-salon",salon)
        self.assertEqual(self.program.active_room(),"Cocina")
        self.assertEqual(self.job.status()["cleaned_cells"],len(salon))
        self.program.station_interrupt(battery_low=True,bin_full=True,
                 mop_dirty=True,goal_terminal=True)
        self.job.observe_resources(Resources(.05))
        dock=self.program.dock
        dock.confirm_dock_arrival(docked=True,nav2_terminal=True,
                                   station_connected=True)
        dock.start_service(station_ack_nonce=dock.state["nonce"],
                           station_ready=True)
        for observation in range(1,3):
            dock.observe(battery_fraction=.92,bin_full=False,mop_dirty=False,
                station_completed=tuple(dock.state["needed"]),
                station_ack_nonce=dock.state["nonce"],docked=True,
                observation_seq=observation)
        self.program.continue_after_service(nav2_idle_verified=True,
                        fresh_pose=True,resources_healthy=True)
        self.job.resume(pose_cell=1,resources=Resources(.92))
        self.project("room-cocina",cocina)
        self.assertEqual(self.job.status()["state"],"completed")
        self.assertEqual(self.program.finish_if_measured(self.raster).state,"completed")
        self.assertEqual(self.program.campaign.summary().verified_covered_cells,len(self.program.scope))
        self.assertEqual(self.program.dock.state["cycles"],1)
        self.assertEqual(len(self.grid.keepout & set(self.program.campaign.state["covered"])),0)
        self.assertFalse(self.wal.project("room-cocina",self.job,self.program.campaign))
        self.wal.require_drained()

    def test_after_first_checkpoint_crash_both_restore_after_verified_idle(self):
        salon=tuple(sorted(self.grid.rooms["Salón"]))
        with self.assertRaisesRegex(CoverageLedgerError,"AFTER JOB"):
            self.project("first-room",salon,simulate_crash_after_job=True)
        self.assertEqual(self.job.status()["cleaned_cells"],len(salon))
        self.assertEqual(self.program.campaign.summary().verified_covered_cells,0)
        # New process requires BOTH ledgers paused and explicit backend proof.
        re_job=CleaningJobs(self.grid,self.root/"job.json")
        re_program=MultiRoomProgram(self.grid,self.raster,self.plan,
                                    self.root/"program",program_id="integrated",
                                    selected_rooms=("Salón","Cocina"))
        re_wal=CoverageWAL(self.root/"coverage.wal",grid=self.grid,
                           scope=re_program.scope,producer="verified_sim_meter",
                           job_id="integrated")
        with self.assertRaises(CoverageLedgerError):
            re_wal.project("first-room",re_job,re_program.campaign)
        self.assertEqual(re_program.campaign.summary().state,"paused_recovery")
        re_job.resume(pose_cell=1,resources=Resources(.92))
        re_program.campaign.resumed_after_verified_idle(idle_verified=True)
        self.assertTrue(re_wal.project("first-room",re_job,re_program.campaign))
        self.assertEqual(re_program.active_room(),"Cocina")
        self.assertFalse(re_wal.pending())
        self.assertEqual(re_job.status()["cleaned_cells"],len(salon))

    def test_invalid_nonselected_room_event_not_written(self):
        restricted=MultiRoomProgram(self.grid,self.raster,self.plan,
                          self.root/"only_salon",program_id="only_salon",
                          selected_rooms=("Salón",))
        restricted.campaign.start()
        wal=CoverageWAL(self.root/"only_salon.wal",grid=self.grid,
                        scope=restricted.scope,producer="verified_sim_meter",
                        job_id="only_salon")
        with self.assertRaises(CoverageLedgerError):
            wal.stage("outside",(next(iter(self.grid.rooms["Cocina"])),),
                      externally_validated=True)
        self.assertFalse(wal.pending())

if __name__=="__main__":
    unittest.main()
