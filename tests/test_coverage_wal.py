"""Durable external-coverage write-ahead log crash / replay regression matrix.

No ROS2 or physical sensor. Callers supply already vetted coverage events.
The tests inject process-crash boundaries after each fsynced projection.
"""
from __future__ import annotations
import json
import tempfile
from pathlib import Path
import unittest

from oomwoo_cleaning_jobs import Grid, CleaningJobs, Resources
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.upstream_sweep_adapter import adapt_upstream_plan
from oomwoo_cleaning_jobs.sweep_campaign import SweepCampaign,SweepStateError
from oomwoo_cleaning_jobs.coverage_wal import CoverageWAL,CoverageLedgerError


class CoverageWALTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        d=Path(self.tmp.name)
        self.grid=Grid(8,8,(0,)*64,resolution_m=.1)
        self.raster=Raster(8,8,.1,-.4,-.4,0.,(0,)*64)
        self.scope=self.grid.traversable
        points=(self.raster.centre(18),self.raster.centre(26))
        self.plan=adapt_upstream_plan(self.grid,self.raster,points)
        self.job_path=d/"job.json"
        self.campaign_path=d/"campaign.json"
        self.wal_path=d/"coverage.wal"
        self.job=CleaningJobs(self.grid,self.job_path)
        self.job.start("meter_bridge",mode="whole",start_cell=18,resources=Resources(.91))
        self.campaign=SweepCampaign(self.grid,self.raster,self.plan,
            self.campaign_path,scope=self.scope,session_id="meter_bridge")
        self.campaign.start()
        self.wal=CoverageWAL(self.wal_path,grid=self.grid,scope=self.scope,
                             producer="validated_oomwoo_meter",job_id="meter_bridge")

    def stage(self,event="frame0001",cells=(18,19)):
        self.wal.stage(event,cells,externally_validated=True)

    def test_fsync_before_projection_and_both_views_agree(self):
        self.stage()
        self.assertEqual(self.wal.pending(),("frame0001",))
        self.assertEqual(self.job.status()["cleaned_cells"],0)
        self.assertEqual(self.campaign.summary().verified_covered_cells,0)
        self.assertTrue(self.wal.project("frame0001",self.job,self.campaign))
        self.assertEqual(self.job.status()["cleaned_cells"],2)
        self.assertEqual(self.campaign.summary().verified_covered_cells,2)
        self.assertFalse(self.wal.pending())
        self.wal.require_drained()

    def test_crash_after_job_projection_replays_into_campaign(self):
        self.stage()
        with self.assertRaisesRegex(CoverageLedgerError,"AFTER JOB"):
            self.wal.project("frame0001",self.job,self.campaign,
                             simulate_crash_after_job=True)
        self.assertEqual(self.job.status()["cleaned_cells"],2)
        self.assertEqual(self.campaign.summary().verified_covered_cells,0)
        self.assertEqual(self.wal.pending(),("frame0001",))
        old_revision=self.job.status()["revision"]
        self.assertTrue(self.wal.project("frame0001",self.job,self.campaign))
        self.assertEqual(self.job.status()["revision"],old_revision)
        self.assertEqual(self.campaign.summary().verified_covered_cells,2)
        self.assertEqual(self.wal.pending(),())

    def test_crash_after_campaign_projection_replay_does_not_duplicate_either(self):
        self.stage()
        with self.assertRaisesRegex(CoverageLedgerError,"AFTER CAMPAIGN"):
            self.wal.project("frame0001",self.job,self.campaign,
                             simulate_crash_after_campaign=True)
        before=(self.job.status()["revision"],
                self.campaign.summary().revision)
        self.assertTrue(self.wal.project("frame0001",self.job,self.campaign))
        after=(self.job.status()["revision"],
               self.campaign.summary().revision)
        self.assertEqual(before,after)
        self.assertFalse(self.wal.pending())

    def test_second_application_is_idempotent(self):
        self.stage()
        self.wal.project("frame0001",self.job,self.campaign)
        self.assertFalse(self.wal.project("frame0001",self.job,self.campaign))
        self.assertEqual(self.job.status()["cleaned_cells"],2)

    def test_duplicate_frame_id_new_payload_rejected(self):
        self.stage()
        before=self.wal_path.read_bytes()
        with self.assertRaises(CoverageLedgerError):
            self.wal.stage("frame0001",(18,20),externally_validated=True)
        self.assertEqual(self.wal_path.read_bytes(),before)

    def test_duplicate_exact_frame_is_idempotent(self):
        self.stage()
        before=self.wal_path.read_bytes()
        self.stage()
        self.assertEqual(self.wal_path.read_bytes(),before)

    def test_non_external_measurement_never_staged(self):
        with self.assertRaises(CoverageLedgerError):
            self.wal.stage("invented",(18,),externally_validated=False)
        self.assertEqual(self.wal.pending(),())

    def test_outside_scope_never_persisted(self):
        with self.assertRaises(CoverageLedgerError):
            self.wal.stage("bad",(-1,),externally_validated=True)
        self.assertEqual(self.wal.pending(),())

    def test_invalid_event_id_rejected(self):
        for event in ("","foo/bar"," bad ",None):
            with self.subTest(event=event),self.assertRaises(CoverageLedgerError):
                self.wal.stage(event,(18,),externally_validated=True)

    def test_unrecognized_frame_cannot_be_projected(self):
        with self.assertRaises(CoverageLedgerError):
            self.wal.project("unknown",self.job,self.campaign)

    def test_mutated_source_map_scope_cannot_reopen(self):
        changed=Grid(8,8,(0,)*64,resolution_m=.1,keepout=frozenset((5,)))
        with self.assertRaises(CoverageLedgerError):
            CoverageWAL(self.wal_path,grid=changed,scope=changed.traversable,
                        producer="validated_oomwoo_meter",job_id="meter_bridge")

    def test_wrong_producer_on_reopen_refused(self):
        with self.assertRaises(CoverageLedgerError):
            CoverageWAL(self.wal_path,grid=self.grid,scope=self.scope,
                        producer="other_meter",job_id="meter_bridge")

    def test_wal_corrupted_or_missing_fails_closed(self):
        self.wal_path.write_text("{")
        with self.assertRaises(CoverageLedgerError):
            CoverageWAL(self.wal_path,grid=self.grid,scope=self.scope,
                        producer="validated_oomwoo_meter",job_id="meter_bridge")

    def test_all_receipts_required_before_finalization(self):
        self.stage()
        with self.assertRaises(CoverageLedgerError):
            self.wal.require_drained()
        self.wal.project("frame0001",self.job,self.campaign)
        self.wal.require_drained()

    def test_pending_replay_after_process_restart_demands_revalidation(self):
        self.stage()
        with self.assertRaises(CoverageLedgerError):
            self.wal.project("frame0001",self.job,self.campaign,
                             simulate_crash_after_job=True)
        reloaded_job=CleaningJobs(self.grid,self.job_path)
        reloaded_campaign=SweepCampaign(self.grid,self.raster,self.plan,
            self.campaign_path,scope=self.scope,session_id="meter_bridge")
        reloaded_wal=CoverageWAL(self.wal_path,grid=self.grid,scope=self.scope,
                            producer="validated_oomwoo_meter",job_id="meter_bridge")
        self.assertEqual(reloaded_job.status()["state"],"paused_recovery")
        self.assertEqual(reloaded_campaign.summary().state,"paused_recovery")
        with self.assertRaises(CoverageLedgerError):
            reloaded_wal.project("frame0001",reloaded_job,reloaded_campaign)
        # Actual ROS2 new-process rescue is tested separately; this is only
        # the pure Python admission and replay after an explicitly verified idle.
        reloaded_job.resume(pose_cell=18,resources=Resources(.89))
        reloaded_campaign.resumed_after_verified_idle(idle_verified=True)
        self.assertTrue(reloaded_wal.project("frame0001",reloaded_job,reloaded_campaign))
        self.assertEqual(reloaded_job.status()["cleaned_cells"],2)
        self.assertEqual(reloaded_campaign.summary().verified_covered_cells,2)

    def test_journal_event_limit_blocks_growth(self):
        small=CoverageWAL(Path(self.tmp.name)/"small.wal",
            grid=self.grid,scope=self.scope,producer="validated_oomwoo_meter",
            job_id="meter_bridge",max_events=1)
        small.stage("frame1",(18,),externally_validated=True)
        with self.assertRaises(CoverageLedgerError):
            small.stage("frame2",(19,),externally_validated=True)

    def test_incomplete_wal_not_claimed_complete(self):
        self.stage(cells=(18,))
        self.assertEqual(self.campaign.summary().measured_fraction,0.)
        self.assertEqual(self.job.status()["coverage_fraction"],0.)
        with self.assertRaises(CoverageLedgerError):
            self.wal.require_drained()

if __name__=="__main__":
    unittest.main()
