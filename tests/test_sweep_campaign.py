"""Large durable sweep campaign regression, including intentional recovery faults."""
from __future__ import annotations
from pathlib import Path
import tempfile
import unittest
from oomwoo_cleaning_jobs import Grid
from oomwoo_cleaning_jobs.map_admission import Raster
from oomwoo_cleaning_jobs.upstream_sweep_adapter import adapt_upstream_plan
from oomwoo_cleaning_jobs.sweep_campaign import SweepCampaign,SweepStateError


class DurableSweepCampaignTests(unittest.TestCase):
    def setUp(self):
        self.scratch=tempfile.TemporaryDirectory()
        self.addCleanup(self.scratch.cleanup)
        self.p=Path(self.scratch.name)/"session.json"
        self.raw=(0,)*16*16
        self.grid=Grid(16,16,self.raw,resolution_m=.1,clearance_cells=1,
             rooms={"Salón":tuple((r*16+c) for r in range(2,8) for c in range(2,9)),
                    "Cocina":tuple((r*16+c) for r in range(9,14) for c in range(9,14))})
        self.map=Raster(16,16,.1,-.8,-.8,0.,self.raw)
        points=tuple(self.map.centre(i) for i in (3*16+3,4*16+5,10*16+10,11*16+11))
        self.plan=adapt_upstream_plan(self.grid,self.map,points)
        self.room=frozenset(self.grid.rooms["Salón"])
        self.clean=Raster(16,16,.1,-.8,-.8,0.,self.raw)
        self.campaign=SweepCampaign(self.grid,self.map,self.plan,self.p,
                scope=self.room,session_id="room_salon")

    def test_room_scope_exact_and_persistent(self):
        self.campaign.start()
        self.assertEqual(self.campaign.summary().eligible_cells,len(self.room))
        self.assertTrue(self.p.is_file())
        loaded=SweepCampaign(self.grid,self.map,self.plan,self.p,
                scope=self.room,session_id="room_salon")
        self.assertEqual(loaded.summary().state,"paused_recovery")
        self.assertEqual(loaded.summary().verified_covered_cells,0)

    def test_external_measured_updates_persist(self):
        self.campaign.start()
        self.campaign.apply_external_coverage((3*16+3,4*16+4))
        self.assertEqual(self.campaign.summary().verified_covered_cells,2)
        loaded=SweepCampaign(self.grid,self.map,self.plan,self.p,
                scope=self.room,session_id="room_salon")
        self.assertEqual(loaded.summary().verified_covered_cells,2)

    def test_next_target_requires_running(self):
        with self.assertRaises(SweepStateError):
            self.campaign.next_candidate(self.clean)

    def test_stale_plan_digest_refused(self):
        self.campaign.start()
        alternate=adapt_upstream_plan(self.grid,self.map,(self.map.centre(4*16+4),))
        with self.assertRaises(SweepStateError):
            SweepCampaign(self.grid,self.map,alternate,self.p,
                scope=self.room,session_id="room_salon")

    def test_changed_keepout_rejects_restore(self):
        self.campaign.start()
        changed=Grid(16,16,self.raw,resolution_m=.1,clearance_cells=1,
              keepout=frozenset((2*16+2,)))
        other=adapt_upstream_plan(changed,self.map,
                                  (self.map.centre(3*16+3),))
        with self.assertRaises(SweepStateError):
            SweepCampaign(changed,self.map,other,self.p,
                          scope=frozenset(changed.traversable),
                          session_id="room_salon")

    def test_no_concurrent_goal_reservations(self):
        self.campaign.start()
        choice=self.campaign.next_candidate(self.clean)
        self.assertIsNotNone(choice)
        self.campaign.reserve_goal(choice,"sim-goal-1")
        with self.assertRaises(SweepStateError):
            self.campaign.next_candidate(self.clean)
        with self.assertRaises(SweepStateError):
            self.campaign.reserve_goal(choice,"sim-goal-2")
        self.campaign.on_terminal("sim-goal-1","succeeded")
        self.assertIsNotNone(self.campaign.next_candidate(self.clean))

    def test_goal_completion_without_meter_does_not_credit(self):
        self.campaign.start()
        choice=self.campaign.next_candidate(self.clean)
        self.campaign.reserve_goal(choice,"sim-goal-1")
        self.campaign.on_terminal("sim-goal-1","succeeded")
        self.assertEqual(self.campaign.summary().verified_covered_cells,0)

    def test_retry_budget_exhaustion_skips_goal_not_full_scope(self):
        self.campaign.start()
        first=self.campaign.next_candidate(self.clean)
        for attempt in range(3):
            self.campaign.reserve_goal(first,"try-"+str(attempt))
            self.campaign.on_terminal("try-"+str(attempt),"succeeded")
        new=self.campaign.next_candidate(self.clean)
        self.assertNotEqual(first.target_cell,new.target_cell)
        self.assertEqual(self.campaign.summary().verified_covered_cells,0)

    def test_crashed_active_goal_reopens_disarmed(self):
        self.campaign.start()
        candidate=self.campaign.next_candidate(self.clean)
        self.campaign.reserve_goal(candidate,"known-uuid-goal")
        restored=SweepCampaign(self.grid,self.map,self.plan,self.p,
                scope=self.room,session_id="room_salon")
        self.assertEqual(restored.summary().state,"paused_recovery")
        with self.assertRaises(SweepStateError):
            restored.resumed_after_verified_idle(idle_verified=True)
        with self.assertRaises(SweepStateError):
            restored.reconcile_old_goal(token="wrong",terminal_confirmed=True)
        with self.assertRaises(SweepStateError):
            restored.reconcile_old_goal(token="known-uuid-goal",terminal_confirmed=False)
        restored.reconcile_old_goal(token="known-uuid-goal",terminal_confirmed=True)
        with self.assertRaises(SweepStateError):
            restored.resumed_after_verified_idle(idle_verified=False)
        restored.resumed_after_verified_idle(idle_verified=True)
        self.assertEqual(restored.summary().state,"running")
        self.assertEqual(restored.summary().verified_covered_cells,0)

    def test_wrong_measured_room_is_rejected(self):
        self.campaign.start()
        outside=10*16+10
        with self.assertRaises(SweepStateError):
            self.campaign.apply_external_coverage((outside,))
        self.assertEqual(self.campaign.summary().verified_covered_cells,0)

    def test_out_of_order_terminal_cannot_release_wrong_goal(self):
        self.campaign.start()
        a=self.campaign.next_candidate(self.clean)
        self.campaign.reserve_goal(a,"goal-a")
        with self.assertRaises(SweepStateError):
            self.campaign.on_terminal("goal-b","succeeded")
        self.assertEqual(self.campaign.state["active_token"],"goal-a")

    def test_service_interrupt_and_restart_preserves_count(self):
        self.campaign.start()
        self.campaign.apply_external_coverage((3*16+3,))
        self.campaign.pause(reason="LOW_BATTERY")
        self.campaign.record_service()
        self.assertEqual(self.campaign.summary().service_cycles,1)
        restored=SweepCampaign(self.grid,self.map,self.plan,self.p,
                scope=self.room,session_id="room_salon")
        self.assertEqual(restored.summary().service_cycles,1)
        self.assertEqual(restored.summary().verified_covered_cells,1)
        restored.resumed_after_verified_idle(idle_verified=True)
        self.assertEqual(restored.summary().state,"running")

    def test_corrupted_json_refuses_rearm(self):
        self.p.write_text('{"unparseable"')
        with self.assertRaises(SweepStateError):
            SweepCampaign(self.grid,self.map,self.plan,self.p,
                          scope=self.room,session_id="room_salon")

    def test_two_writers_cannot_mutate_old_revision(self):
        clone=SweepCampaign(self.grid,self.map,self.plan,self.p,
                scope=self.room,session_id="room_salon")
        self.campaign.start()
        with self.assertRaises(SweepStateError):
            clone.start()

    def test_full_externally_verified_scope_completes_without_redundant_motion(self):
        self.campaign.start()
        self.campaign.apply_external_coverage(tuple(sorted(self.room)))
        status=self.campaign.finalize(self.clean)
        self.assertEqual(status.state,"completed")
        self.assertEqual(status.measured_fraction,1.)
        self.assertEqual(status.remaining_cells,0)

    def test_meter_values_alone_without_credited_deltas_do_not_complete(self):
        self.campaign.start()
        meter=Raster(16,16,.1,-.8,-.8,0.,(100,)*256)
        status=self.campaign.finalize(meter)
        self.assertEqual(status.state,"manual_review")
        self.assertEqual(status.measured_fraction,0.)
        self.assertGreater(status.remaining_cells,0)

    def test_fully_meter_confirmed_room_completes_after_terminal_cancel(self):
        self.campaign.start()
        choice=self.campaign.next_candidate(self.clean)
        self.campaign.reserve_goal(choice,"nav-terminal-1")
        # A real external meter finished the scope before Nav2 canceled.
        self.campaign.apply_external_coverage(tuple(sorted(self.room)))
        self.campaign.on_terminal("nav-terminal-1","canceled")
        self.assertEqual(self.campaign.summary().state,"completed")
        self.assertEqual(self.campaign.summary().remaining_cells,0)

    def test_cancel_is_durable_and_irreversible(self):
        self.campaign.start()
        self.campaign.cancel()
        self.assertEqual(self.campaign.summary().state,"cancelled")
        with self.assertRaises(SweepStateError):
            self.campaign.resumed_after_verified_idle(idle_verified=True)

if __name__=="__main__":
    unittest.main()
