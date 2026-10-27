"""Headless, synthetic complete-room job workflow across battery pause / restart.

Verifies the cleaning-jobs state-machine, persisted room labels and no-go
scope; it does NOT claim any Gazebo movement or physically measured cleaning.
The external coverage events here are deliberate test fixtures.
"""
from __future__ import annotations

from pathlib import Path
import tempfile
import unittest

from oomwoo_cleaning_jobs import CleaningJobs, Grid, JobError, Resources
from oomwoo_cleaning_jobs.nav_interlock import NavigationInterlock
from oomwoo_cleaning_jobs.rooms import segment_rooms

from test_rooms import two_rooms


class StubNav2Port:
    def __init__(self):
        self.goals=[]
        self.cancel_requests=[]
    def send_goal(self, goal):
        self.goals.append(goal)
    def request_cancel(self, token):
        self.cancel_requests.append(token)


class RoomJobFlowTests(unittest.TestCase):
    def setUp(self):
        self.scratch=tempfile.TemporaryDirectory(prefix="oomwoo-room-flow-")
        self.addCleanup(self.scratch.cleanup)
        self.checkpoint=Path(self.scratch.name)/"room_job.json"
        self.grid=two_rooms(keepout=frozenset((5*28+5,)))
        self.plan=segment_rooms(self.grid).renamed("room_001","Salón")
        self.plan=self.plan.renamed("room_002","Cocina")
        self.labelled=self.plan.grid_with_rooms(self.grid)
        self.pose=4*28+4
        self.targets=self.labelled.targets(mode="rooms",rooms=("Salón",))

    def _new_gate(self, job):
        port=StubNav2Port()
        gate=NavigationInterlock(job,port,coverage_source="SYNTHETIC_COVERAGE_FIXTURE")
        return gate,port

    def test_finish_selected_room_after_battery_pause_and_process_restart(self):
        job=CleaningJobs(self.labelled,self.checkpoint)
        first=job.start("room_salon",mode="rooms",rooms=("Salón",),
                        start_cell=self.pose,resources=Resources(.9))
        self.assertEqual(first["target_cells"],len(self.targets))
        self.assertEqual(first["cleaned_cells"],0)
        self.assertTrue(self.targets.isdisjoint(self.labelled.rooms["Cocina"]))
        gate,port=self._new_gate(job)
        gate.confirm_backend_idle(verified_no_outstanding_goals=True)
        action=gate.dispatch(pose_cell=self.pose,admit_route=lambda route:True)
        halfway=len(self.targets)//2
        first_half=tuple(sorted(self.targets)[:halfway])
        rest=tuple(sorted(self.targets)[halfway:])
        progress=gate.acknowledge_coverage(
            token=action.token,cells=first_half,
            producer="SYNTHETIC_COVERAGE_FIXTURE",
            map_digest=self.labelled.fingerprint())
        self.assertEqual(progress["cleaned_cells"],halfway)
        self.assertLess(progress["coverage_fraction"],1.)
        gate.observe_resources(Resources(.05))
        self.assertEqual(job.status()["reason_code"],"LOW_BATTERY")
        self.assertEqual(port.cancel_requests,[action.token])
        gate.on_goal_result(token=action.token,outcome="canceled")
        self.assertEqual(job.status()["state"],"paused_service")
        # Simulated *controller process restart*. Durable progress persists,
        # but new interlock STARTS DISARMED: a separate exclusive simulated
        # Nav2 backend-idle check is needed to resume.
        reloaded=CleaningJobs(self.labelled,self.checkpoint)
        self.assertEqual(reloaded.status()["cleaned_cells"],halfway)
        new_gate,new_port=self._new_gate(reloaded)
        with self.assertRaises(JobError):
            new_gate.resume(pose_cell=self.pose,resources=Resources(.9))
        new_gate.confirm_backend_idle(verified_no_outstanding_goals=True)
        with self.assertRaises(JobError):
            new_gate.resume(pose_cell=self.pose,resources=Resources(.04))
        self.assertEqual(
            new_gate.resume(pose_cell=self.pose,resources=Resources(.8))["state"],"running")
        other_goal=new_gate.dispatch(pose_cell=self.pose,admit_route=lambda route:True)
        end=new_gate.acknowledge_coverage(
            token=other_goal.token,cells=rest,
            producer="SYNTHETIC_COVERAGE_FIXTURE",
            map_digest=self.labelled.fingerprint())
        self.assertEqual(end["state"],"completed")
        self.assertEqual(end["cleaned_cells"],len(self.targets))
        self.assertEqual(end["coverage_fraction"],1.)
        self.assertEqual(new_port.cancel_requests,[other_goal.token])
        new_gate.on_goal_result(token=other_goal.token,outcome="canceled")
        self.assertFalse(new_gate.can_dispatch())
        # Tests may not mark any no-go nor the *other* room as cleaned.
        stored=set(reloaded.state["cleaned"])
        self.assertEqual(stored,set(self.targets))
        self.assertFalse(stored & set(self.grid.keepout))
        self.assertFalse(stored & set(self.labelled.rooms["Cocina"]))

    def test_room_layout_changes_invalidate_checkpoint(self):
        job=CleaningJobs(self.labelled,self.checkpoint)
        job.start("room_test",mode="rooms",rooms=("Salón",),
                  start_cell=self.pose,resources=Resources(.85))
        switched=self.plan.renamed("Salón","Dormitorio")
        with self.assertRaises(JobError):
            CleaningJobs(switched.grid_with_rooms(self.grid),self.checkpoint)

    def test_nonselected_room_coverage_never_acknowledged(self):
        job=CleaningJobs(self.labelled,self.checkpoint)
        job.start("roomscope",mode="rooms",rooms=("Salón",),start_cell=self.pose)
        gate,_=self._new_gate(job)
        gate.confirm_backend_idle(verified_no_outstanding_goals=True)
        token=gate.dispatch(pose_cell=self.pose,admit_route=lambda route:True).token
        before=self.checkpoint.read_bytes()
        wrong=next(iter(self.labelled.rooms["Cocina"]))
        with self.assertRaises(JobError):
            gate.acknowledge_coverage(token=token,cells=(wrong,),
                producer="SYNTHETIC_COVERAGE_FIXTURE",
                map_digest=self.labelled.fingerprint())
        self.assertEqual(self.checkpoint.read_bytes(),before)

    def test_recovery_refuses_map_redefined_keepout(self):
        job=CleaningJobs(self.labelled,self.checkpoint)
        job.start("roomscope",mode="rooms",rooms=("Salón",),start_cell=self.pose)
        changed=two_rooms(keepout=frozenset((5*28+5,7*28+7)))
        with self.assertRaises(Exception):
            self.plan.grid_with_rooms(changed)
        self.assertEqual(self.checkpoint.exists(),True)


    def test_virtual_polygon_masks_scope_before_room_job(self):
        from oomwoo_cleaning_jobs.map_admission import Raster
        from oomwoo_cleaning_jobs.zones import ZoneShape,ZonePlan
        base=two_rooms()
        raster=Raster(base.width,base.height,base.resolution_m,0.,0.,0.,base.occupancy)
        excluded=5*28+5
        x,y=raster.centre(excluded)
        polygon=ZoneShape("No pasar","polygon",
            ((x-.021,y-.021),(x+.021,y-.021),
             (x+.021,y+.021),(x-.021,y+.021)))
        plan=ZonePlan.create(raster,base,(polygon,))
        zoned=plan.grid_with_keepouts(raster,base)
        self.assertIn(excluded,zoned.keepout)
        rooms=segment_rooms(zoned).renamed("room_001","Salón")
        labelled=rooms.grid_with_rooms(zoned)
        chosen=labelled.targets(mode="rooms",rooms=("Salón",))
        self.assertNotIn(excluded,chosen)
        self.assertEqual(chosen,set(labelled.rooms["Salón"]))
        path=Path(self.scratch.name)/"no_go_room.json"
        jobs=CleaningJobs(labelled,path)
        started=jobs.start("polygon_room",mode="rooms",rooms=("Salón",),
                           start_cell=self.pose,resources=Resources(.9))
        self.assertEqual(started["target_cells"],len(chosen))
        self.assertEqual(started["cleaned_cells"],0)


if __name__=="__main__":
    unittest.main()
