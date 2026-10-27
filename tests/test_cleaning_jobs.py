"""Deterministic, hardware-free regression coverage for the phase-0 RFC prototype."""
from __future__ import annotations

import json
from pathlib import Path
import random
import tempfile
import unittest

from oomwoo_cleaning_jobs import CleaningJobs, Grid, JobError, MapError, Resources, UnreachableArea
from oomwoo_cleaning_jobs.demo import run_demo


def grid(width=6, height=5, walls=(), no_go=(), clearance=0, rooms=None):
    occ = [0] * (width * height)
    for i in walls:
        occ[i] = 100
    return Grid(width=width, height=height, occupancy=tuple(occ),
                keepout=frozenset(no_go), clearance_cells=clearance,
                rooms=rooms or {'left': tuple(y*width+x for y in range(height) for x in range(width//2)),
                                'right': tuple(y*width+x for y in range(height) for x in range(width//2,width))})


class MapTests(unittest.TestCase):
    def test_unknown_and_walls_are_unsafe(self):
        g = Grid(3, 2, (0, -1, 100, 0, 0, 0))
        self.assertEqual(g.traversable, {0, 3, 4, 5})
        with self.assertRaises(MapError): g.reachable(1)

    def test_reject_corrupt_data(self):
        for occupancy in [(0,), (-2, 0), (101, 0), (0.1, 0)]:
            with self.subTest(occupancy=occupancy), self.assertRaises(MapError):
                Grid(2, 1, occupancy)
        for idx in (-1, 12, 1.2):
            with self.subTest(idx=idx), self.assertRaises(MapError):
                Grid(3, 3, (0,) * 9, keepout=frozenset({idx}))
        with self.assertRaises(MapError): Grid(1, 1, (0,), resolution_m=0)
        with self.assertRaises(MapError): Grid(1, 1, (0,), clearance_cells=-1)

    def test_keepout_blocks_path_and_goals(self):
        g = grid(width=5, height=3, walls=(6,), no_go=(7,))
        safe=g.traversable
        self.assertNotIn(7, safe)
        path=g.path_to_nearest(5,frozenset({9}))
        self.assertEqual(path[0], 5)
        self.assertEqual(path[-1], 9)
        self.assertTrue(set(path) <= safe)
        self.assertNotIn(7, path)
        for a,b in zip(path,path[1:]):
            self.assertEqual(abs(a%5-b%5)+abs(a//5-b//5), 1)

    def test_no_corner_cut(self):
        g = grid(width=2,height=2,walls=(1,2))
        with self.assertRaises(UnreachableArea):g.path_to_nearest(0,frozenset({3}))

    def test_clearance_inflates_occupancy_keepout_and_boundary(self):
        g = grid(width=7,height=7,no_go=(3*7+3,),clearance=1)
        for y in range(2,5):
            for x in range(2,5):self.assertIn(y*7+x,g.blocked)
        self.assertIn(0,g.blocked)
        self.assertIn(6,g.blocked)
        self.assertIn(6*7+6,g.blocked)
        self.assertIn(1*7+1,g.traversable)

    def test_rooms_spot_and_whole(self):
        g = grid(width=6,height=3,no_go=(0,))
        self.assertEqual(len(g.targets(mode='whole')),17)
        self.assertTrue(all(i%6 < 3 for i in g.targets(mode='rooms',rooms=('left',))))
        self.assertEqual(g.targets(mode='spot',spot=8,radius_m=0),{8})
        self.assertEqual(len(g.targets(mode='spot',spot=8,radius_m=g.resolution_m)),5)
        with self.assertRaises(MapError):g.targets(mode='rooms',rooms=('kitchen',))
        with self.assertRaises(MapError):g.targets(mode='rooms',rooms=('left','left'))
        with self.assertRaises(MapError):g.targets(mode='spot',spot=0,radius_m=0)
        with self.assertRaises(MapError):g.targets(mode='whole',rooms=('left',))
        with self.assertRaises(MapError):g.targets(mode='spots')

    def test_fingerprint_tracks_safety_relevant_changes(self):
        a=grid(width=5,height=4)
        self.assertNotEqual(a.fingerprint(),grid(width=5,height=4,no_go=(0,)).fingerprint())
        self.assertNotEqual(a.fingerprint(),grid(width=5,height=4,clearance=1).fingerprint())
        self.assertNotEqual(a.fingerprint(),grid(width=5,height=4,walls=(1,)).fingerprint())

    def test_reachability_detects_split(self):
        g=grid(width=5,height=3,walls=(2,7,12))
        with self.assertRaises(UnreachableArea):g.path_to_nearest(5,frozenset({9}))
        self.assertEqual(g.reachable(5),{0,1,5,6,10,11})


class ResourceTests(unittest.TestCase):
    def test_priority(self):
        self.assertEqual(Resources(.1,True,True).needed(),'LOW_BATTERY')
        self.assertEqual(Resources(.8,True,True).needed(),'BIN_FULL')
        self.assertEqual(Resources(.8,False,True).needed(),'MOP_SERVICE')
        self.assertIsNone(Resources(.8,False,False).needed())
        self.assertEqual(Resources(.2).needed(),'LOW_BATTERY')

    def test_invalid_resources(self):
        for fraction in (-1, 2, float('nan'),float('inf')):
            with self.subTest(fraction=fraction), self.assertRaises(JobError):
                Resources(fraction).needed()
        with self.assertRaises(JobError):Resources(.5,bin_full=1).needed()


class JobTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path=Path(self.temp.name)/'checkpoints'/'job.json'
        self.map=grid()
        self.job=CleaningJobs(self.map,self.path)

    def launch(self, **kwargs):
        return self.job.start('case_01',mode='whole',start_cell=0,**kwargs)

    def test_start_and_full_completion(self):
        started=self.launch()
        self.assertEqual(started['state'],'running')
        self.assertEqual(started['target_cells'],30)
        current=0
        for _ in range(30):
            route=self.job.waypoint(current)
            self.assertTrue(set(route) <= self.map.traversable)
            current=route[-1]
            self.job.acknowledge_cleaned((current,))
        end=self.job.status()
        self.assertEqual(end['state'],'completed')
        self.assertEqual(end['coverage_fraction'],1)
        self.assertEqual(end['cleaned_cells'],end['target_cells'])
        with self.assertRaises(JobError):self.job.waypoint(current)
        with self.assertRaises(JobError):self.job.resume(pose_cell=current,resources=Resources(.8))

    def test_never_mark_unscoped_or_no_go(self):
        self.map=grid(width=4,height=3,no_go=(1,),rooms={'A':(0,2,4,5)})
        self.job=CleaningJobs(self.map,self.path)
        self.job.start('rooms',mode='rooms',rooms=('A',),start_cell=0)
        before=self.path.read_bytes()
        with self.assertRaises(JobError):self.job.acknowledge_cleaned((1,))
        self.assertEqual(self.path.read_bytes(),before)
        with self.assertRaises(JobError):self.job.acknowledge_cleaned((3,))
        self.assertEqual(self.path.read_bytes(),before)
        with self.assertRaises(JobError):self.job.acknowledge_cleaned(())
        with self.assertRaises(JobError):self.job.acknowledge_cleaned((True,))

    def test_unreachable_target_rejected_before_checkpoint(self):
        self.map=grid(width=5,height=3,walls=(2,7,12))
        self.job=CleaningJobs(self.map,self.path)
        with self.assertRaises(UnreachableArea):self.job.start('bad',mode='whole',start_cell=5)
        self.assertFalse(self.path.exists())

    def test_pause_and_resume_from_fresh_pose(self):
        self.launch()
        self.job.acknowledge_cleaned((0,1))
        s=self.job.pause()
        self.assertEqual(s['reason_code'],'USER_PAUSE')
        self.assertEqual(self.job.pause()['state'],'paused_user')
        with self.assertRaises(JobError):self.job.waypoint(1)
        with self.assertRaises(JobError):self.job.acknowledge_cleaned((2,))
        with self.assertRaises(MapError):self.job.resume(pose_cell=-1,resources=Resources(.8))
        resumed=self.job.resume(pose_cell=1,resources=Resources(.8))
        self.assertEqual(resumed['cleaned_cells'],2)
        self.assertEqual(resumed['resume_count'],1)
        self.assertEqual(resumed['state'],'running')

    def test_pause_low_battery_then_bin_then_mop(self):
        self.launch()
        low=Resources(.1,True,True)
        s=self.job.observe_resources(low)
        self.assertEqual(s['reason_code'],'LOW_BATTERY')
        with self.assertRaises(JobError):self.job.resume(pose_cell=0,resources=low)
        with self.assertRaises(JobError):self.job.resume(pose_cell=0,resources=Resources(.8,True,False))
        with self.assertRaises(JobError):self.job.resume(pose_cell=0,resources=Resources(.8,False,True))
        self.job.resume(pose_cell=0,resources=Resources(.8))
        self.assertEqual(self.job.status()['state'],'running')

    def test_start_low_battery_fails_to_arm(self):
        s=self.launch(resources=Resources(.1))
        self.assertEqual(s['state'],'paused_service')
        with self.assertRaises(JobError):self.job.waypoint(0)

    def test_simulated_crash_requires_manual_resume_and_keeps_progress(self):
        self.launch()
        self.job.acknowledge_cleaned((0,1,2))
        restarted=CleaningJobs(self.map,self.path)
        s=restarted.status()
        self.assertEqual(s['state'],'paused_recovery')
        self.assertEqual(s['reason_code'],'RESTART_NEEDS_REVALIDATION')
        self.assertEqual(s['cleaned_cells'],3)
        with self.assertRaises(JobError):restarted.waypoint(2)
        restarted.resume(pose_cell=2,resources=Resources(.9))
        self.assertEqual(restarted.status()['state'],'running')
        self.assertEqual(restarted.status()['cleaned_cells'],3)

    def test_paused_job_stays_paused_across_restarts(self):
        self.launch()
        self.job.pause()
        restarted=CleaningJobs(self.map,self.path)
        self.assertEqual(restarted.status()['state'],'paused_user')

    def test_map_mutation_fails_closed(self):
        self.launch()
        with self.assertRaisesRegex(JobError,'map/keepout'):
            CleaningJobs(grid(no_go=(4,)),self.path)

    def test_corrupt_checkpoint_fails_closed(self):
        self.path.parent.mkdir(parents=True,exist_ok=True)
        self.path.write_text('{bad JSON')
        with self.assertRaises(JobError):CleaningJobs(self.map,self.path)
        self.path.write_text('{}')
        with self.assertRaises(JobError):CleaningJobs(self.map,self.path)

    def test_start_invalid_inputs(self):
        for name in ('', 'spaces are bad', '../traversal', 'x'*70):
            with self.subTest(name=name),self.assertRaises(JobError):
                self.job.start(name,mode='whole',start_cell=0)
        with self.assertRaises(MapError):self.job.start('valid',mode='whole',start_cell=500)
        self.assertFalse(self.path.exists())

    def test_idempotent_feedback(self):
        self.launch()
        self.job.acknowledge_cleaned((0,0,1))
        self.job.acknowledge_cleaned((1,))
        self.assertEqual(self.job.status()['cleaned_cells'],2)
        self.assertEqual(self.job.status()['coverage_fraction'],2/30)

    def test_cancel_irreversible(self):
        self.launch()
        s=self.job.cancel()
        self.assertEqual(s['state'],'cancelled')
        self.assertEqual(self.job.cancel()['state'],'cancelled')
        with self.assertRaises(JobError):self.job.resume(pose_cell=0,resources=Resources(.8))
        with self.assertRaises(JobError):self.job.acknowledge_cleaned((0,))
        with self.assertRaises(JobError):self.job.start('newjob',mode='whole',start_cell=0)

    def test_failure_irreversible(self):
        self.launch()
        with self.assertRaises(JobError):self.job.fail('x')
        self.assertEqual(self.job.fail('NAVIGATION_LOST')['state'],'error')
        with self.assertRaises(JobError):self.job.resume(pose_cell=0,resources=Resources(.8))
        with self.assertRaises(JobError):self.job.cancel()

    def test_concurrent_revision_refused(self):
        self.launch()
        second=CleaningJobs(self.map,self.path)
        self.assertEqual(second.status()['state'],'paused_recovery')
        with self.assertRaises(JobError):self.job.acknowledge_cleaned((0,))

    def test_rooms_partial_scope_completes_only_room(self):
        self.job.start('room',mode='rooms',rooms=('left',),start_cell=0)
        target=set(self.job.state['targets'])
        self.assertEqual(len(target),15)
        self.job.acknowledge_cleaned(tuple(sorted(target)))
        self.assertEqual(self.job.status()['state'],'completed')

    def test_spot_only_centre_completes(self):
        self.job.start('spot',mode='spot',spot=7,radius_m=0,start_cell=7)
        self.job.acknowledge_cleaned((7,))
        self.assertEqual(self.job.status()['target_cells'],1)
        self.assertEqual(self.job.status()['state'],'completed')

    def test_repeated_resource_event_does_not_bump_revision(self):
        self.launch()
        a=self.job.observe_resources(Resources(.1))
        b=self.job.observe_resources(Resources(.1))
        self.assertEqual(a['revision'],b['revision'])

    def test_crash_recovery_does_not_rearm_until_recheck(self):
        self.launch()
        self.job.acknowledge_cleaned((0,))
        j=CleaningJobs(self.map,self.path)
        with self.assertRaises(JobError):j.resume(pose_cell=0,resources=Resources(.05))
        self.assertEqual(j.status()['state'],'paused_recovery')
        self.assertEqual(j.status()['cleaned_cells'],1)

    def test_safety_never_issue_goal_on_pause(self):
        self.launch()
        self.job.observe_resources(Resources(.10))
        with self.assertRaises(JobError):self.job.waypoint(0)

    def test_map_objects_immutable_from_caller(self):
        room_data={'room':[0,1,2]}
        occ=[0]*9
        keepout={8}
        g=Grid(3,3,occ,rooms=room_data,keepout=keepout)
        fp=g.fingerprint()
        room_data['room'].append(3)
        occ[0]=100
        keepout.add(7)
        self.assertEqual(g.fingerprint(),fp)
        self.assertEqual(g.rooms['room'],(0,1,2))

    def test_corrupted_scope_fails_even_if_valid_subset(self):
        self.launch()
        saved=json.loads(self.path.read_text())
        saved['targets'].remove(0)
        self.path.write_text(json.dumps(saved))
        with self.assertRaises(JobError):CleaningJobs(self.map,self.path)

    def test_paused_service_checkpoint_survives_reopen(self):
        self.launch()
        self.job.acknowledge_cleaned((0,))
        self.job.observe_resources(Resources(.80,bin_full=True))
        resumed=CleaningJobs(self.map,self.path)
        self.assertEqual(resumed.status()['state'],'paused_service')
        self.assertEqual(resumed.status()['cleaned_cells'],1)
        with self.assertRaises(JobError):resumed.resume(pose_cell=0,resources=Resources(.8,bin_full=True))
        resumed.resume(pose_cell=0,resources=Resources(.8))
        self.assertEqual(resumed.status()['cleaned_cells'],1)

    def test_randomized_route_invariants_and_full_work(self):
        rng=random.Random(20261027)
        for case in range(20):
            # Random keepouts but restrict job to initial reachable component.
            cells=frozenset(i for i in range(9*6) if rng.random()<.10)
            g=grid(width=9,height=6,no_go=cells)
            starts=sorted(g.traversable)
            if not starts:continue
            start=starts[0]
            comp=g.reachable(start)
            # Rooms intentionally encode only the reachable component.
            g=Grid(9,6,g.occupancy,rooms={'target':tuple(sorted(comp))},keepout=cells)
            path=Path(self.temp.name)/f'{case}.json'
            j=CleaningJobs(g,path)
            j.start(f'random_{case}',mode='rooms',rooms=('target',),start_cell=start)
            current=start
            for _ in range(len(comp)):
                route=j.waypoint(current)
                self.assertTrue(set(route)<=g.traversable)
                self.assertTrue(all(abs(a%9-b%9)+abs(a//9-b//9)==1 for a,b in zip(route,route[1:])))
                current=route[-1]
                j.acknowledge_cleaned((current,))
            self.assertEqual(j.status()['state'],'completed')
            self.assertEqual(j.status()['cleaned_cells'],len(comp))

    def test_demo(self):
        done=run_demo()
        self.assertTrue(done['all_targets_completed'])
        self.assertTrue(done['no_go_excluded'])
        self.assertEqual(done['resume_count'],1)


if __name__=='__main__':
    unittest.main()
