"""Fault-injection tests for the Nav2 action handoff without a running ROS graph.

These exercise interlock invariants, NOT Gazebo coverage or real robot stops.
"""
from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from oomwoo_cleaning_jobs import CleaningJobs, Grid, JobError, MapError, Resources
from oomwoo_cleaning_jobs.nav_interlock import NavigationInterlock


class MockActionPort:
    def __init__(self):
        self.sent = []
        self.cancels = []
        self.fail_send = False
        self.fail_cancel = False

    def send_goal(self, goal):
        self.sent.append(goal)
        if self.fail_send:
            raise OSError('goal might have been accepted before TCP drop')

    def request_cancel(self, token):
        self.cancels.append(token)
        if self.fail_cancel:
            raise OSError('cancel request might be lost')


class NavInterlockTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.checkpoint = Path(self.temp.name) / 'checkpoint.json'
        self.grid = Grid(3, 2, (0, 0, 0, 0, 0, 0))
        self.jobs = CleaningJobs(self.grid, self.checkpoint)
        self.jobs.start('nav_test', mode='whole', start_cell=0)
        self.port = MockActionPort()
        self.now = [100.]
        self.gate = NavigationInterlock(self.jobs, self.port,
                                         coverage_source='verified_sweep',
                                         cancel_timeout_s=2.0, clock=lambda: self.now[0])

    def arm(self):
        self.gate.confirm_backend_idle(verified_no_outstanding_goals=True)

    def send(self):
        self.arm()
        return self.gate.dispatch(pose_cell=0, admit_route=lambda route: True)

    def evidence(self, token, cells=(0,)):
        return self.gate.acknowledge_coverage(
            token=token, cells=cells, producer='verified_sweep',
            map_digest=self.grid.fingerprint())

    def test_never_arm_implicitly_on_construction(self):
        self.assertFalse(self.gate.snapshot()['can_dispatch'])
        with self.assertRaises(JobError):
            self.gate.dispatch(pose_cell=0, admit_route=lambda r: True)
        self.assertFalse(self.port.sent)

    def test_backend_idle_must_be_explicitly_verified(self):
        for value in (False, None, 0, 'yes'):
            with self.subTest(value=value), self.assertRaises(JobError):
                self.gate.confirm_backend_idle(verified_no_outstanding_goals=value)
        self.assertFalse(self.gate.snapshot()['idle_reconciled'])

    def test_one_goal_at_a_time(self):
        goal = self.send()
        self.assertEqual(len(self.port.sent), 1)
        self.assertEqual(self.port.sent[0], goal)
        with self.assertRaises(JobError):
            self.gate.dispatch(pose_cell=0, admit_route=lambda r: True)
        self.assertEqual(len(self.port.sent), 1)

    def test_goal_has_matching_job_map_and_route(self):
        goal = self.send()
        self.assertEqual(goal.job_id, 'nav_test')
        self.assertEqual(goal.map_digest, self.grid.fingerprint())
        self.assertEqual(goal.candidate_cells[0], 0)
        self.assertEqual(goal.target_cell, goal.candidate_cells[-1])

    def test_route_admission_failure_latches_and_pauses(self):
        self.arm()
        with self.assertRaisesRegex(JobError, 'not admitted'):
            self.gate.dispatch(pose_cell=0, admit_route=lambda r: False)
        self.assertFalse(self.port.sent)
        self.assertEqual(self.jobs.status()['state'], 'paused_recovery')
        self.assertEqual(self.jobs.status()['reason_code'], 'ROUTE_NOT_ADMITTED')
        self.assertFalse(self.gate.can_dispatch())

    def test_admission_callback_exception_latches_and_pauses(self):
        self.arm()
        def broken(route):
            raise RuntimeError('lost costmap')
        with self.assertRaisesRegex(JobError, 'admission check failed'):
            self.gate.dispatch(pose_cell=0, admit_route=broken)
        self.assertEqual(self.jobs.status()['reason_code'], 'ROUTE_ADMISSION_ERROR')
        self.assertFalse(self.port.sent)

    def test_missing_admission_callback_does_not_launch(self):
        self.arm()
        with self.assertRaises(JobError):
            self.gate.dispatch(pose_cell=0, admit_route=None)
        self.assertFalse(self.port.sent)

    def test_successful_navigation_does_not_forge_cleaned_coverage(self):
        goal = self.send()
        self.assertTrue(self.gate.on_goal_result(token=goal.token, outcome='succeeded'))
        self.assertEqual(self.jobs.status()['cleaned_cells'], 0)
        self.assertTrue(self.gate.can_dispatch())

    def test_only_matching_trusted_map_scoped_evidence_can_mark_clean(self):
        goal = self.send()
        before = self.checkpoint.read_bytes()
        bad = [
            {'token': 'stale', 'producer': 'verified_sweep', 'map_digest': self.grid.fingerprint()},
            {'token': goal.token, 'producer': 'anyone', 'map_digest': self.grid.fingerprint()},
            {'token': goal.token, 'producer': 'verified_sweep', 'map_digest': 'fake'},
        ]
        for data in bad:
            with self.subTest(data=data), self.assertRaises(JobError):
                self.gate.acknowledge_coverage(cells=(0,), **data)
            self.assertEqual(before, self.checkpoint.read_bytes())
        self.assertEqual(self.evidence(goal.token)['cleaned_cells'], 1)

    def test_resume_never_creates_goal_without_explicit_dispatch(self):
        goal = self.send()
        self.gate.pause()
        self.gate.on_goal_result(token=goal.token, outcome='canceled')
        self.gate.resume(pose_cell=0, resources=Resources(.9))
        self.assertTrue(self.gate.can_dispatch())
        self.assertEqual(len(self.port.sent), 1)

    def test_pause_requests_cancel_but_does_not_assume_stopped(self):
        goal = self.send()
        self.gate.pause()
        self.assertEqual(self.port.cancels, [goal.token])
        self.assertEqual(self.jobs.status()['state'], 'paused_user')
        snap = self.gate.snapshot()
        self.assertTrue(snap['awaiting_terminal_result'])
        self.assertFalse(snap['can_dispatch'])
        with self.assertRaisesRegex(JobError, 'previous Nav2 action'):
            self.gate.resume(pose_cell=0, resources=Resources(.9))

    def test_late_success_after_cancel_does_not_count_coverage(self):
        goal = self.send()
        self.gate.pause()
        self.assertTrue(self.gate.on_goal_result(token=goal.token, outcome='succeeded'))
        self.assertEqual(self.jobs.status()['cleaned_cells'], 0)
        self.assertEqual(self.jobs.status()['state'], 'paused_user')
        with self.assertRaises(JobError):
            self.evidence(goal.token)

    def test_cancel_terminal_response_frees_barrier_not_resume(self):
        goal = self.send()
        self.gate.pause()
        self.assertTrue(self.gate.on_goal_result(token=goal.token, outcome='canceled'))
        self.assertEqual(self.gate.snapshot()['active_goal'], None)
        self.assertFalse(self.gate.can_dispatch())
        self.gate.resume(pose_cell=0, resources=Resources(.8))
        self.assertTrue(self.gate.can_dispatch())

    def test_duplicate_and_stale_result_do_not_release_wrong_goal(self):
        first = self.send()
        self.gate.on_goal_result(token=first.token, outcome='succeeded')
        second = self.gate.dispatch(pose_cell=1, admit_route=lambda r: True)
        self.assertNotEqual(first.token, second.token)
        self.assertFalse(self.gate.on_goal_result(token=first.token, outcome='canceled'))
        self.assertEqual(self.gate.snapshot()['active_goal'], second.token)
        self.assertEqual(self.jobs.status()['state'], 'running')
        self.assertFalse(self.gate.on_goal_result(token='fake', outcome='succeeded'))
        self.assertTrue(self.gate.on_goal_result(token=second.token, outcome='succeeded'))

    def test_goal_abort_requires_recovery_and_reconciliation(self):
        goal = self.send()
        self.gate.on_goal_result(token=goal.token, outcome='aborted')
        self.assertEqual(self.jobs.status()['reason_code'], 'NAV2_GOAL_ABORTED')
        with self.assertRaises(JobError):
            self.gate.resume(pose_cell=0, resources=Resources(.8))
        self.arm()
        self.gate.resume(pose_cell=0, resources=Resources(.8))
        self.assertEqual(self.jobs.status()['state'], 'running')

    def test_unexpected_canceled_result_latches(self):
        goal = self.send()
        self.gate.on_goal_result(token=goal.token, outcome='canceled')
        self.assertEqual(self.jobs.status()['state'], 'paused_recovery')

    def test_transport_send_failure_may_have_dispatched_so_requires_reconcile(self):
        self.arm()
        self.port.fail_send = True
        with self.assertRaisesRegex(JobError, 'acceptance uncertain'):
            self.gate.dispatch(pose_cell=0, admit_route=lambda r: True)
        self.assertEqual(self.jobs.status()['state'], 'paused_recovery')
        self.assertIsNotNone(self.gate.snapshot()['active_goal'])
        self.assertEqual(self.port.cancels, [self.port.sent[0].token])
        self.assertFalse(self.gate.can_dispatch())
        self.port.fail_send = False
        self.arm()
        self.gate.resume(pose_cell=0, resources=Resources(.8))
        self.assertTrue(self.gate.can_dispatch())

    def test_transport_cancel_failure_does_not_rearm(self):
        goal = self.send()
        self.port.fail_cancel = True
        with self.assertRaisesRegex(JobError, 'physically verify'):
            self.gate.pause()
        self.assertEqual(self.jobs.status()['state'], 'paused_user')
        self.assertTrue(self.gate.snapshot()['awaiting_terminal_result'])
        self.assertEqual(self.gate.snapshot()['interlock_fault'], 'NAV2_CANCEL_REQUEST_UNCERTAIN')
        self.gate.on_goal_result(token=goal.token, outcome='canceled')
        self.assertFalse(self.gate.snapshot()['can_dispatch'])
        self.port.fail_cancel = False
        self.arm()
        self.gate.resume(pose_cell=0, resources=Resources(.8))
        self.assertTrue(self.gate.can_dispatch())

    def test_battery_service_pause_cancels_outstanding_goal(self):
        goal = self.send()
        self.gate.observe_resources(Resources(.05))
        self.assertEqual(self.jobs.status()['reason_code'], 'LOW_BATTERY')
        self.assertEqual(self.port.cancels, [goal.token])
        with self.assertRaises(JobError):
            self.evidence(goal.token)
        self.gate.on_goal_result(token=goal.token, outcome='canceled')
        with self.assertRaisesRegex(JobError, 'LOW_BATTERY'):
            self.gate.resume(pose_cell=0, resources=Resources(.05))
        self.gate.resume(pose_cell=0, resources=Resources(.8))
        self.assertTrue(self.gate.can_dispatch())

    def test_timeout_is_latched_and_still_waits_for_external_stop(self):
        goal = self.send()
        self.gate.pause()
        self.now[0] = 101.9
        self.assertFalse(self.gate.check_timeout())
        self.now[0] = 102.1
        self.assertTrue(self.gate.check_timeout())
        self.assertEqual(self.gate.snapshot()['interlock_fault'], 'NAV2_CANCEL_TIMEOUT')
        self.assertTrue(self.gate.snapshot()['awaiting_terminal_result'])
        with self.assertRaises(JobError):
            self.gate.resume(pose_cell=0, resources=Resources(.8))
        self.gate.on_goal_result(token=goal.token, outcome='canceled')
        with self.assertRaises(JobError):
            self.gate.resume(pose_cell=0, resources=Resources(.8))
        self.arm()
        self.gate.resume(pose_cell=0, resources=Resources(.8))
        self.assertTrue(self.gate.can_dispatch())

    def test_safety_or_localization_event_suspends_and_cancels(self):
        goal = self.send()
        status = self.gate.pause_for_recovery('LOCALIZATION_LOST')
        self.assertEqual(status['reason_code'], 'LOCALIZATION_LOST')
        self.assertEqual(self.port.cancels, [goal.token])
        self.assertFalse(self.gate.can_dispatch())

    def test_cancel_job_cancels_action_and_never_resumes(self):
        goal = self.send()
        self.gate.cancel_job()
        self.assertEqual(self.jobs.status()['state'], 'cancelled')
        self.assertEqual(self.port.cancels, [goal.token])
        self.gate.on_goal_result(token=goal.token, outcome='canceled')
        self.arm()
        self.assertFalse(self.gate.can_dispatch())
        with self.assertRaises(JobError):
            self.gate.resume(pose_cell=0, resources=Resources(.8))

    def test_last_coverage_ack_cancels_goal_even_if_nav2_not_done(self):
        goal = self.send()
        all_cells=tuple(sorted(self.jobs.state['targets']))
        self.assertEqual(self.evidence(goal.token, all_cells)['state'], 'completed')
        self.assertEqual(self.port.cancels, [goal.token])
        self.assertFalse(self.gate.can_dispatch())
        self.assertTrue(self.gate.snapshot()['awaiting_terminal_result'])

    def test_resume_on_new_process_needs_nav_idle_and_fresh_pose(self):
        self.jobs.acknowledge_cleaned((0,))
        new_job = CleaningJobs(self.grid, self.checkpoint)
        self.assertEqual(new_job.status()['state'], 'paused_recovery')
        new_gate = NavigationInterlock(new_job, MockActionPort(), coverage_source='verified_sweep')
        with self.assertRaises(JobError):
            new_gate.resume(pose_cell=0, resources=Resources(.8))
        new_gate.confirm_backend_idle(verified_no_outstanding_goals=True)
        with self.assertRaises(MapError):
            new_gate.resume(pose_cell=200, resources=Resources(.8))
        self.assertEqual(new_gate.resume(pose_cell=0, resources=Resources(.8))['cleaned_cells'],1)

    def test_epoch_prevents_previous_process_callbacks_matching(self):
        self.arm()
        old_goal = self.gate.dispatch(pose_cell=0, admit_route=lambda r: True)
        restarted = CleaningJobs(self.grid, self.checkpoint)
        other = NavigationInterlock(restarted, MockActionPort(), coverage_source='verified_sweep')
        other.confirm_backend_idle(verified_no_outstanding_goals=True)
        other.resume(pose_cell=0, resources=Resources(.8))
        fresh = other.dispatch(pose_cell=0, admit_route=lambda r: True)
        self.assertNotEqual(fresh.token,old_goal.token)
        self.assertFalse(other.on_goal_result(token=old_goal.token,outcome='succeeded'))
        self.assertEqual(other.snapshot()['active_goal'], fresh.token)

    def test_invalid_terminal_response_rejected(self):
        goal=self.send()
        with self.assertRaises(ValueError):
            self.gate.on_goal_result(token=goal.token, outcome='pending')
        self.assertEqual(self.gate.snapshot()['active_goal'], goal.token)

    def test_duplicate_recovery_pause_does_not_reset_first_reason(self):
        self.jobs.suspend_recovery('ROBOT_STUCK')
        self.jobs.suspend_recovery('SECOND_FAULT')
        self.assertEqual(self.jobs.status()['reason_code'], 'ROBOT_STUCK')

    def test_invalid_recovery_pause_no_mutation(self):
        before=self.checkpoint.read_bytes()
        for reason in ('oops', '', 'BAD-FAULT','X', 100):
            with self.subTest(reason=reason), self.assertRaises(JobError):
                self.jobs.suspend_recovery(reason)
        self.assertEqual(before,self.checkpoint.read_bytes())

    def test_do_not_dispatch_when_resources_still_bad(self):
        self.gate.observe_resources(Resources(.01))
        self.arm()
        self.assertFalse(self.gate.can_dispatch())
        with self.assertRaises(JobError):
            self.gate.resume(pose_cell=0, resources=Resources(.01))

    def test_gate_rejects_invalid_configuration(self):
        for timeout in (0, -1, float('nan'), float('inf'), '2'):
            with self.subTest(timeout=timeout), self.assertRaises(ValueError):
                NavigationInterlock(self.jobs,self.port,coverage_source='x',cancel_timeout_s=timeout)
        with self.assertRaises(ValueError):
            NavigationInterlock(self.jobs,self.port,coverage_source=' ')

    def test_recovery_pause_not_overwrite_service_cause(self):
        self.gate.observe_resources(Resources(.01))
        with self.assertRaises(JobError):
            self.gate.pause_for_recovery('NAV2_ABORT')
        self.assertEqual(self.jobs.status()['reason_code'], 'LOW_BATTERY')

    def test_no_automatic_restart_after_concurrent_process(self):
        self.send()
        other=CleaningJobs(self.grid,self.checkpoint)  # A second writer invalidates revision.
        self.assertEqual(other.status()['reason_code'],'RESTART_NEEDS_REVALIDATION')
        with self.assertRaises(JobError):
            self.evidence(self.gate.snapshot()['active_goal'])
        self.assertEqual(other.status()['state'],'paused_recovery')


    def test_async_transport_fault_latches_and_attempts_cancel(self):
        goal=self.send()
        self.assertTrue(self.gate.on_transport_fault(
            token=goal.token,reason='NAV2_ACCEPTANCE_UNCERTAIN'))
        self.assertEqual(self.jobs.status()['state'],'paused_recovery')
        self.assertEqual(self.gate.snapshot()['interlock_fault'],'NAV2_ACCEPTANCE_UNCERTAIN')
        self.assertEqual(self.port.cancels,[goal.token])
        self.assertFalse(self.gate.can_dispatch())

    def test_old_async_fault_does_not_interrupt_current_goal(self):
        first=self.send()
        self.gate.on_goal_result(token=first.token,outcome='succeeded')
        second=self.gate.dispatch(pose_cell=0,admit_route=lambda r:True)
        self.assertFalse(self.gate.on_transport_fault(
            token=first.token,reason='NAV2_RESULT_UNCERTAIN'))
        self.assertEqual(self.jobs.status()['state'],'running')
        self.assertEqual(self.gate.snapshot()['active_goal'],second.token)

    def test_transport_fault_after_user_pause_preserves_user_pause(self):
        goal=self.send()
        self.gate.pause()
        self.assertTrue(self.gate.on_transport_fault(
            token=goal.token,reason='NAV2_CANCEL_NOT_ACCEPTED'))
        self.assertEqual(self.jobs.status()['reason_code'],'USER_PAUSE')
        self.assertEqual(self.gate.snapshot()['interlock_fault'],'NAV2_CANCEL_NOT_ACCEPTED')
        self.gate.on_goal_result(token=goal.token,outcome='canceled')
        with self.assertRaises(JobError):
            self.gate.resume(pose_cell=0,resources=Resources(.8))
        self.gate.confirm_backend_idle(verified_no_outstanding_goals=True)
        self.gate.resume(pose_cell=0,resources=Resources(.8))

    def test_invalid_async_fault_reason_rejected(self):
        goal=self.send()
        for reason in ('', 'LOCALIZATION_LOST', 'unscoped', None):
            with self.subTest(reason=reason),self.assertRaises(JobError):
                self.gate.on_transport_fault(token=goal.token,reason=reason)
        self.assertEqual(self.jobs.status()['state'],'running')
        self.assertEqual(self.port.cancels,[])

    def test_timeout_after_async_fault_remains_disarmed(self):
        goal=self.send()
        self.gate.on_transport_fault(token=goal.token,reason='NAV2_RESULT_UNCERTAIN')
        self.now[0]=102.1
        self.assertTrue(self.gate.check_timeout())
        self.gate.on_goal_result(token=goal.token,outcome='canceled')
        self.assertFalse(self.gate.can_dispatch())
        self.assertEqual(self.jobs.status()['state'],'paused_recovery')


    def test_safe_sweep_route_override_admitted(self):
        self.arm()
        goal=self.gate.dispatch(pose_cell=0, route_override=(0,1,2),
                                admit_route=lambda r:r==(0,1,2))
        self.assertEqual(goal.target_cell,2)
        self.assertEqual(goal.candidate_cells,(0,1,2))
        self.assertEqual(len(self.port.sent),1)

    def test_invalid_override_cannot_dispatch(self):
        self.arm()
        for route in ((0,2),(1,2),(0,200),(0,0),(),[0,1]):
            with self.subTest(route=route),self.assertRaises(JobError):
                self.gate.dispatch(pose_cell=0,route_override=route,
                                   admit_route=lambda r:True)
        self.assertEqual(len(self.port.sent),0)

    def test_overridden_route_still_respects_dynamic_costmap(self):
        self.arm()
        with self.assertRaises(JobError):
            self.gate.dispatch(pose_cell=0,route_override=(0,1,2),
                               admit_route=lambda r:False)
        self.assertEqual(self.jobs.status()['state'],'paused_recovery')
        self.assertEqual(len(self.port.sent),0)



    def test_measured_coverage_can_arrive_after_nav2_success(self):
        goal=self.send()
        self.assertTrue(self.gate.on_goal_result(token=goal.token,outcome='succeeded'))
        self.assertEqual(self.jobs.status()['cleaned_cells'],0)
        self.assertEqual(self.gate.eligible_coverage_tail_token(),goal.token)
        out=self.gate.acknowledge_post_goal_coverage(
            token=goal.token,cells=(0,),producer='verified_sweep',
            map_digest=self.grid.fingerprint())
        self.assertEqual(out['cleaned_cells'],1)

    def test_nav2_success_never_auto_credits_coverage(self):
        goal=self.send()
        self.gate.on_goal_result(token=goal.token,outcome='succeeded')
        self.assertEqual(self.jobs.status()['cleaned_cells'],0)
        self.assertEqual(self.checkpoint.exists(),True)
        self.assertEqual(self.gate.eligible_coverage_tail_token(),goal.token)

    def test_late_meter_sample_beyond_bounded_window_is_refused(self):
        goal=self.send()
        self.gate.on_goal_result(token=goal.token,outcome='succeeded')
        self.now[0]+=3.1
        with self.assertRaises(JobError):
            self.gate.acknowledge_post_goal_coverage(
                token=goal.token,cells=(0,),producer='verified_sweep',
                map_digest=self.grid.fingerprint())
        self.assertEqual(self.jobs.status()['cleaned_cells'],0)

    def test_pausing_clears_recent_success_coverage_window(self):
        goal=self.send()
        self.gate.on_goal_result(token=goal.token,outcome='succeeded')
        self.gate.pause()
        self.assertIsNone(self.gate.eligible_coverage_tail_token())
        with self.assertRaises(JobError):
            self.gate.acknowledge_post_goal_coverage(
                token=goal.token,cells=(0,),producer='verified_sweep',
                map_digest=self.grid.fingerprint())

    def test_canceling_clears_recent_success_coverage_window(self):
        goal=self.send()
        self.gate.on_goal_result(token=goal.token,outcome='succeeded')
        self.gate.cancel_job()
        self.assertIsNone(self.gate.eligible_coverage_tail_token())

    def test_next_dispatch_invalidates_previous_tail_token(self):
        first=self.send()
        self.gate.on_goal_result(token=first.token,outcome='succeeded')
        second=self.gate.dispatch(pose_cell=0,admit_route=lambda route:True)
        self.assertNotEqual(first.token,second.token)
        with self.assertRaises(JobError):
            self.gate.acknowledge_post_goal_coverage(
                token=first.token,cells=(0,),producer='verified_sweep',
                map_digest=self.grid.fingerprint())

    def test_wrong_source_or_bad_map_rejected_on_post_success(self):
        goal=self.send()
        self.gate.on_goal_result(token=goal.token,outcome='succeeded')
        for source, digest in (('spoof',self.grid.fingerprint()),('verified_sweep','bad')):
            with self.subTest(source=source,digest=digest),self.assertRaises(JobError):
                self.gate.acknowledge_post_goal_coverage(
                    token=goal.token,cells=(0,),producer=source,map_digest=digest)
        self.assertEqual(self.jobs.status()['cleaned_cells'],0)

    def test_canceled_or_aborted_goal_has_no_meter_tail(self):
        for result in ('canceled','aborted'):
            with self.subTest(result=result):
                from pathlib import Path
                import tempfile
                from oomwoo_cleaning_jobs.nav_interlock import NavigationInterlock
                with tempfile.TemporaryDirectory() as tmp:
                    jb=CleaningJobs(self.grid,Path(tmp)/'p.json')
                    jb.start('subtest',mode='whole',start_cell=0)
                    po=MockActionPort()
                    gate=NavigationInterlock(jb,po,coverage_source='verified_sweep',
                                             clock=lambda:self.now[0])
                    gate.confirm_backend_idle(verified_no_outstanding_goals=True)
                    goal=gate.dispatch(pose_cell=0,admit_route=lambda route:True)
                    gate.on_goal_result(token=goal.token,outcome=result)
                    self.assertIsNone(gate.eligible_coverage_tail_token())

    def test_last_tail_sample_completes_job_without_new_navigation(self):
        from pathlib import Path
        import tempfile
        from oomwoo_cleaning_jobs.nav_interlock import NavigationInterlock
        with tempfile.TemporaryDirectory() as tmp:
            jb=CleaningJobs(Grid(1,1,(0,)),Path(tmp)/'tail.json')
            jb.start('small',mode='whole',start_cell=0)
            po=MockActionPort()
            gate=NavigationInterlock(jb,po,coverage_source='verified_sweep',
                                     clock=lambda:self.now[0])
            gate.confirm_backend_idle(verified_no_outstanding_goals=True)
            goal=gate.dispatch(pose_cell=0,admit_route=lambda route:True)
            gate.on_goal_result(token=goal.token,outcome='succeeded')
            st=gate.acknowledge_post_goal_coverage(token=goal.token,cells=(0,),
                  producer='verified_sweep',map_digest=jb.grid.fingerprint())
            self.assertEqual(st['state'],'completed')
            self.assertIsNone(gate.eligible_coverage_tail_token())
            self.assertEqual(len(po.sent),1)
            self.assertEqual(po.cancels,[])



if __name__ == '__main__':
    unittest.main()
