"""Station transaction tests: no physical dock actions, strong pause/ACK gates."""
from pathlib import Path
import tempfile
import unittest
from oomwoo_cleaning_jobs.dock_cycle import DockCycle,DockCycleError


class SimDockCycleTests(unittest.TestCase):
    def setUp(self):
        self.scratch=tempfile.TemporaryDirectory()
        self.addCleanup(self.scratch.cleanup)
        self.file=Path(self.scratch.name)/"dock.json"
        self.d=DockCycle(self.file,job_id="whole_home",map_digest="e"*64)
        self._observation_sequence=0

    def request(self,**values):
        a={"battery_low":True,"bin_full":False,"mop_dirty":False,
           "job_paused":True,"goal_terminal":True}
        a.update(values)
        self.d.request(**a)
        return self.d.state["nonce"]

    def arrive(self):
        self.d.confirm_dock_arrival(docked=True,nav2_terminal=True,
                                    station_connected=True)

    def servicing(self):
        self.arrive()
        self.d.start_service(station_ack_nonce=self.d.state["nonce"],
                             station_ready=True)

    def sample(self,**kwargs):
        self._observation_sequence+=1
        base={"battery_fraction":.94,"bin_full":False,"mop_dirty":False,
              "station_completed":tuple(self.d.state["needed"]),
              "station_ack_nonce":self.d.state["nonce"],"docked":True,
              "observation_seq":self._observation_sequence}
        base.update(kwargs)
        return self.d.observe(**base)

    def test_battery_recharge_requires_two_stable_reports_and_confirmed_arrival(self):
        token=self.request()
        self.servicing()
        self.sample()
        self.assertEqual(self.d.state["state"],"servicing")
        with self.assertRaises(DockCycleError):
            self.d.release(nav2_idle_verified=True,fresh_pose=True,resources_healthy=True)
        self.sample()
        self.assertEqual(self.d.state["state"],"ready_to_resume")
        got=self.d.release(nav2_idle_verified=True,fresh_pose=True,resources_healthy=True)
        self.assertEqual(got,token)
        self.assertEqual(self.d.state["cycles"],1)
        self.assertEqual(self.d.state["state"],"idle")

    def test_multiple_services_one_transaction(self):
        self.request(bin_full=True,mop_dirty=True)
        self.servicing()
        self.sample(battery_fraction=.79,bin_full=True,mop_dirty=True)
        self.assertEqual(self.d.state["completed"],[])
        self.sample(battery_fraction=.95,bin_full=True,mop_dirty=True,
                    station_completed=("recharge",))
        self.assertEqual(self.d.state["completed"],["recharge"])
        self.sample(station_completed=("recharge","empty_bin"))
        self.assertEqual(self.d.state["completed"],["empty_bin","recharge"])
        self.sample()
        self.assertEqual(self.d.state["state"],"servicing")
        self.sample()
        self.assertEqual(self.d.state["state"],"ready_to_resume")
        self.d.release(nav2_idle_verified=True,fresh_pose=True,resources_healthy=True)

    def test_power_loss_after_ready_requires_new_station_proof(self):
        old_nonce=self.request()
        self.servicing()
        self.sample()
        self.sample()
        self.assertEqual(self.d.state["state"],"ready_to_resume")
        reboot=DockCycle(self.file,job_id="whole_home",map_digest="e"*64)
        self.assertEqual(reboot.state["state"],"paused_recovery")
        with self.assertRaises(DockCycleError):
            reboot.release(nav2_idle_verified=True,fresh_pose=True,
                           resources_healthy=True)
        reboot.revalidate_after_restart(
           nav2_idle_verified=True,docked=True,station_connected=True)
        self.assertNotEqual(reboot.state["nonce"],old_nonce)
        with self.assertRaises(DockCycleError):
            reboot.start_service(station_ack_nonce=old_nonce,
                                 station_ready=True)

    def test_cannot_replace_pending_ready_transaction(self):
        self.request()
        self.servicing()
        self.sample()
        self.sample()
        with self.assertRaises(DockCycleError):
            self.d.request(battery_low=True,bin_full=False,mop_dirty=False,
                           job_paused=True,goal_terminal=True)
        self.assertEqual(self.d.state["state"],"ready_to_resume")

    def test_corrupt_idle_transaction_never_autorearms(self):
        import json
        row=self.d.state
        row["nonce"]="a"*32
        self.file.write_text(json.dumps(row))
        with self.assertRaises(DockCycleError):
            DockCycle(self.file,job_id="whole_home",map_digest="e"*64)

    def test_never_resume_from_just_docking(self):
        self.request()
        self.arrive()
        with self.assertRaises(DockCycleError):
            self.d.release(nav2_idle_verified=True,fresh_pose=True,resources_healthy=True)

    def test_no_motion_before_request_requires_terminal_goal(self):
        with self.assertRaises(DockCycleError):
            self.request(goal_terminal=False)
        self.assertEqual(self.d.state["state"],"idle")

    def test_job_not_paused_denies_service(self):
        with self.assertRaises(DockCycleError):
            self.request(job_paused=False)
        self.assertEqual(self.d.state["cycles"],0)

    def test_no_real_service_needed_rejected(self):
        with self.assertRaises(DockCycleError):
            self.request(battery_low=False,bin_full=False,mop_dirty=False)

    def test_bad_station_ack_not_accepted(self):
        self.request()
        self.arrive()
        with self.assertRaises(DockCycleError):
            self.d.start_service(station_ack_nonce="old",station_ready=True)
        self.assertEqual(self.d.state["state"],"at_dock")

    def test_incomplete_battery_service_blocks_resume(self):
        self.request()
        self.servicing()
        self.sample(battery_fraction=.3)
        self.sample(battery_fraction=.5)
        self.assertEqual(self.d.state["completed"],[])
        self.assertNotEqual(self.d.state["state"],"ready_to_resume")

    def test_station_unplugged_during_service_refuses_status(self):
        self.request()
        self.servicing()
        with self.assertRaises(DockCycleError):
            self.sample(docked=False)
        self.assertEqual(self.d.state["state"],"servicing")

    def test_unexpected_service_ack_refused(self):
        self.request()
        self.servicing()
        with self.assertRaises(DockCycleError):
            self.sample(station_completed=("wash_mop",))

    def test_battery_regression_invalidates_station_readiness(self):
        self.request()
        self.servicing()
        self.sample()
        self.assertEqual(self.d.state["stable_samples"],1)
        self.sample(battery_fraction=.33)
        self.assertEqual(self.d.state["stable_samples"],0)
        self.assertEqual(self.d.state["state"],"servicing")
        self.sample()
        self.assertEqual(self.d.state["stable_samples"],1)
        self.sample()
        self.assertEqual(self.d.state["state"],"ready_to_resume")

    def test_replay_of_station_telemetry_frame_rejected(self):
        self.request()
        self.servicing()
        self.sample()
        with self.assertRaisesRegex(DockCycleError,"replayed or out of order"):
            self.d.observe(battery_fraction=.94,bin_full=False,mop_dirty=False,
              station_completed=tuple(self.d.state["needed"]),
              station_ack_nonce=self.d.state["nonce"],docked=True,
              observation_seq=1)
        self.assertEqual(self.d.state["stable_samples"],1)

    def test_station_reboot_rotates_nonce_and_refuses_old_ack(self):
        token=self.request()
        self.servicing()
        reboot=DockCycle(self.file,job_id="whole_home",map_digest="e"*64)
        reboot.revalidate_after_restart(
                 nav2_idle_verified=True,docked=True,station_connected=True)
        self.assertNotEqual(reboot.state["nonce"],token)
        with self.assertRaises(DockCycleError):
            reboot.start_service(station_ack_nonce=token,station_ready=True)

    def test_bin_or_mop_regression_invalidates_previous_ready(self):
        self.request(bin_full=True,mop_dirty=True)
        self.servicing()
        self.sample()
        self.sample()
        self.assertEqual(self.d.state["state"],"ready_to_resume")
        self.sample(bin_full=True,mop_dirty=True)
        self.assertEqual(self.d.state["state"],"servicing")
        with self.assertRaises(DockCycleError):
            self.d.release(nav2_idle_verified=True,fresh_pose=True,
                           resources_healthy=True)

    def test_ready_requires_fresh_localization_and_nav2_idle(self):
        self.request()
        self.servicing()
        self.sample();self.sample()
        for values in [
            dict(nav2_idle_verified=False,fresh_pose=True,resources_healthy=True),
            dict(nav2_idle_verified=True,fresh_pose=False,resources_healthy=True),
            dict(nav2_idle_verified=True,fresh_pose=True,resources_healthy=False)]:
            with self.subTest(values=values),self.assertRaises(DockCycleError):
                self.d.release(**values)
        self.assertEqual(self.d.state["cycles"],0)

    def test_interrupted_service_restart_fails_closed_and_revalidates(self):
        self.request()
        self.servicing()
        self.sample()
        other=DockCycle(self.file,job_id="whole_home",map_digest="e"*64)
        self.assertEqual(other.state["state"],"paused_recovery")
        with self.assertRaises(DockCycleError):
            other.revalidate_after_restart(
                 nav2_idle_verified=False,docked=True,station_connected=True)
        other.revalidate_after_restart(
                 nav2_idle_verified=True,docked=True,station_connected=True)
        self.assertEqual(other.state["state"],"at_dock")
        self.assertEqual(other.state["completed"],[])
        other.start_service(station_ack_nonce=other.state["nonce"],
                            station_ready=True)
        other.observe(battery_fraction=.91,bin_full=False,mop_dirty=False,
                 station_completed=("recharge",),
                 station_ack_nonce=other.state["nonce"],docked=True,
                 observation_seq=1)
        other.observe(battery_fraction=.91,bin_full=False,mop_dirty=False,
                 station_completed=("recharge",),
                 station_ack_nonce=other.state["nonce"],docked=True,
                 observation_seq=2)
        self.assertEqual(other.state["state"],"ready_to_resume")

    def test_changed_map_fingerprint_refuses_stale_station(self):
        self.request()
        with self.assertRaises(DockCycleError):
            DockCycle(self.file,job_id="whole_home",map_digest="f"*64)

    def test_corrupted_service_checkpoint_refuses_rearm(self):
        self.file.write_text('{')
        with self.assertRaises(DockCycleError):
            DockCycle(self.file,job_id="whole_home",map_digest="e"*64)

    def test_multiple_recharge_cycles_per_job(self):
        for _ in range(3):
            self.request()
            self.servicing();self.sample();self.sample()
            self.d.release(nav2_idle_verified=True,fresh_pose=True,resources_healthy=True)
        self.assertEqual(self.d.state["cycles"],3)

    def test_service_cycle_budget_enforced(self):
        d=DockCycle(Path(self.scratch.name)/"bounded.json",
                    job_id="limited",map_digest="f"*64,max_cycles=1)
        d.request(battery_low=True,bin_full=False,mop_dirty=False,
                  job_paused=True,goal_terminal=True)
        d.confirm_dock_arrival(docked=True,nav2_terminal=True,station_connected=True)
        d.start_service(station_ack_nonce=d.state["nonce"],station_ready=True)
        for observation in range(1,3):
            d.observe(battery_fraction=.9,bin_full=False,mop_dirty=False,
                      station_completed=("recharge",),
                      station_ack_nonce=d.state["nonce"],docked=True,
                      observation_seq=observation)
        d.release(nav2_idle_verified=True,fresh_pose=True,resources_healthy=True)
        with self.assertRaises(DockCycleError):
            d.request(battery_low=True,bin_full=False,mop_dirty=False,
                      job_paused=True,goal_terminal=True)

    def test_concurrent_writer_revision_protection(self):
        clone=DockCycle(self.file,job_id="whole_home",map_digest="e"*64)
        self.request()
        with self.assertRaises(DockCycleError):
            clone.request(battery_low=True,bin_full=False,mop_dirty=False,
                          job_paused=True,goal_terminal=True)

if __name__=="__main__":
    unittest.main()
