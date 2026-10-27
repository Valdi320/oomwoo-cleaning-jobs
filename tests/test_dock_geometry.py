"""Verified upstream OOMWOO dock_drive topic ordering and service nonclaims."""
import unittest
from oomwoo_cleaning_jobs.dock_geometry import DockGeometryWitness,DockGeometryError


class DockGeometryWitnessTests(unittest.TestCase):
    def setUp(self):
        self.g=DockGeometryWitness()
        self.g.arm(now=10.,epoch="dock-A",nav2_terminal=True,
                   isolated_dock_cmd_owner=True)

    def finish(self):
        self.g.state(now=10.1,mode="SEARCH")
        self.g.state(now=10.4,mode="GOTO")
        self.g.state(now=11.,mode="DONE")
        self.g.docked(now=11.1,value=True)
        for k in range(10):
            self.g.odom(now=11.2+k*.1,linear_mps=0.,angular_rps=0.)

    def test_completed_docking_only_proves_geometry_not_station_service(self):
        self.finish()
        r=self.g.report(now=12.11)
        self.assertTrue(r["geometry_docked"])
        self.assertFalse(r["physical_station_contact_proven"])
        self.assertFalse(r["electrical_charging_proven"])
        self.assertFalse(r["bin_empty_proven"])
        self.assertFalse(r["mop_wash_proven"])

    def test_latched_old_docked_true_cannot_validate_new_attempt(self):
        self.g.docked(now=10.05,value=True)
        self.assertFalse(self.g.report(now=10.06)["geometry_docked"])

    def test_done_without_fresh_search_is_untrusted(self):
        with self.assertRaises(DockGeometryError):
            self.g.state(now=10.1,mode="DONE")

    def test_stale_odometry_disables_confirmation(self):
        self.finish()
        self.assertFalse(self.g.report(now=13.)["geometry_docked"])

    def test_motion_resets_required_stable_stop_window(self):
        self.g.state(now=10.1,mode="SEARCH")
        self.g.state(now=10.4,mode="DONE")
        self.g.docked(now=10.5,value=True)
        for k in range(5):
            self.g.odom(now=10.6+k*.1,linear_mps=0.,angular_rps=0.)
        self.g.odom(now=11.1,linear_mps=.15,angular_rps=0.)
        self.assertFalse(self.g.report(now=11.2)["geometry_docked"])

    def test_restart_new_epoch_cannot_reuse_old_done(self):
        self.finish()
        self.assertTrue(self.g.report(now=12.11)["geometry_docked"])
        self.g.arm(now=12.2,epoch="dock-B",nav2_terminal=True,
                   isolated_dock_cmd_owner=True)
        self.assertFalse(self.g.report(now=12.3)["geometry_docked"])

    def test_unowned_docking_never_arms(self):
        x=DockGeometryWitness()
        with self.assertRaises(DockGeometryError):
            x.arm(now=0.,epoch="stray",nav2_terminal=False,
                  isolated_dock_cmd_owner=True)

    def test_lost_dock_drive_to_idle_is_fail_closed(self):
        self.g.state(now=10.1,mode="SEARCH")
        self.g.state(now=10.4,mode="IDLE")
        self.assertFalse(self.g.report(now=10.5)["geometry_docked"])

    def test_backwards_clock_latches_failure(self):
        self.g.state(now=10.5,mode="SEARCH")
        with self.assertRaises(DockGeometryError):
            self.g.state(now=10.4,mode="DONE")
        self.assertFalse(self.g.active)

    def test_nonfinite_odom_latches_failure(self):
        self.g.state(now=10.1,mode="SEARCH")
        with self.assertRaises(DockGeometryError):
            self.g.odom(now=10.2,linear_mps=float("nan"),angular_rps=0.)
        self.assertFalse(self.g.active)

if __name__=="__main__":
    unittest.main()
