"""Negative admission matrix: no physical cell credit with bad real signals."""
import unittest
from dataclasses import replace
from oomwoo_cleaning_jobs.physical_coverage_admission import (
    PhysicalCoverageAdmission,PhysicalCoverageNotVerified,ActuatorReading,
    LocalizationReading)


class PhysicalAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.policy=PhysicalCoverageAdmission()
        self.act=ActuatorReading(100.,True,.41,True,True)
        self.loc=LocalizationReading(100.,"map",.03,.07)
        self.base={"now_s":100.2,"measurement_stamp_s":100.1,
            "cells":(52,53,54),
            "actuator":self.act,"localization":self.loc,
            "goal_accepted":True,"job_running":True,
            "mode":"vacuum_and_mop","trusted_coverage_source":True,
            "source_map_matches":True}

    def check_bad(self,**change):
        args={**self.base,**change}
        with self.assertRaises(PhysicalCoverageNotVerified):
            self.policy.admit(**args)

    def test_all_live_verified_admits_unchanged_cells(self):
        self.assertEqual(self.policy.admit(**self.base),(52,53,54))

    def test_vacuum_off_blocks(self):
        self.check_bad(actuator=replace(self.act,vacuum_enabled=False))

    def test_low_vacuum_current_blocks(self):
        self.check_bad(actuator=replace(self.act,vacuum_current_a=.03))

    def test_nan_vacuum_current_blocks(self):
        self.check_bad(actuator=replace(self.act,vacuum_current_a=float("nan")))

    def test_mop_off_blocks(self):
        self.check_bad(actuator=replace(self.act,mop_enabled=False))

    def test_mop_not_contacting_floor_blocks(self):
        self.check_bad(actuator=replace(self.act,mop_engaged=False))

    def test_vacuum_only_not_require_mop(self):
        data={**self.base,"mode":"vacuum",
             "actuator":replace(self.act,mop_enabled=False,mop_engaged=False)}
        self.assertEqual(self.policy.admit(**data),(52,53,54))

    def test_mop_only_not_require_vacuum(self):
        data={**self.base,"mode":"mop",
             "actuator":replace(self.act,vacuum_enabled=False,vacuum_current_a=0.)}
        self.assertEqual(self.policy.admit(**data),(52,53,54))

    def test_stale_actuator_blocks(self):
        self.check_bad(actuator=replace(self.act,stamp_s=98.))

    def test_stale_localization_blocks(self):
        self.check_bad(localization=replace(self.loc,stamp_s=98.))

    def test_future_timestamp_blocks(self):
        self.check_bad(localization=replace(self.loc,stamp_s=102.))

    def test_wrong_coordinate_frame_blocks(self):
        self.check_bad(localization=replace(self.loc,frame="odom"))

    def test_position_uncertainty_blocks(self):
        self.check_bad(localization=replace(self.loc,position_stddev_m=.14))

    def test_heading_uncertainty_blocks(self):
        self.check_bad(localization=replace(self.loc,yaw_stddev_rad=.25))

    def test_untrusted_coverage_source_blocks(self):
        self.check_bad(trusted_coverage_source=False)

    def test_changed_map_blocks(self):
        self.check_bad(source_map_matches=False)

    def test_inactive_goal_blocks(self):
        self.check_bad(goal_accepted=False)

    def test_paused_job_blocks(self):
        self.check_bad(job_running=False)

    def test_duplicate_cell_claim_blocks(self):
        self.check_bad(cells=(52,52))

    def test_empty_claim_blocks(self):
        self.check_bad(cells=())

    def test_invalid_mode_blocks(self):
        self.check_bad(mode="auto")

    def test_nan_localization_uncertainty_blocks(self):
        self.check_bad(localization=replace(self.loc,yaw_stddev_rad=float("nan")))


if __name__=="__main__":
    unittest.main()
