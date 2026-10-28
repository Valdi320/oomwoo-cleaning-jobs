"""Pure-Python temporal integrity tests; no hardware."""
import unittest
from dataclasses import replace
from unittest.mock import Mock
from oomwoo_cleaning_jobs.physical_coverage_admission import (
    PhysicalCoverageAdmission, PhysicalCoverageNotVerified, ActuatorReading,
    LocalizationReading)
from oomwoo_cleaning_jobs.physical_coverage_wal_gate import PhysicalCoverageWALGate
from oomwoo_cleaning_jobs.coverage_wal import CoverageWAL

class PhysicalTimeTests(unittest.TestCase):
    def setUp(self):
        self.p=PhysicalCoverageAdmission()
        self.a=ActuatorReading(100.,True,.4,False,False)
        self.l=LocalizationReading(100.,"map",.02,.03)
        self.kw=dict(now_s=100.3,measurement_stamp_s=100.1,cells=(4,5),
            actuator=self.a,localization=self.l,goal_accepted=True,
            job_running=True,mode="vacuum",trusted_coverage_source=True,
            source_map_matches=True)
    def reject(self,**changes):
        with self.assertRaises(PhysicalCoverageNotVerified):
            self.p.admit(**{**self.kw,**changes})
    def test_legacy_unstamped_claim_rejected(self):
        kw=self.kw.copy();kw.pop("measurement_stamp_s")
        with self.assertRaises(TypeError):self.p.admit(**kw)
    def test_valid_contemporaneous_sample(self):
        self.assertEqual(self.p.admit(**self.kw),(4,5))
    def test_actuator_sample_after_coverage_rejected(self):
        self.reject(actuator=replace(self.a,stamp_s=100.2))
    def test_localization_sample_after_coverage_rejected(self):
        self.reject(localization=replace(self.l,stamp_s=100.2))
    def test_old_actuation_relative_to_measurement_rejected(self):
        self.reject(measurement_stamp_s=100.3)
    def test_old_localization_relative_to_measurement_rejected(self):
        self.reject(measurement_stamp_s=100.3,
                    actuator=replace(self.a,stamp_s=100.25))
    def test_old_coverage_laundered_by_new_sensors_rejected(self):
        self.reject(now_s=101.1,measurement_stamp_s=100.,
                    actuator=replace(self.a,stamp_s=101.),
                    localization=replace(self.l,stamp_s=101.))
    def test_future_measurement_rejected(self):
        self.reject(measurement_stamp_s=100.5)
    def test_nan_measurement_rejected(self):
        self.reject(measurement_stamp_s=float("nan"))
    def test_bool_measurement_rejected(self):
        self.reject(measurement_stamp_s=True)
    def test_negative_measurement_rejected(self):
        self.reject(measurement_stamp_s=-1.)
    def test_negative_clock_rejected(self):
        self.reject(now_s=-1.)
    def test_skew_larger_than_max_age_invalid(self):
        with self.assertRaises(ValueError):
            PhysicalCoverageAdmission(max_age_s=.65,max_sample_skew_s=.9)
    def test_bad_measurement_cannot_reach_wal(self):
        wal=Mock(spec=CoverageWAL);wal.producer="physical"
        gate=PhysicalCoverageWALGate(wal,self.p)
        kw=self.kw.copy();cells=kw.pop("cells");kw["measurement_stamp_s"]=100.3
        with self.assertRaises(PhysicalCoverageNotVerified):
            gate.stage("late",cells,**kw)
        wal.stage.assert_not_called()
    def test_verified_measurement_reaches_wal(self):
        wal=Mock(spec=CoverageWAL);wal.producer="physical"
        gate=PhysicalCoverageWALGate(wal,self.p)
        kw=self.kw.copy();cells=kw.pop("cells")
        gate.stage("aligned",cells,**kw)
        wal.stage.assert_called_once_with("aligned",(4,5),externally_validated=True)
