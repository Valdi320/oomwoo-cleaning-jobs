"""Physical WAL fail-closed integration regressions (no robot hardware)."""
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

from oomwoo_cleaning_jobs.physical_coverage_wal_gate import PhysicalCoverageWALGate
from oomwoo_cleaning_jobs.coverage_wal import CoverageWAL, CoverageLedgerError
from oomwoo_cleaning_jobs.physical_coverage_admission import (
    ActuatorReading, LocalizationReading, PhysicalCoverageAdmission,
    PhysicalCoverageNotVerified,
)


class PhysicalCoverageWALGateTests(unittest.TestCase):
    def setUp(self):
        self.wal = Mock(spec=CoverageWAL)
        self.wal.producer = "physical"
        self.gate = PhysicalCoverageWALGate(self.wal, PhysicalCoverageAdmission())
        self.kw = dict(event_id="receipt1", cells=(4, 7), now_s=10.,
            actuator=ActuatorReading(10., True, .25, False, False),
            localization=LocalizationReading(10., "map", .02, .03),
            goal_accepted=True, job_running=True, mode="vacuum",
            trusted_coverage_source=True, source_map_matches=True)

    def test_valid_measurement_reaches_wal_once(self):
        self.gate.stage(**self.kw)
        self.wal.stage.assert_called_once_with("receipt1", (4, 7), externally_validated=True)

    def test_untrusted_source_cannot_write(self):
        self.kw["trusted_coverage_source"] = False
        with self.assertRaises(PhysicalCoverageNotVerified): self.gate.stage(**self.kw)
        self.wal.stage.assert_not_called()

    def test_inactive_goal_cannot_write(self):
        self.kw["goal_accepted"] = False
        with self.assertRaises(PhysicalCoverageNotVerified): self.gate.stage(**self.kw)
        self.wal.stage.assert_not_called()

    def test_stale_actuator_cannot_write(self):
        a = self.kw["actuator"]
        self.kw["actuator"] = ActuatorReading(8., a.vacuum_enabled, a.vacuum_current_a, a.mop_enabled, a.mop_engaged)
        with self.assertRaises(PhysicalCoverageNotVerified): self.gate.stage(**self.kw)
        self.wal.stage.assert_not_called()

    def test_wrong_map_cannot_write(self):
        self.kw["source_map_matches"] = False
        with self.assertRaises(PhysicalCoverageNotVerified): self.gate.stage(**self.kw)
        self.wal.stage.assert_not_called()

    def test_unpowered_vacuum_cannot_write(self):
        self.kw["actuator"] = ActuatorReading(10., True, 0., False, False)
        with self.assertRaises(PhysicalCoverageNotVerified): self.gate.stage(**self.kw)
        self.wal.stage.assert_not_called()

    def test_simulated_wal_forbidden(self):
        self.wal.producer = "gazebo"
        with self.assertRaises(CoverageLedgerError):
            PhysicalCoverageWALGate(self.wal, PhysicalCoverageAdmission())


if __name__ == "__main__":
    unittest.main()
