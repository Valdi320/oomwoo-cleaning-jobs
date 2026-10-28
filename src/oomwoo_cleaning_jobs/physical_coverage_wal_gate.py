"""Opt-in physical-only gate before the durable coverage WAL.

This does NOT authenticate real sensors or command any physical motor.
Only a trusted hardware adapter may supply the source-trust predicates.
The simulation controller must keep using its own independent Gazebo meter.
"""
from __future__ import annotations

from .coverage_wal import CoverageWAL, CoverageLedgerError
from .physical_coverage_admission import (
    ActuatorReading, LocalizationReading, PhysicalCoverageAdmission,
    PhysicalCoverageNotVerified,
)


class PhysicalCoverageWALGate:
    """Reject unverified physical observations BEFORE any WAL disk mutation."""

    def __init__(self, wal: CoverageWAL, admission: PhysicalCoverageAdmission):
        if not isinstance(wal, CoverageWAL):
            raise TypeError("expected durable CoverageWAL")
        if not isinstance(admission, PhysicalCoverageAdmission):
            raise TypeError("expected physical coverage policy")
        if wal.producer != "physical":
            raise CoverageLedgerError("physical credit requires a physical-only WAL")
        self.wal = wal
        self.admission = admission

    def stage(self, event_id: str, cells: tuple[int, ...], *,
              now_s: float, measurement_stamp_s: float,
              actuator: ActuatorReading,
              localization: LocalizationReading, goal_accepted: bool,
              job_running: bool, mode: str,
              trusted_coverage_source: bool, source_map_matches: bool) -> None:
        verified = self.admission.admit(
            now_s=now_s, measurement_stamp_s=measurement_stamp_s,
            cells=cells, actuator=actuator,
            localization=localization, goal_accepted=goal_accepted,
            job_running=job_running, mode=mode,
            trusted_coverage_source=trusted_coverage_source,
            source_map_matches=source_map_matches,
        )
        # No WAL entry is ever emitted on PhysicalCoverageNotVerified.
        self.wal.stage(event_id, verified, externally_validated=True)
