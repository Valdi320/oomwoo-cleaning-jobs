# Nav2 transit-only boundary / reactive room coverage (2026-10-29)

Maintainer direction on PR #79: OOMWOO room coverage motion is moving
to a reactive executor using cmd_vel. Nav2 should be used for
inter-area transits, docking and localization. The independent
historical cleaning-jobs simulation still uses Nav2 sweep waypoints:
this is NOT the intended future production architecture.

This change adds a backwards-compatible purpose field to Goal,
defaulting to coverage for legacy tests. The existing AsyncNav2Transport
can opt in to transit_only=True, which rejects all Nav2 goal purposes
except exact transit and dock BEFORE pre-send hooks, goal construction,
journal mutation or action-client send. The ROS2 factory forwards the
option. Production ROS2 launches have not enabled this option.

Reproduce at the pinned source commit on Linux:
  PYTHONPATH=src python3 -m unittest discover -s tests -p test_nav2_transit_only_20261029.py -v
  PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_*.py' -q
  PYTHONPATH=src python3 -O -m unittest discover -s tests -p 'test_*.py' -q

Security boundary: purpose strings must be assigned by a trusted job
dispatcher, never passed through untrusted user input. This is NOT
motor ownership, authorization, a physical stop or a real Nav2
cancellation barrier. Correctly integrated reactive coverage also
requires an exclusive cmd_vel mux, verified terminal UUIDs, fresh
backend-idle state, MCU watchdog and physical safety validation.
The canonical cleaning-job interface remains undecided.

Validation is purely synthetic Python/Linux, not newly tested in
live ROS2/Gazebo, on ARM or on a physical robot. No battery, motor
or firmware was touched. Legacy mode stays available for the existing
private simulation; strict mode intentionally rejects its sweep goals.
Revert the isolated contributor commit normally if unsuitable.
