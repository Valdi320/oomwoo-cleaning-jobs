# Physical coverage requires contemporaneous actuator + pose (2026-10-28)

Owner: Valdi320; Apache-2.0. Parent: 913519b47f9d56651ac3518bef56bf01f1c6cde8.
Linked to existing upstream PR: https://github.com/makerspet/oomwoo/pull/79

## Defect
The previous physical-coverage admission accepted cell IDs WITHOUT a coverage
timestamp. Fresh motor-current and pose telemetry obtained LATER could lend
credibility to cells actually observed earlier with the vacuum off or elsewhere.

## Fix
Require measurement_stamp_s in the physical-only admission and WAL gate.
Coverage observation, actuator and localization must share one monotonic
clock domain. All three must be within 0.65s of now; actuator/current and
pose samples must PRECEDE the coverage observation by <=0.15s (configurable
but never greater than the freshness window). No WAL mutation on failed
admission. This intentionally changes an experimental, unintegrated API.
The Gazebo meter and OOMWOO upstream ROS2 runtime remain unchanged.

## Reproduce
PYTHONPATH=src python3 -m unittest discover -s tests -p test_physical_coverage_temporal.py -q
PYTHONPATH=src python3 -m unittest discover -s tests -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -q
PYTHONPATH=src python3 -m oomwoo_cleaning_jobs.demo

Pre-fix negative test test_legacy_unstamped_claim_rejected FAIL as intended;
new implementation must PASS. Full machine results in evidence JSON.

## Nonclaims and TODO
Only host Python logic, syntax and synthetic demo tested. NO ROS2/Gazebo
runtime, physical MCU, sensor clocks, motor current, charging or actual dirt
removal. Trust boolean is NOT cryptographic authentication. Point samples
cannot prove the motor remained enabled throughout sweeping one cell.
Hardware integration needs clock synchronization with bounded uncertainty,
trusted MCU telemetry and actuator-ON intervals, followed by Gazebo and bench
safety validation. Never deploy this test contract as a motor safety system.
