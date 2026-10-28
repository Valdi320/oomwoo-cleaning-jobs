# Continuous swept-disk costmap admission - 2026-10-28

Independent implementation by Valdi320 under Apache-2.0. Parent bc3ffb07fbf0b2d824eabb5e0e67f549770555f3.
Same-scope upstream PR: https://github.com/makerspet/oomwoo/pull/79

## Reproduced failure
Previously CostmapChecker.route_admitted used 9 samples per waypoint
(centre and 8 on its circumference). A lethal or UNKNOWN 1-cm cell
at world (0.14,0.19) lies INSIDE radius 0.21m around (0.05,0.05),
but none of those nine points checked its occupancy. More critically,
a dangerous pixel halfway between adjacent coarse-grid centres was
also missed. Pre-patch tests reproduced both unsafe admissions.

## Implementation
For each adjacent 4-connected static-map cell pair, compute its whole
continuous swept circular disk (capsule) in the rotated rolling Nav2
costmap local frame. Every pixel rectangle touching that capsule is
checked with segment-to-rectangle distance; unknown or >max_cost blocks.
Map edges outside the locally observed field are rejected. Only pixels
inside the expanded local bounding box are scanned, not the whole map.
Pure-Python preflight guard, not a Nav2 default change.

## Reproduce
PYTHONPATH=src python3 -m unittest discover -s tests -p test_swept_footprint.py -q
PYTHONPATH=src python3 -m unittest discover -s tests -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -q
PYTHONPATH=src python3 -m oomwoo_cleaning_jobs.demo

Results and before/after proof: docs/evidence/swept_disk_costmap_admission_20261028.json.

## Limits
This iteration executed HOST PYTHON ONLY; no live ROS2/Gazebo run,
hardware, batteries, motors, docking, firmware or physical cleaning.
A circle is a conservative approximation, not a differential-drive
dynamic footprint. TF freshness, map version, actual robot pose,
Nav2 global/local costmap configuration, collision monitor, cliff and
bumper handling, and MCU emergency stop remain separate requirements.
Additional conservative rejections may occur in narrow or partially
observed corridors. Measure Gazebo real navigation and CPU throughput
before integration; never reduce radius merely to silence a test.
