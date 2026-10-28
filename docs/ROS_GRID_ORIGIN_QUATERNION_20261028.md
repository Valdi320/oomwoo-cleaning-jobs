# Reject malformed ROS OccupancyGrid origin quaternions (2026-10-28)

Owner: Valdi320/oomwoo-cleaning-jobs (Apache-2.0). Follow-up to upstream OOMWOO PR #79, base commit 2762d6555e9ff69d1b0d1a658ba40f666654931d. No third-party or makerspet project files modified.

## Defect reproduced
Raster.from_ros silently interpreted a zero quaternion as yaw 0, and also accepted a scaled quaternion or one with substantial non-planar rotation. Invalid grid placement could enter a route preflight with plausible yet incorrect world coordinates. The test file was run against the unmodified base code first, reproducing the incorrect acceptance.

## Fix
ROS origin components must be finite numbers (not bool or missing). Quaternion norm must be 0.995..1.005, and abs(x) and abs(y) must be at most 0.03, consistent with the existing live map<-odom TF yaw validator. Invalid inputs raise AdmissionError. The existing map-framed Nav2 costmap converter wraps that error as CostmapFrameError. Valid planar origins, including 90-degree rotations and equivalent quaternion signs, remain admitted without map-cell coordinate drift.

## Reproduction
Run from repository root with Python >=3.10:

    PYTHONPATH=src python -m unittest discover -s tests -p test_ros_grid_origin_orientation.py -v
    PYTHONPATH=src python -m unittest discover -s tests -q
    PYTHONPATH=src python -O -m unittest discover -s tests -q
    PYTHONPATH=src python -m oomwoo_cleaning_jobs.demo

Result counts, negative controls and precise evidence level are in docs/evidence/ros_grid_origin_quaternion_20261028.json.

## Limits
Synthetic Python ROS-message-shaped inputs and pure-Python regression on the contributor's PC. Not a real ROS2/Gazebo run and not a hardware, motor, battery, MCU or collision-stop test. The inherited small x/y tolerance is an explicit engineering approximation; physical map alignment and costmap freshness need separate verification.

AI-assisted implementation; the OOMWOO maintainer retains ownership of project integration decisions.
