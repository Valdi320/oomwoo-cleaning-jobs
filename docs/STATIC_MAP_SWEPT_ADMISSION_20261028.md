# Static map swept-footprint admission (2026-10-28)

Owner: Valdi320 / oomwoo-cleaning-jobs (Apache-2.0; ChatGPT-assisted).
Base commit: e59247eaa48c3d3880f2fd998ddcdb4869bab21d. Existing upstream pointer: makerspet/oomwoo PR #79.

## Reproduced defect and correction

A static occupancy-grid route checked FREE at its centre cells but checked the continuous circular footprint only in the live Nav2 costmap. When the live rolling map was all-clear/stale, a saved wall, unknown static pixel, or floor beyond the saved static map could be admitted even if the robot circle would intersect it.

Now the exact existing swept disk test is called on the saved static grid first with max_cost=0 (only occupancy 0 is safe), then on the live Nav2 costmap with its configured max_cost. This preserves the existing cached-safe-pixel fast path. It adds no dependencies or motor bindings.

## Reproduce

    PYTHONPATH=src python -m unittest discover -s tests -p test_static_swept_admission.py -v
    PYTHONPATH=src python -m unittest discover -s tests -q
    PYTHONPATH=src python -O -m unittest discover -s tests -q
    PYTHONPATH=src python -m oomwoo_cleaning_jobs.demo

Prior code: 5/8 expected failing assertions (2 safe controls passed). After change: focal 8/8 PASS, full 531/531 PASS, Python -O 531/531 PASS, demo PASS.
Machine evidence: docs/evidence/static_map_swept_admission_20261028.json.

## Exact limits

Python-only synthetic rasters on host CPU, NOT ROS2/Gazebo or physical navigation/cleaning. No freshness measurement, calibrated chassis shape, LiDAR, STM32, wheel motor, docking or battery tested. This is a conservative admission predicate, not a substitute for Nav2 collision monitor or safety MCU. Additional static-map checks cost CPU; measure on a target SBC before adoption. Border rejection can block an incompletely mapped room by design. No official or third-party files changed.
