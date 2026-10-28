# Malformed ROS2 costmap/TF boundary (2026-10-28)

Owner: Valdi320 (Apache-2.0), AI-assisted by ChatGPT. Base: 63f00848724200adc680f8cb555fe7b3bb0f0bd5. Upstream pointer PR: https://github.com/makerspet/oomwoo/pull/79

## Defect and correction
The experimental rolling_costmap_in_map could let AttributeError or TypeError escape when a ROS2-like OccupancyGrid or map<-odom TF lacked fields, used null data, or nonnumeric coordinates. A caller catching CostmapFrameError to stop navigation could miss that malformed-input failure. A public wrapper now converts only AttributeError/TypeError/OverflowError into CostmapFrameError, while preserving existing geometry, clock, and frame checks.

## Reproduce on pure Python host
    PYTHONPATH=src python3 -m unittest discover -s tests -p test_costmap_malformed_payload.py -v
    PYTHONPATH=src python3 -m unittest discover -s tests -q
    PYTHONPATH=src python3 -O -m unittest discover -s tests -q
    PYTHONPATH=src python3 -m oomwoo_cleaning_jobs.demo

Negative-control before change: 12 unexpected errors, 3 controls pass. After: 15/15 focal, 556/556 standard and optimized regressions, demo PASS, syntax PASS. Evidence: docs/evidence/ros_costmap_malformed_payload_20261028.json.

## Limits
Only synthetic duck-typed ROS messages under host Python, not ROS2/Nav2/Gazebo process tests or motor/MCU hardware tests. Production must catch CostmapFrameError and independently enforce a verified STOP on the actual motor driver and safety MCU; this patch neither authenticates messages nor ensures motor STOP. No upstream or third-party files changed; zero new dependencies/BOM.
