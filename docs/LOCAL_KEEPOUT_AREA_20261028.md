# Rolling local Nav2 keepout: full observable area proof (2026-10-28)

Scope: Valdi320 Apache-2.0 experimental cleaning-jobs only. Based on commit
b6705a0f7eb1602915007e563e1a8d39df39804b. AI-assisted.
Upstream reference/pointer: https://github.com/makerspet/oomwoo/pull/79

## Reproduced defect

During a paused live keepout-mask edit, ZoneUpdateTransaction.verify_sample
checked the local rolling costmap at each forbidden mask-cell centre only.
If the mask uses 0.10 m cells and the local costmap uses 0.05 m cells,
only the subcell containing the centre could be lethal: three other local
pixels were FREE yet the controller counted a post-filter observation.
This was an EXPERIMENTAL APPLICATION CONTRACT BUG, not proof that Nav2
itself drives through a forbidden zone.

## Engineering change

A new dependency-free local_keepout_proof module compares actual rotated
raster cell rectangles using a separating-axis overlap predicate. Every
observed positive-area intersection with a forbidden mask rectangle must
be lethal >=95, including thin boundary overlaps. The rolling costmap is
not required to see remote forbidden cells; an independent global full-area
proof still checks the whole zone. To preserve the previous minimum local
visibility rule, at least one forbidden mask-cell centre must be present.
An aligned-grid fast path and 2,000,000-comparison cap avoid unbounded
admission work; budget exhaustion fails CLOSED. The edit still needs two
monotonic independent local/global observation sequences, paused job and
terminal goal. No change to upstream OOMWOO code or safety-critical MCU.

## Reproduce

From this published contributor branch:

PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_local_keepout_proof.py' -v
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_zone_revision.py' -v
PYTHONPATH=src python3 -m unittest discover -s tests -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -q
PYTHONPATH=src python3 -m oomwoo_cleaning_jobs.demo

Evidence: docs/evidence/local_keepout_area_20261028.json.

## Exact limits

All new tests use SYNTHETIC Raster values executed on the owner's Linux
host, not ROS2 middleware, Nav2/Gazebo, measured LiDAR costmaps or motors.
No physical braking, wireless telemetry, battery charging or MCU STOP was
tested. The numerical SAT geometry is conservative near boundaries;
performance and latency on Raspberry Pi are unmeasured. The source of
local costmap samples is trusted by the caller, not cryptographically
authenticated. Real ROS2 Gazebo hot-mask observations and maintainer
single-nav-owner review remain required for production integration.
