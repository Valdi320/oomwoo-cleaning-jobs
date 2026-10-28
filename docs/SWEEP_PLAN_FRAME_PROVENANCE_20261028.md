# Fail closed on stale plan after ROS map-origin changes

## Bug and adversarial pre-fix reproduction
The same occupancy and Grid fingerprint can be expressed by a different
ROS OccupancyGrid origin (translation or yaw). Adapted source waypoints
encode *indices* while the physical target of each index depends on that
origin. A persisted old plan was silently accepted with a new map transform.
On the exact baseline SHA 8123edd8, 3/5 new focused tests FAILED
before the change: translated-map and rotated-map selectors accepted a
stale plan, and a durable campaign wrote a new session for a foreign map.

## Fix
SafeSweepPlan now binds an immutable SHA-256 geometry signature of the
source Raster frame, dimensions, resolution, origin x/y and yaw, alongside
the original Grid fingerprint. The plan digest also includes this signature.
Both MeasuredSweepSelector and SweepCampaign reject any mismatch before
selection or writing a checkpoint. Exact binary floats fail closed even
for small transform changes; this may require re-adapting equivalent
maps after a round-trip through a ROS serialization boundary.

## Reproduce (synthetic headless only)
```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_sweep_plan_geometry_provenance_20261028.py' -v
PYTHONPATH=src python3 -m unittest discover -s tests -q
PYTHONPATH=src python3 -O -m unittest discover -s tests -q
```
Machine-readable results: docs/evidence/sweep_plan_frame_binding_20261028.json

## Compatibility / rollback / outstanding
Existing in-memory SafeSweepPlan construction must use adapt_upstream_plan;
old dataclass objects and persisted campaign plan digests are intentionally
not compatible. Re-adapt plans from the currently authenticated map and
start new checkpoints under external Nav2 reconciliation; never silently
claim prior coverage. A reviewer can revert this single source commit to
restore old format, but that reintroduces stale physical-frame acceptance.

**Limits:** these are Python unit tests using mock geometry and filesystem
checkpoints. No live ROS2 Nav2, Gazebo, SLAM, MCU, docking, physical
coverage or motor tests were run. ROS2 plan origin freshness and action
ownership require system integration and maintainer review.
