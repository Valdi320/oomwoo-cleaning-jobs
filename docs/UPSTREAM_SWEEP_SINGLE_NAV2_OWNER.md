# Reusing OOMWOO's original boustrophedon geometry under one Nav2 owner

27 October 2026 — official OOMWOO Jazzy/Gazebo test_room.
This is a **partial full-map sweep integration**, NOT proof of a
complete room or any real-world vacuuming.

## Why this matters

OOMWOO already contains a sophisticated boustrophedon planner. Replacing it
with our own naive lawnmower would duplicate work, reduce coverage quality
and risk launching TWO independent Nav2 action clients.

Instead, integration/export_upstream_coverage_plan.py imports the official
oomwoo_coverage.coverage_planner_node.CoveragePlanner installed inside
OOMWOO's development image, fetches the official map with a separate
read-only ROS2 node, initializes the source planner for geometry only and
calls its existing _plan_waypoints() directly. **We never spin that planner
or allow its built-in action executor to issue goals.**

The official test-room map produced 409 ordered sweep waypoints in
0.016 seconds of planner computation. This is the real upstream algorithm,
not a hand-written replacement or a fake list of targets.

## Conservative clearance compatibility

The official planner deliberately approaches obstacles at closer
distances than our CleaningJobs default safety clearance.

The pure-Python upstream_sweep_adapter:
- Validates source occupancy, map dimensions, resolution, map frame,
  route provenance and finite poses.
- Moves near-obstacle waypoints only into actually traversable cell
  centres, within a strict maximum distance.
- Never lowers configured robot clearance to force acceptance.
- Records all rejected points. require_complete() refuses a plan
  with unresolved waypoints: it cannot be misreported as 100% cleaning.
- Still requires the REAL current Nav2 costmap/footprint route check
  before each physical simulator NavigateToPose action.

Recorded compatibility on official test_room:

| Clearance | Allowed shift | Unique admitted endpoints | Rejected original waypoints |
| --- | --- | ---: | ---: |
| 4 cells | 18 cm | 401 | 5 |
| 4 cells | 24 cm | 401 | 5 |
| 4 cells | 35 cm | 402 | 4 |
| 5 cells | 18 cm | 391 | 5 |
| 5 cells | 24 cm | 391 | 5 |
| 5 cells | 35 cm | 392 | 4 |

Numbers reflect snapping/repeated waypoint coalescence, not
an achieved percent of physically covered floor. The remaining
4–5 original points need alternate entry paths, explicit measured-gap
treatment or individual admission review. They are NOT silently marked
cleaned or forwarded to motors.

## New measured waypoint selection

src/oomwoo_cleaning_jobs/measured_sweep_selector.py chooses from
upstream ordered waypoints, considering the independently observed
0/100 ground-truth covered grid inside the local cleaning-disk radius.
It can skip a point when the surrounding eligible cells are already
mostly externally measured clean. Nav2 goal success is NOT a meter
sample. The selector rejects mismatched map frames, altered origins,
out-of-scope targets and invalid measurements.

The current experimental persistent ROS2 job node still handles small
spot jobs. The upstream geometry is executed in a distinct bounded
test harness, not yet promoted to its public or private full-room API.

## Real Nav2 + Gazebo partial sweep test

integration/gazebo_upstream_plan_probe.py was run in the official
OOMWOO One simulator with exactly ONE Nav2 action issuer — our
NavigationInterlock / AsyncNav2Transport. We launched the original
ground_truth and coverage_meter, but NOT the upstream coverage_planner
executor. Its precomputed waypoints were consumed as advisory targets
and each was gated through our conservative static map and actual
dynamic global Nav2 costmap before dispatch.

Result: **5 accepted NavigateToPose goals, all terminal succeeded**,
then a deliberate orderly user pause, with **337 independently
measured new swept-grid centre cells** persisted in the checkpoint.

The measured progress after each goal was 85, 190, 239, 281 and
337 cells. The final job state remained paused_user; no whole-room
completion was claimed. No transport faults or fabricated cleaning
credit were reported.

Machine-readable evidence:
[evidence/oomwoo_upstream_sweep_5goals_20261027.json](evidence/oomwoo_upstream_sweep_5goals_20261027.json).

## Reproducing in a private offline simulator

1. Use the official development image docker.io/makerspet/oomwoo:jazzy-dev
   and start a private test_room Gazebo/AMCL/Nav2 session, as in
   ROS2_GAZEBO_VALIDATION.md. Use a private ROS_DOMAIN_ID, no network
   and NO other application sending Nav2 goals.
2. Launch the original unmodified oomwoo_sim_support ground_truth
   and coverage_meter but **not** coverage_planner action executor.
3. Export the upstream geometry with:

~~~sh
export PYTHONPATH=/workspace/valdi-cleaning-jobs/src
python3 /workspace/valdi-cleaning-jobs/integration/export_upstream_coverage_plan.py
~~~

The exporter prints an OOMWOO_UPSTREAM_PLAN JSON record. Save the
result object (without its log prefix) at:
 /workspace/valdi-cleaning-jobs/.local-sim/oomwoo_upstream_waypoints.json

4. Reconcile to the conservative grid:

~~~sh
python3 /workspace/valdi-cleaning-jobs/integration/verify_upstream_sweep_compat.py
~~~

5. With a FRESH isolated simulator and no old active Nav2 action,
   run the bounded probe (default 2 goals, 1..6 allowed):

~~~sh
OOMWOO_TEST_WAYPOINT_LIMIT=5 \
python3 /workspace/valdi-cleaning-jobs/integration/gazebo_upstream_plan_probe.py
~~~

A successful REAL_UPSTREAM_SWEEP JSON record includes actual Nav2
terminal results, measured coverage deltas, count of safely adapted
waypoints, and rejected original plan locations. Any failure exits
nonzero; it must not be presented as complete cleaning.

## Still needed to reach a completed room / full cleaning system

- A maintainer-reviewed ROS2 job action/service interface and full-room
  plan queue integrated into the long-running controller, not just
  the standalone trial harness.
- An exclusive and documented Nav2 goal owner: OOMWOO's upstream
  coverage_planner executor and our controller MUST NOT both issue
  motor/navigation goals simultaneously.
- Correct gap-fill retries for the unadmitted 4–5 waypoints, without
  reducing clearance or converting Nav2 success into floor coverage.
- A many-hour multi-room Gazebo scenario including actual keepout
  filters, dynamic obstacles, relocalization, restarts and docking.
- A real robot needs an independent estimator based on calibrated
  localization AND confirmed mop/vacuum actuator state, not
  simulation ground-truth coverage; physical MCU safety remains separate.

This is a verifiable integration milestone, not a claim of 100%.
